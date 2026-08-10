#include "game_io.h"
#include "../config_manager/config_manager.h"
#include "../binding_service/binding_service.h"
#include "../sensor_link/sensor_link.h"
#include "../latency_stats.h"
#include "../self_heal/self_heal.h"
#include "../usb_debug.h"
#include "../../protocol/psoc/psoc.h"
#include <pico/stdlib.h>

GameIoService* GameIoService::_instance = nullptr;

void GameIoService::SerialPublishState::reset() {
    _flags.flags = 0u;
    _last_sent.clear();
    _last_aggregate.clear();
    _last_recorded.clear();
    _last_rate_send_us = 0u;
    _remaining_extra_sends = 0u;
    for (uint16_t slot = 0u; slot < kAggregateSlots; ++slot) {
        _samples[slot].time_ms = UINT32_MAX;
        _samples[slot].state = 0u;
    }
}

bool GameIoService::SerialPublishState::rate_limited(
    uint32_t now_us, const SerialPublishSettings& settings) const {
    if (!settings.bits.rate_limit_enabled || _last_rate_send_us == 0u) return false;
    const uint32_t interval_us = 1000000u / settings.rate_limit_hz;
    return interval_us != 0u && now_us - _last_rate_send_us < interval_us;
}

void GameIoService::SerialPublishState::record(
    uint32_t now_us, const Mai2Serial_TouchState& sample) {
    const uint32_t now_ms = now_us / 1000u;
    Sample& slot = _samples[now_ms % kAggregateSlots];
    slot.time_ms = now_ms;
    slot.state = sample.raw;
    _last_recorded = sample;
}

Mai2Serial_TouchState GameIoService::SerialPublishState::value(
    uint32_t now_us, uint16_t window_ms) {
    if (window_ms == 0u) return _last_recorded;

    const uint32_t now_ms = now_us / 1000u;
    uint8_t votes[35] = {};
    uint8_t count = 0u;
    for (uint16_t index = 0u; index < kAggregateSlots; ++index) {
        const Sample& current = _samples[index];
        if (current.time_ms == UINT32_MAX || now_ms - current.time_ms > window_ms) continue;
        ++count;
        for (uint8_t bit = 0u; bit < 35u; ++bit) {
            votes[bit] += static_cast<uint8_t>((current.state >> bit) & 1u);
        }
    }
    if (count == 0u) return _flags.bits.has_last_aggregate ? _last_aggregate : _last_recorded;

    Mai2Serial_TouchState result;
    result.clear();
    for (uint8_t bit = 0u; bit < 35u; ++bit) {
        const uint8_t doubled_votes = static_cast<uint8_t>(votes[bit] * 2u);
        const bool keep_last = doubled_votes == count && _flags.bits.has_last_aggregate &&
                               ((_last_aggregate.raw >> bit) & 1u) != 0u;
        if (doubled_votes > count || keep_last) result.raw |= (uint64_t(1u) << bit);
    }
    _last_aggregate = result;
    _flags.bits.has_last_aggregate = 1u;
    return result;
}

bool GameIoService::SerialPublishState::should_send(
    const Mai2Serial_TouchState& state, const SerialPublishSettings& settings) const {
    const bool changed = !_flags.bits.has_last_sent || state.raw != _last_sent.raw;
    return !settings.bits.send_only_on_change || changed || _remaining_extra_sends != 0u;
}

void GameIoService::SerialPublishState::note_send_success(
    const Mai2Serial_TouchState& state, uint32_t now_us, const SerialPublishSettings& settings) {
    const bool changed = !_flags.bits.has_last_sent || state.raw != _last_sent.raw;
    if (settings.bits.send_only_on_change) {
        if (changed) {
            _remaining_extra_sends = settings.extra_send;
        } else if (_remaining_extra_sends != 0u) {
            --_remaining_extra_sends;
        }
    } else if (changed) {
        _remaining_extra_sends = settings.extra_send;
    }
    _last_sent = state;
    _flags.bits.has_last_sent = 1u;
    if (settings.bits.rate_limit_enabled) _last_rate_send_us = now_us;
}

GameIoService::GameIoService()
    : _serial_uart(HAL_USB_Device::getInstance(), UsbCdcPort::CDC_SERIAL),
      _light_uart(HAL_USB_Device::getInstance(), UsbCdcPort::CDC_LIGHT),
      _serial(&_serial_uart),
      _light(&_light_uart, ConfigManager::get_uint8("led.node_id")),
      _light_rgb{},
      _light_generation(0),
      _led_map_ready(false),
      _initialized(false),
      _touch_delay_units(0),
      _delay_refresh_us(0),
      _serial_reset_requests(0) {
    _serial_publish_settings.clear();
    _serial_publish.reset();
}

// 懒构造无锁: 调用点(loop 内的 task、以及 UsbComm::update 同步派发的 LED_*/MAI2_* host_cmd 处理)
// 全在 core0 单线程; core1 只跑 Psoc::core1_run, 不进入本服务, 故首次 new 无竞态。
// 若将来把 host_cmd 派发移到 core1 或中断上下文, 此处必须改为预创建或加锁。
GameIoService* GameIoService::getInstance() {
    if (_instance == nullptr) _instance = new GameIoService();
    return _instance;
}

bool GameIoService::mai2_touch_sending() const {
    return _serial.sending_active();
}

bool GameIoService::init(UsbWorkMode mode) {
    if (_initialized) return true;
    if (mode != UsbWorkMode::WORK_SERIAL) return false;

    const uint32_t serial_baud = ConfigManager::get_uint32("comm.serial_baud");
    const uint32_t light_baud = ConfigManager::get_uint32("comm.light_baud");
    if (!_serial_uart.init(255, 255, serial_baud) || !_light_uart.init(255, 255, light_baud)) {
        deinit();
        return false;
    }

    Mai2Serial_Config serial_config;
    serial_config.baud_rate = serial_baud;
    _serial.set_config(serial_config);

    Mai2Light_Config light_config;
    light_config.baud_rate = light_baud;
    light_config.node_id = ConfigManager::get_uint8("led.node_id");
    _light.set_config(light_config);

    if (!_serial.init() || !_light.init()) {
        deinit();
        return false;
    }
    _serial.set_command_callback(_handle_mai2_command);

    // WS2812 输出为辅路: 建链失败(PIO 状态机/指令内存耗尽)不拖垮触控与串口, 但返回值必须留下,
    // 由 LED_GET 状态字的每链 ready 位与故障分档如实暴露, 不静默假装成功。
    _led_map_ready = LedMapService::getInstance()->init();

    _consume_light_state();

    // 触控延迟线预热: 全环清 0(无触摸), 读入初始延迟片数。
    _touch_delay.reset(0);
    _serial_publish.reset();
    _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
    _refresh_serial_publish_settings();
    _delay_refresh_us = 0;

    _initialized = true;
    return true;
}

void GameIoService::register_host_cmds() {
    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::MAI2_GET_STATE, _handle_mai2_get_state);
    dispatcher->register_handler(HostCmd::MAI2_SET_SEND_EN, _handle_mai2_set_send_en);
    // 灯效域同样与 init 分离: HID 模式下映射表/灯珠数仍可读写, 灯板状态如实回 STOPPED。
    dispatcher->register_handler(HostCmd::LED_GET, _handle_led_get);
    dispatcher->register_handler(HostCmd::LED_SET_REGION, _handle_led_set_region);
    dispatcher->register_handler(HostCmd::LED_PREVIEW, _handle_led_preview);
}

void GameIoService::_handle_led_get(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    GameIoService* self = getInstance();
    LedMapService* led_map = LedMapService::getInstance();
    Mai2Light_Config light_config;
    self->_light.get_config(light_config);
    const Mai2Light_Stats& stats = self->_light.get_stats();

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::LED_GET);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;

    // 状态字(byte0)在 96B 定长内复用高位承载 WS2812 诊断, 不新增字段:
    //   bit0..3 = Mai2Light 状态机(0=STOPPED 1=READY 2=RUNNING)
    //   bit4/bit5 = 灯链0/灯链1 初始化就绪
    //   bit6..7 = 初始化故障分档(0=无 1=PIO 2=链0 3=链1)
    uint16_t n = 0;
    uint8_t status = static_cast<uint8_t>(self->_light.get_status()) & 0x0Fu;
    if (led_map->chain_ready(0u)) status |= 0x10u;
    if (led_map->chain_ready(1u)) status |= 0x20u;
    status |= (uint8_t)((led_map->init_fault() & 0x03u) << 6);
    r.payload[n++] = status;
    // byte1 同样在 96B 定长内复用高位承载预览链路诊断, 不新增字段(旧上位机只看 bit0, 语义不变):
    //   bit0    = Mai2Light 应答使能(原语义)
    //   bit1    = 预览掩码非空(预览色正在覆盖协议色)
    //   bit2    = LedMapService 已 init(灯链就绪, 生效色会被刷出)
    //   bit3    = _refresh 至少执行过一次(灯服务确实被 task 到了)
    //   bit4..7 = _refresh 实跑次数低 4 位(连续两次 LED_GET 该值不变 = 灯服务已停摆)
    uint8_t diag = self->_light.response_enabled() ? 0x01u : 0x00u;
    if (led_map->preview_active()) diag |= 0x02u;
    if (led_map->is_ready()) diag |= 0x04u;
    const uint32_t refresh_count = led_map->refresh_count();
    if (refresh_count != 0u) diag |= 0x08u;
    diag |= (uint8_t)((refresh_count & 0x0Fu) << 4);
    r.payload[n++] = diag;
    r.payload[n++] = MAI2LIGHT_NUM_LEDS;

    // 当前色 = 实际输出色(预览生效时为预览色), 与灯链所见一致。
    const uint8_t* rgb = led_map->effective_rgb();
    for (uint8_t i = 0; i < MAI2LIGHT_NUM_LEDS * 3u; i++) r.payload[n++] = rgb[i];

    const LedMapEntry* table = led_map->map_table();
    for (uint8_t unit = 0; unit < MAI2LIGHT_NUM_LEDS; unit++) {
        const LedMapEntry& e = table[unit];
        const bool mapped = e.mapped();
        r.payload[n++] = mapped ? e.channel : LEDMAP_CH_NONE;
        r.payload[n++] = mapped ? (uint8_t)(e.start & 0xFFu) : 0u;
        r.payload[n++] = mapped ? (uint8_t)((e.start >> 8) & 0xFFu) : 0u;
        r.payload[n++] = mapped ? e.count : 0u;
    }

    // HID 模式下灯板未 init(get_config 只有构造默认值), 此时如实回配置里的波特率而非默认值。
    const uint32_t baud = self->_light.is_ready() ? light_config.baud_rate
                                                 : ConfigManager::get_uint32("comm.light_baud");
    const uint32_t words[3] = { baud, stats.rx_frames, stats.sum_errors };
    for (uint8_t w = 0; w < 3u; w++) {
        r.payload[n++] = (uint8_t)(words[w] & 0xFFu);
        r.payload[n++] = (uint8_t)((words[w] >> 8) & 0xFFu);
        r.payload[n++] = (uint8_t)((words[w] >> 16) & 0xFFu);
        r.payload[n++] = (uint8_t)((words[w] >> 24) & 0xFFu);
    }
    for (uint8_t ch = 0; ch < LEDMAP_CHANNEL_COUNT; ch++) {
        const uint16_t count = led_map->chain_count(ch);
        r.payload[n++] = (uint8_t)(count & 0xFFu);
        r.payload[n++] = (uint8_t)((count >> 8) & 0xFFu);
    }
    // ★尾部追加 byte96 = 当前【已生效】亮度★(同 KBD_GET_STATE / AUTO_TUNE_PROGRESS 的追加先例:
    // 前 96 字节一字未动, 只读 96 字节的旧上位机不受影响)。
    // 追加它的理由: led.ws_brightness 写进 KV 只代表"配置改了", 而灯链上真正在用的是
    // LedMapService 自己那份 _brightness —— 两者此前无从对账, "亮度到底生效没有"只能靠肉眼看灯。
    // 有了这个字节, 批量保存后的即时生效就成了可验证事实(写入后立刻 LED_GET 比对即可)。
    r.payload[n++] = led_map->applied_brightness();

    r.len = n;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void GameIoService::_handle_led_set_region(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // payload = [unit(u8), ch(u8: 0/1 或 0xFF 解除), start(u16 LE), count(u8)] × n
    constexpr uint16_t ITEM_LEN = 5u;
    if (frame.len == 0u || (frame.len % ITEM_LEN) != 0u) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "led_set_region payload must be [unit,ch,start(u16),count] x n", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    LedMapService* led_map = LedMapService::getInstance();
    LedMapEntry table[LEDMAP_UNIT_COUNT];
    for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) table[unit] = led_map->map_table()[unit];

    const uint16_t items = (uint16_t)(frame.len / ITEM_LEN);
    for (uint16_t i = 0; i < items; i++) {
        const uint8_t* item = &frame.payload[i * ITEM_LEN];
        if (item[0] >= LEDMAP_UNIT_COUNT) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "led_set_region unit out of range", resp, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        LedMapEntry& e = table[item[0]];
        if (item[1] == LEDMAP_CH_NONE) {
            e.clear();
            continue;
        }
        e.channel = item[1];
        e.start = (uint16_t)((uint16_t)item[2] | ((uint16_t)item[3] << 8));
        e.count = item[4];
    }

    // 越界 / 重叠 / channel 非法 → 整批不生效(apply_map 内先校验全表再落地)。
    if (!led_map->apply_map(table)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "led_set_region rejected: segment out of chain or overlapping", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void GameIoService::_handle_led_preview(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // payload = [unit(u8: 0xFF=全部), r, g, b] × n
    constexpr uint16_t ITEM_LEN = 4u;
    if (frame.len == 0u || (frame.len % ITEM_LEN) != 0u) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "led_preview payload must be [unit,r,g,b] x n", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    LedMapService* led_map = LedMapService::getInstance();
    const uint16_t items = (uint16_t)(frame.len / ITEM_LEN);
    for (uint16_t i = 0; i < items; i++) {
        const uint8_t* item = &frame.payload[i * ITEM_LEN];
        led_map->preview_unit(item[0], item[1], item[2], item[3]);
    }
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void GameIoService::_handle_mai2_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    GameIoService* self = getInstance();
    const uint32_t baud = self->_serial.get_config().baud_rate;
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::MAI2_GET_STATE);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    r.payload[0] = self->mai2_touch_sending() ? 1 : 0;
    r.payload[1] = static_cast<uint8_t>(self->_serial.get_status());  // 0=STOPPED 1=READY 2=RUNNING
    r.payload[2] = (uint8_t)(baud & 0xFF);
    r.payload[3] = (uint8_t)((baud >> 8) & 0xFF);
    r.payload[4] = (uint8_t)((baud >> 16) & 0xFF);
    r.payload[5] = (uint8_t)((baud >> 24) & 0xFF);
    r.len = 6;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void GameIoService::_handle_mai2_set_send_en(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    if (frame.len < 1) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "mai2_set_send_en payload must be [en(u8)]", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    GameIoService* self = getInstance();
    const bool enable = frame.payload[0] != 0u;
    const bool changed = enable ? self->_serial.start() : self->_serial.stop();
    if (!changed) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "mai2 serial state change failed", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (enable) self->_serial.set_serial_ok(true);
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void GameIoService::_handle_mai2_command(Mai2Serial_Command command, const uint8_t*, uint8_t) {
    if (command != MAI2SERIAL_CMD_RSET) return;
    GameIoService* self = getInstance();
    if (self->_serial_reset_requests != 0xFFu) self->_serial_reset_requests++;
}

void GameIoService::_process_serial_reset() {
    if (_serial_reset_requests == 0u) return;
    _serial_reset_requests--;

    uint32_t actions = 0u;
    Psoc* psoc = Psoc::getInstance();
    if (ConfigManager::get_bool("comm.serial_reset_calibrate") && psoc->calibrate()) {
        actions |= 0x01u;
    }
    if (ConfigManager::get_bool("comm.serial_reset_baseline") && psoc->baseline_reset()) {
        actions |= 0x02u;
    }
    if (actions != 0u) {
        SelfHeal::getInstance()->note(SH_SERIAL_RESET_ACTIONS, actions);
    }
}

void GameIoService::_refresh_serial_publish_settings() {
    _serial_publish_settings.aggregation_delay_ms =
        ConfigManager::get_uint8("comm.aggregation_delay_ms");
    _serial_publish_settings.extra_send = ConfigManager::get_uint8("comm.extra_send");
    _serial_publish_settings.rate_limit_hz = ConfigManager::get_uint16("comm.rate_limit_hz");
    _serial_publish_settings.bits.rate_limit_enabled =
        ConfigManager::get_bool("comm.rate_limit_en") ? 1u : 0u;
    _serial_publish_settings.bits.send_only_on_change =
        ConfigManager::get_bool("comm.send_only_on_change") ? 1u : 0u;
}

void GameIoService::deinit() {
    _led_map_ready = false;
    LedMapService::getInstance()->deinit();
    _serial.deinit();
    _light.deinit();
    _serial_uart.deinit();
    _light_uart.deinit();
    _initialized = false;
}

void GameIoService::_consume_light_state() {
    const Mai2Light_LEDStatus* source = _light.get_led_status_array();
    bool changed = false;
    for (uint8_t index = 0; index < MAI2LIGHT_NUM_LEDS; index++) {
        const Mai2Light_LEDStatus& next = source[index];
        Mai2Light_LEDStatus& current = _light_state[index];
        if (current.color.r != next.color.r || current.color.g != next.color.g ||
            current.color.b != next.color.b || current.brightness != next.brightness ||
            current.enabled != next.enabled) {
            current = next;
            changed = true;
        }
        _light_rgb[index * 3u + 0u] = current.color.r;
        _light_rgb[index * 3u + 1u] = current.color.g;
        _light_rgb[index * 3u + 2u] = current.color.b;
    }
    if (changed) _light_generation++;
    LedMapService::getInstance()->set_unit_colors(_light_rgb);
}

void GameIoService::task() {
    if (!_initialized) return;

    // ★子段阶段码 + 子段耗时★(纯测量): 实测一次看门狗复位停在 game_io 段, 但该段含两条协议 +
    // 延迟线 + WS2812 共七件事, 段级粒度说不出是哪一件。见 usb_debug.h 的 GameIoSeg。
    uint32_t gio_t = gio_seg_begin(GIO_SEG_SERIAL_RX);
    _serial.task();
    gio_seg_mark(GIO_SEG_SERIAL_RX, gio_t);

    gio_t = gio_seg_begin(GIO_SEG_SER_RESET);
    _process_serial_reset();
    gio_seg_mark(GIO_SEG_SER_RESET, gio_t);

    gio_t = gio_seg_begin(GIO_SEG_LIGHT);
    _light.task();
    gio_seg_mark(GIO_SEG_LIGHT, gio_t);

    // Map the retained PSoC touch mask to the 34 protocol areas.
    Psoc* psoc = Psoc::getInstance();
    const uint32_t _lt0 = gio_seg_begin(GIO_SEG_TOUCH_MAP);
    // 扫描会话期间(见 SensorLink::output_suppressed)掩码是被逐格改写的无意义值: 按全松开上报,
    // 且仍走同一条 DelayLine/publish 路径, 使已按下的区域正常释放而不是卡在按下态。
    const uint64_t area_now = SensorLink::getInstance()->output_suppressed()
        ? 0u
        : BindingService::getInstance()->map_to_areas(psoc->touch_mask());
    if (_lt0 - _delay_refresh_us >= 50000u) {
        _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
        _refresh_serial_publish_settings();
        _delay_refresh_us = _lt0;
    }
    const Mai2Serial_TouchState delayed_touch(
        _touch_delay.tick(_lt0, _touch_delay_units, area_now));
    _serial_publish.record(_lt0, delayed_touch);
    gio_seg_mark(GIO_SEG_TOUCH_MAP, _lt0);
    const uint32_t _lt1 = gio_seg_begin(GIO_SEG_SEND_TOUCH);
    if (!_serial_publish.rate_limited(_lt0, _serial_publish_settings)) {
        Mai2Serial_TouchState publish_touch(
            _serial_publish.value(_lt0, _serial_publish_settings.aggregation_delay_ms));
        if (_serial_publish.should_send(publish_touch, _serial_publish_settings) &&
            _serial.send_touch_data(publish_touch)) {
            _serial_publish.note_send_success(publish_touch, _lt0, _serial_publish_settings);
        }
    }
    const uint32_t _lt2 = time_us_32();
    gio_seg_mark(GIO_SEG_SEND_TOUCH, _lt1);
    latency_note(&g_lat_proc_us, _lt1 - _lt0);
    latency_note(&g_lat_usb_us, _lt2 - _lt1);

    gio_t = gio_seg_begin(GIO_SEG_LIGHT_STATE);
    _consume_light_state();
    gio_seg_mark(GIO_SEG_LIGHT_STATE, gio_t);

    // 建链失败时不再每 tick 进灯服务: 内部虽有 _initialized 短路, 但辅路故障不该占快路的调用开销。
    if (_led_map_ready) {
        gio_t = gio_seg_begin(GIO_SEG_LEDMAP);
        LedMapService::getInstance()->task();
        gio_seg_mark(GIO_SEG_LEDMAP, gio_t);
    }
}
