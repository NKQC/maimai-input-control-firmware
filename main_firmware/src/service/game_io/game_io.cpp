#include "game_io.h"
#include "../config_manager/config_manager.h"
#include "../binding_service/binding_service.h"
#include "../latency_stats.h"
#include "../self_heal/self_heal.h"
#include "../../protocol/psoc/psoc.h"
#include <pico/stdlib.h>

GameIoService* GameIoService::_instance = nullptr;

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
      _serial_reset_requests(0) {}

// 懒构造无锁: 调用点(loop 内的 task、以及 UsbComm::update 同步派发的 LED_*/MAI2_* host_cmd 处理)
// 全在 core0 单线程; core1 只跑 Psoc::core1_run, 不进入本服务, 故首次 new 无竞态。
// 若将来把 host_cmd 派发移到 core1 或中断上下文, 此处必须改为预创建或加锁。
GameIoService* GameIoService::getInstance() {
    if (_instance == nullptr) _instance = new GameIoService();
    return _instance;
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
    _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
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

    HostFrame r;
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
    HostFrame r;
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::MAI2_GET_STATE);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    r.payload[0] = self->_serial.get_serial_ok() ? 1 : 0;
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
    getInstance()->_serial.set_serial_ok(frame.payload[0] != 0);
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

    _serial.task();
    _process_serial_reset();
    _light.task();

    // 实时触控快路：链路正常时取 36 位 on/off 掩码，经 BindingService 真实绑定表转 34 区；
    // 链路异常时上报全 0（无触摸），保持诚实。
    Psoc* psoc = Psoc::getInstance();
    const uint32_t _lt0 = time_us_32();
    const uint64_t channel_mask = psoc->link_ok() ? psoc->touch_mask() : 0;
    const uint64_t area_now = BindingService::getInstance()->map_to_areas(channel_mask);
    // 触控延迟线(100us 片, 0..100ms, UI 可配): 单拷贝环形, O(1)。延迟值 50ms 缓存刷新一次避免频繁查表。
    if (_lt0 - _delay_refresh_us >= 50000u) {
        _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
        _delay_refresh_us = _lt0;
    }
    Mai2Serial_TouchState touch(_touch_delay.tick(_lt0, _touch_delay_units, area_now));
    const uint32_t _lt1 = time_us_32();
    _serial.send_touch_data(touch);
    const uint32_t _lt2 = time_us_32();
    latency_note(&g_lat_proc_us, _lt1 - _lt0);
    latency_note(&g_lat_usb_us, _lt2 - _lt1);
    _consume_light_state();
    // 建链失败时不再每 tick 进灯服务: 内部虽有 _initialized 短路, 但辅路故障不该占快路的调用开销。
    if (_led_map_ready) LedMapService::getInstance()->task();
}
