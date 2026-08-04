#include "sensor_link.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/psoc/psoc.h"
#include "../csd_config/csd_config.h"
#include "../psoc_algo/psoc_algo.h"
#include "../psoc_updater/psoc_updater.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../latency_stats.h"
#include <pico/stdlib.h>
#include <cstring>

// 遥测租约(ms): 上位机需在此时限内经任意命令帧续期(UsbComm renew_all), 否则任务自动停。
static constexpr uint32_t TELEM_LEASE_MS = 3000;
// 频率自适应进度推送: 5Hz 足够看清阶段变化又不占带宽; 租约须覆盖最坏总耗时+余量,
// 因为长自适应期间上位机可能一条命令都不发(无 renew_all), 靠初始租约撑到完成。
static constexpr uint32_t AUTOTUNE_INTERVAL_US = 200000;
static constexpr uint32_t AUTOTUNE_LEASE_MS = 60000;
// 自续租上限(帧数): 长自适应期间上位机可能一条命令都不发, 而任意主机帧都会 renew_all(3s) 覆盖本任务
// 租约 → 靠本任务自续租撑过全程; 上限须【大于】RP2040 的自适应窗口(45s), 否则全通道逐通道自适应
// (36 × 单通道 ≈ 11-23s, 最坏更长)会在终态帧发出前先把推送任务饿死 → 上位机只能等超时。
// 350 帧 × 200ms = 70s > 45s, 异常情况下推送仍必然自灭。
static constexpr uint16_t AUTOTUNE_MAX_TICKS = 350;
// PSoC 救砖进度推送: 全片擦写+校验+重新下发数秒~十几秒, 同自适应用 5Hz + 长租约 + 帧数硬上限自灭。
static constexpr uint32_t RESCUE_INTERVAL_US = 200000;
static constexpr uint32_t RESCUE_LEASE_MS = 30000;
static constexpr uint16_t RESCUE_MAX_TICKS = 300;   // 60s > 最坏总耗时, 异常时推送必然自灭

volatile uint16_t g_lat_spi_us = 0;
volatile uint16_t g_lat_proc_us = 0;
volatile uint16_t g_lat_usb_us = 0;

SensorLink* SensorLink::_instance = nullptr;

namespace {
// Phase A 可运行时读写的 CSD 参数 id（与 PSoC cmd_get/set_param 及上位机 proto 对齐）
constexpr uint8_t kParamIds[] = {
    0x01,  // FINGER_TH
    0x02,  // NOISE_TH
    0x03,  // NEG_NOISE_TH
    0x04,  // HYSTERESIS
    0x05,  // ON_DEBOUNCE
    0x06,  // LOW_BSLN_RST
    0x07,  // RESOLUTION
    0x08,  // SNS_CLK_DIV
    0x09,  // IDAC_MOD
    0x0A,  // SNS_CLK_SOURCE
    0x0B,  // IDAC_GAIN
    0x0C,  // ENABLED(通道启用开关: 0=禁用/电极高阻 1=启用)
};
constexpr uint8_t kParamCount = sizeof(kParamIds) / sizeof(kParamIds[0]);
// Cp 哨兵(与 PSoC/上位机一致): 未测量 / 测量失败 / 读取失败。
constexpr uint32_t CP_UNMEASURED_FF = 0x00FFFFFFu;
// PARAM_GET_ALL 的"全通道单参数"变体标记(payload = 0xFF + param_id)。
constexpr uint8_t kAllChannels = 0xFFu;

// ★长周期 PSoC 指令的反堆叠闸门★
// 已有一条在途时新请求一律回 DEVICE_BUSY, **绝不排队**。排队会让若干条秒级操作背靠背堆在 core1 上,
// 而 core1 忙 = core0 的落盘窗口关闭 + 命令环满时 core0 卡在入队自旋, 主循环被整体拖长 ——
// 用户实测"一次保存后紧接一次 CH8 自适应就掉线"正落在这条上。
// DEVICE_BUSY 是可重试语义(上位机既有重试/冷却路径认它), 比排队后延迟数十秒才生效诚实得多。
// 返回 true = 已写好 NAK, 调用方直接 return。
inline bool heavy_gate_reject(const char* what, const HostFrame& frame,
                              uint8_t* response, uint16_t* response_length) {
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->heavy_busy()) return false;
    psoc->note_heavy_reject();
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                                what, response, 512);
    return true;
}

// ch_mask(u64 LE) → PSoC 的通道字节: 恰好一位 ⇒ 该通道号(0..35); 其余(空/多位/全 36 位) ⇒ 0xFF 全通道。
// ★为什么"恰好一位"才算单通道★ 上位机的单通道入口本来就编码成 `1<<ch`, 而批量入口是 host 侧
// 逐通道串行队列(每条仍是 `1<<ch`)。真正需要 0xFF 的只有"全 36 位"这种兼容/兜底调用。
// 多位但不满 36 的掩码没有对应的固件语义(帧里只有一个通道字节), 退化为全通道比只做第一位诚实。
inline uint8_t _mask_to_single_ch(const HostFrame& frame) {
    if (frame.len < 8) return 0xFFu;
    uint64_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) mask |= (uint64_t)frame.payload[i] << (8u * i);
    mask &= 0xFFFFFFFFFULL;   // 低 36 位有效
    if (mask == 0u || (mask & (mask - 1u)) != 0u) return 0xFFu;   // 空 / 多位 → 全通道
    uint8_t ch = 0;
    while ((mask >> ch) != 1u) ch++;
    return ch;
}

// 单通道重操作落在【已禁用】通道上 ⇒ 明确 NAK, 而不是让它静默空转。
// ★为什么要在 RP 这一层拦★ PSoC 侧本来就会跳过禁用通道(它的电极必须保持高阻, 不能为了校准去连),
// 但那是"什么都没发生"—— 上位机收到 ACK 却看不到任何变化, 只能当成"设备坏了"。在这里如实回绝,
// 上位机的既有 NAK 日志路径就会把原因写清楚。0xFF(全通道)不拦: 固件会逐通道跳过禁用项。
inline bool disabled_ch_reject(const char* what, uint8_t ch, const HostFrame& frame,
                               uint8_t* response, uint16_t* response_length) {
    if (ch >= SENSOR_LINK_CHANNELS) return false;
    if (CsdConfig::getInstance()->ch_enabled(ch)) return false;
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                                                what, response, 512);
    return true;
}
}  // namespace

SensorLink::SensorLink()
    : _lease_ms(TELEM_LEASE_MS),
      _mode(0),
      _rate_hz(30),
      _fields(TELEM_FIELD_RAW | TELEM_FIELD_BASELINE | TELEM_FIELD_DIFF | TELEM_FIELD_STATUS),
      _ch_mask(0),
      _last_emit_us(0),
      _stream_seq(0),
      _at_req_ch(0xFF),
      _at_ticks(0) {
    std::memset(_tx_buf, 0, sizeof(_tx_buf));
}

SensorLink* SensorLink::getInstance() {
    if (_instance == nullptr) _instance = new SensorLink();
    return _instance;
}

void SensorLink::init() {
    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::TELEM_START, _handle_telem_start);
    dispatcher->register_handler(HostCmd::TELEM_STOP, _handle_telem_stop);
    dispatcher->register_handler(HostCmd::PARAM_GET, _handle_param_get);
    dispatcher->register_handler(HostCmd::PARAM_SET, _handle_param_set);
    dispatcher->register_handler(HostCmd::PARAM_GET_ALL, _handle_param_get_all);
    dispatcher->register_handler(HostCmd::CALIBRATE, _handle_calibrate);
    dispatcher->register_handler(HostCmd::BASELINE_RESET, _handle_baseline_reset);
    dispatcher->register_handler(HostCmd::MODE_SET, _handle_mode_set);
    dispatcher->register_handler(HostCmd::CSD_CAPTURE, _handle_csd_capture);
    dispatcher->register_handler(HostCmd::CP_MEASURE, _handle_cp_measure);
    dispatcher->register_handler(HostCmd::CP_GET, _handle_cp_get);
    dispatcher->register_handler(HostCmd::GLOBAL_GET, _handle_global_get);
    dispatcher->register_handler(HostCmd::GLOBAL_SET, _handle_global_set);
    dispatcher->register_handler(HostCmd::GLOBAL_GET_ALL, _handle_global_get_all);
    dispatcher->register_handler(HostCmd::GLOBAL_COMMIT, _handle_global_commit);
    dispatcher->register_handler(HostCmd::AUTO_TUNE, _handle_auto_tune);
    dispatcher->register_handler(HostCmd::PSOC_RESCUE, _handle_psoc_rescue);
    dispatcher->register_handler(HostCmd::ALGO_GET_INFO, _handle_algo_get_info);
    dispatcher->register_handler(HostCmd::ALGO_UPLOAD, _handle_algo_upload);
    dispatcher->register_handler(HostCmd::ALGO_APPLY, _handle_algo_apply);
    dispatcher->register_handler(HostCmd::ALGO_RESET_DEFAULT, _handle_algo_reset_default);
    dispatcher->register_handler(HostCmd::ALGO_SET_ROM, _handle_algo_set_rom);
    dispatcher->register_handler(HostCmd::ALGO_GET_ROM, _handle_algo_get_rom);
    dispatcher->register_handler(HostCmd::ALGO_GET_TRACE, _handle_algo_get_trace);
    dispatcher->register_handler(HostCmd::ALGO_SET_CFG, _handle_algo_set_cfg);
    dispatcher->register_handler(HostCmd::ALGO_GET_CFG, _handle_algo_get_cfg);
    dispatcher->register_handler(HostCmd::ALGO_GET_SRC, _handle_algo_get_src);
    dispatcher->register_handler(HostCmd::ALGO_SET_SRC, _handle_algo_set_src);
    dispatcher->register_handler(HostCmd::ALGO_GET_CODE, _handle_algo_get_code);
}

void SensorLink::stop() {
    _stream.clear();
    Psoc::getInstance()->set_telemetry_active(false);
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
}

void SensorLink::suspend() {
    if (!_stream.active || _stream.suspended) return;
    _stream.suspended = true;
    Psoc::getInstance()->set_telemetry_active(false);   // 快照慢路暂停, 触控快路不受影响
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
}

void SensorLink::resume() {
    if (!_stream.active || !_stream.suspended) return;
    _stream.suspended = false;
    Psoc::getInstance()->set_telemetry_active(true);
    const uint32_t interval_us = 1000000UL / ((_rate_hz != 0u) ? _rate_hz : 1u);
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, _lease_ms,
                                         &SensorLink::emit_telem_task);
}

void SensorLink::emit_telem_task() {
    getInstance()->tick();
}

void SensorLink::tick() {
    // 周期由 TxScheduler 定时任务驱动; 本函数只负责"发送一帧"(不再自门控频率)。
    if (!_stream.emitting()) return;

    const uint32_t now_us = time_us_32();

    const psoc::SensorSnapshot& snapshot = Psoc::getInstance()->snapshot();
    uint16_t length = 0;
    uint8_t* payload = _telem_frame.payload;

    payload[length++] = static_cast<uint8_t>(now_us);
    payload[length++] = static_cast<uint8_t>(now_us >> 8);
    payload[length++] = static_cast<uint8_t>(now_us >> 16);
    payload[length++] = static_cast<uint8_t>(now_us >> 24);
    const uint16_t channel_count_position = length++;
    payload[length++] = _fields;

    if ((_fields & TELEM_FIELD_STATS) != 0) {
        const uint32_t sps = Psoc::getInstance()->samples_per_sec();
        const uint32_t spu = Psoc::getInstance()->scan_period_us();
        payload[length++] = (uint8_t)sps; payload[length++] = (uint8_t)(sps >> 8);
        payload[length++] = (uint8_t)(sps >> 16); payload[length++] = (uint8_t)(sps >> 24);
        payload[length++] = (uint8_t)spu; payload[length++] = (uint8_t)(spu >> 8);
        payload[length++] = (uint8_t)(spu >> 16); payload[length++] = (uint8_t)(spu >> 24);
    }

    if ((_fields & TELEM_FIELD_LATENCY) != 0) {
        const uint16_t ls = g_lat_spi_us, lp = g_lat_proc_us, lu = g_lat_usb_us;
        payload[length++] = (uint8_t)ls; payload[length++] = (uint8_t)(ls >> 8);
        payload[length++] = (uint8_t)lp; payload[length++] = (uint8_t)(lp >> 8);
        payload[length++] = (uint8_t)lu; payload[length++] = (uint8_t)(lu >> 8);
        g_lat_spi_us = 0; g_lat_proc_us = 0; g_lat_usb_us = 0;  // 清零开新窗口
    }

    uint8_t channel_count = 0;
    for (uint8_t channel = 0; channel < SENSOR_LINK_CHANNELS; channel++) {
        if (!_channel_selected(_ch_mask, channel)) continue;
        if (length + 8 > HOST_CMD_PAYLOAD_MAX) break;

        uint16_t raw = 0;
        uint16_t baseline = 0;
        int16_t diff = 0;
        uint8_t status = 0;
        if (snapshot.valid) {
            const psoc::SensorSample& sample = snapshot.channels[channel];
            raw = sample.raw;
            baseline = sample.baseline;
            diff = sample.diff;
            status = sample.status;
        }

        payload[length++] = channel;
        if ((_fields & TELEM_FIELD_RAW) != 0) {
            payload[length++] = static_cast<uint8_t>(raw);
            payload[length++] = static_cast<uint8_t>(raw >> 8);
        }
        if ((_fields & TELEM_FIELD_BASELINE) != 0) {
            payload[length++] = static_cast<uint8_t>(baseline);
            payload[length++] = static_cast<uint8_t>(baseline >> 8);
        }
        if ((_fields & TELEM_FIELD_DIFF) != 0) {
            payload[length++] = static_cast<uint8_t>(diff);
            payload[length++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) >> 8);
        }
        if ((_fields & TELEM_FIELD_STATUS) != 0) payload[length++] = status;
        channel_count++;
    }
    payload[channel_count_position] = channel_count;

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::TELEM_DATA);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        // FIFO 尚有空间才发本帧(全满则跳过, 让积压/命令响应先泵出); config_write 分段入队非阻塞。
        // 过载(host 消费慢)自然跳帧, 不堆积、不阻塞。
        if (usb->config_write_available() > 0) {
            usb->config_write(_tx_buf, frame_length);
        }
    }
    if (_mode == 1) {
        stop();   // 单次模式: 一帧即止(与主机显式 TELEM_STOP 同语义, 不自动恢复)
    }
}

void SensorLink::emit_autotune_task() {
    getInstance()->autotune_tick();
}

// 频率自适应阶段进度推送: core1 在阻塞完成自适应的同时发布 phase/step/试探分频,
// 本任务把它按 5Hz 组帧推给上位机, 使 20-25s 的长操作全程可见(而非上位机干等一个最终响应)。
void SensorLink::autotune_tick() {
    // 自续租: 本任务的存活由"设备侧操作未完成"决定, 不依赖上位机发命令(renew_all 会把租约压到 3s)。
    // 有硬上限, 异常时必然自灭。
    if (_at_ticks < AUTOTUNE_MAX_TICKS) {
        _at_ticks++;
        TxScheduler::getInstance()->renew(TX_TASK_AUTOTUNE, AUTOTUNE_LEASE_MS);
    }
    Psoc* psoc = Psoc::getInstance();
    psoc::AutoTuneProgress st = psoc->autotune_status();
    // core1 尚未取到本轮命令(队列排队中): 发布态仍是上一轮的结果 → 一律按"已受理/排队中"上报,
    // 否则上一轮的 done 会让本轮进度流刚开始就自取消。
    if (st.req != psoc->autotune_req()) {
        st.clear();
        st.state = 1;
        st.ch = _at_req_ch;
    }
    const bool done = (st.state == 2u);
    // 全通道终态必须回显请求哨兵 0xFF；进度帧才回显当前通道，不能让最后一次进度污染完成分支。
    const uint8_t report_ch = (done && _at_req_ch == kAllChannels) ? kAllChannels : st.ch;

    uint8_t* payload = _telem_frame.payload;
    uint16_t length = 0;
    payload[length++] = st.state;
    payload[length++] = st.phase;
    payload[length++] = st.step;
    payload[length++] = static_cast<uint8_t>(st.cur_div);
    payload[length++] = static_cast<uint8_t>(st.cur_div >> 8);
    payload[length++] = report_ch;
    payload[length++] = st.result;
    payload[length++] = static_cast<uint8_t>(st.div);
    payload[length++] = static_cast<uint8_t>(st.div >> 8);
    // ★尾部追加: 发起本轮的上位机请求 seq★ 前 9 字节布局一字未动(旧上位机按 >=9 解析, 兼容),
    // 新上位机据此把终态严格归属到自己发起的那一次请求。
    payload[length++] = _at_req_seq;

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::AUTO_TUNE_PROGRESS);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    bool sent = false;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        if (usb->config_write_available() > 0) {   // 背压保护: 满则跳过本帧, 不阻塞不排队
            usb->config_write(_tx_buf, frame_length);
            sent = true;
        }
    }

    // 终态帧若被背压丢弃则不收尾, 下个周期重发: 否则上位机永远等不到完成帧(要靠 28s 超时兜)。
    if (!done || !sent) return;
    // 终态: 成功且找到分频 → 写穿 RP2040 真相源(供持久化与回读一致)。原同步 handler 里的这段
    // 必须搬到此处, 因为受理时刻还没有结果。
    if (st.result == 1u) {
        CsdConfig* cfg = CsdConfig::getInstance();
        if (_at_req_ch < SENSOR_LINK_CHANNELS && st.div != 0u) {
            cfg->note_param(_at_req_ch, 0x08u, st.div);   // 0x08 = PARAM_SNS_CLK_DIV
        } else if (_at_req_ch >= SENSOR_LINK_CHANNELS) {
            // ★逐通道自适应★: 各通道分频互不相同(终态 st.div 已复用为"成功通道数"), 故逐通道
            // 从 PSoC 回读真实 snsClk 写穿真相源, 否则重启后 download_to_psoc 会用旧值覆盖调好的结果。
            for (uint8_t ch = 0; ch < SENSOR_LINK_CHANNELS; ch++) {
                uint32_t v = 0;
                if (Psoc::getInstance()->get_param(ch, 0x08u, &v) && v != 0u) {
                    cfg->note_param(ch, 0x08u, v);
                }
            }
        }
        // 写穿后请求持久化: 否则调好的 snsClk 只活在 RAM, 重启后 download_to_psoc 用旧 blob 覆盖。
        // 实际 flash 写由主循环安全窗口(main.cpp: has_pending_save)执行, 本函数不阻塞。
        // 生效无需再 APPLY: PSoC 自适应内部已写 widgetContext 并 InitializeAllBaselines, 真相源写穿
        // 只为持久化/回读一致; 多余 APPLY 会触发整片重初始化+重校准(额外重扫, 白掉一次基线)。
        if (!_at_defer_save) {
            cfg->request_save();
        }
    }
    TxScheduler::getInstance()->cancel(TX_TASK_AUTOTUNE);   // 最终帧已发出 → 自取消
}

void SensorLink::emit_rescue_task() {
    getInstance()->rescue_tick();
}

// PSoC 救砖阶段进度推送: 与自适应同结构(自续租 + 终态自取消)。重刷期间主循环被 SWD 阻塞,
// 本函数由 SwdProgrammer 的保活钩子经 TxScheduler::tick 调用, 故擦写全程仍有帧发出。
void SensorLink::rescue_tick() {
    if (_rescue_ticks < RESCUE_MAX_TICKS) {
        _rescue_ticks++;
        TxScheduler::getInstance()->renew(TX_TASK_RESCUE, RESCUE_LEASE_MS);
    }
    PsocUpdater* updater = PsocUpdater::getInstance();
    const PsocBringupReport& rep = updater->report();
    const uint8_t state = updater->rescue_state();

    uint8_t* payload = _telem_frame.payload;
    uint16_t length = 0;
    payload[length++] = state;
    payload[length++] = updater->rescue_phase();
    payload[length++] = updater->rescue_result();
    payload[length++] = static_cast<uint8_t>(rep.last_stage);
    payload[length++] = static_cast<uint8_t>(rep.failure_stage);

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::PSOC_RESCUE_PROGRESS);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    bool sent = false;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        if (usb->config_write_available() > 0) {   // 背压保护: 满则跳过本帧
            usb->config_write(_tx_buf, frame_length);
            sent = true;
        }
    }
    // 终态帧被背压丢弃则下周期重发, 否则上位机等不到完成帧。
    if (state == 2u && sent) TxScheduler::getInstance()->cancel(TX_TASK_RESCUE);
}

void SensorLink::_handle_psoc_rescue(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // PSOC_RESCUE(0x08): 空 payload。★立即回 ACK("已受理")★ —— 全片擦写数秒, 在 handler 里同步做
    // 会饿死 USB 导致掉线。仅置位请求, 由主循环 rescue_step 执行(其内部保活喂狗+泵 USB+推进度)。
    PsocUpdater* updater = PsocUpdater::getInstance();
    if (updater->rescue_active()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "psoc rescue already running", response, 512);
        return;
    }
    updater->rescue_request();
    SensorLink* self = getInstance();
    self->_rescue_ticks = 0;
    TxScheduler::getInstance()->schedule(TX_TASK_RESCUE, RESCUE_INTERVAL_US, RESCUE_LEASE_MS,
                                        &SensorLink::emit_rescue_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_telem_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len < 12) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "telem_start payload too short", response, 512);
        return;
    }

    SensorLink* self = getInstance();
    uint16_t position = 0;
    self->_mode = (frame.payload[position++] == 1) ? 1 : 0;
    self->_rate_hz = static_cast<uint16_t>(frame.payload[position]) |
                     (static_cast<uint16_t>(frame.payload[position + 1]) << 8);
    position += 2;
    self->_fields = frame.payload[position++];
    self->_ch_mask = 0;
    for (uint8_t byte = 0; byte < 8; byte++) {
        self->_ch_mask |= static_cast<uint64_t>(frame.payload[position + byte]) << (8u * byte);
    }

    if (self->_rate_hz < 1) self->_rate_hz = 1;
    if (self->_rate_hz > 1000) self->_rate_hz = 1000;
    self->_last_emit_us = 0;
    self->_stream.active = true;
    self->_stream.suspended = false;
    Psoc::getInstance()->set_telemetry_active(true);   // Phase C：开启全通道 raw 快照慢路

    // 注册/续期遥测定时任务(续期制): 周期=1e6/rate。可选 payload[12..13]=lease_ms(u16), 缺省 3s。
    // 之后任意主机命令帧都会经 UsbComm renew_all 续租; 续期超时(上位机丢失)则任务自动停。
    uint32_t lease_ms = TELEM_LEASE_MS;
    if (frame.len >= 14) {
        const uint16_t l = static_cast<uint16_t>(frame.payload[12]) |
                           (static_cast<uint16_t>(frame.payload[13]) << 8);
        if (l != 0) lease_ms = l;
    }
    self->_lease_ms = lease_ms;   // 记住协商值: 租约超时挂起后自动恢复要复用同一租约
    const uint32_t interval_us = 1000000UL / self->_rate_hz;
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, lease_ms,
                                         &SensorLink::emit_telem_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_telem_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    getInstance()->stop();   // Phase C：关闭快照慢路，回落触控快路；显式停流不自动恢复
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_param_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) + value(u32 LE)
    if (frame.len < 6) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_set payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    const uint32_t value = static_cast<uint32_t>(frame.payload[2]) |
                           (static_cast<uint32_t>(frame.payload[3]) << 8) |
                           (static_cast<uint32_t>(frame.payload[4]) << 16) |
                           (static_cast<uint32_t>(frame.payload[5]) << 24);

    // ★三处同源★ 本围栏与以下两处必须逐位等价, 任何一处改动必须三处同改:
    //   - PSoC:   psoc_firmware/CY8C4147AZI-SensorCore/main.c::_param_value_legal
    //   - 上位机: control_software/src/proto/telemetry.rs::param_fence(上位机侧唯一权威声明表,
    //             UI 输入范围 / 下发前拒绝 / 回读防污染都只读它)
    // 合法性防护(与 PSoC 端一致): 拒绝会 railed/时钟异常/校准发散的非法值, 不下发也不写真相源。
    // RESOLUTION 6..16; SNS_CLK_DIV 1..255; IDAC_MOD 0..127; IDAC_GAIN 0..6; SNS_CLK_SOURCE 低7位 0..6;
    // ENABLED(0x0C) 0..1。
    bool legal = true;
    switch (param_id) {
        case 0x07: legal = (value >= 6u)  && (value <= 16u);  break;   // RESOLUTION
        case 0x08: legal = (value >= 1u)  && (value <= 255u); break;   // SNS_CLK_DIV
        case 0x09: legal = (value <= 127u);                   break;   // IDAC_MOD
        case 0x0B: legal = (value <= 6u);                     break;   // IDAC_GAIN(0..6, 表7项索引7越界崩溃)
        case 0x0A: legal = ((value & 0x7Fu) <= 6u);           break;   // SNS_CLK_SOURCE
        case 0x0C: legal = (value <= 1u);                     break;   // ENABLED(硬件开关, 只 0/1)
        default: break;
    }
    if (!legal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param value out of legal range (guarded)", response, 512);
        return;
    }

    if (Psoc::getInstance()->set_param(ch, param_id, value)) {
        CsdConfig::getInstance()->note_param(ch, param_id, value);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_set failed", response, 512);
    }
}

void SensorLink::_handle_param_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) → 响应 channel + param_id + value(u32 LE)
    if (frame.len < 2) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_param(ch, param_id, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_get failed", response, 512);
        return;
    }

    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = ch;
    resp.payload[position++] = param_id;
    resp.payload[position++] = static_cast<uint8_t>(value);
    resp.payload[position++] = static_cast<uint8_t>(value >> 8);
    resp.payload[position++] = static_cast<uint8_t>(value >> 16);
    resp.payload[position++] = static_cast<uint8_t>(value >> 24);
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_param_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) → 响应 channel + count + [param_id + value(u32 LE)]×count
    // payload = 0xFF + param_id(u8) → "全通道单参数"变体: 响应 0xFF + param_id + count + [ch + value(u32 LE)]×count
    //   (36 通道单参数一次取回, 替代 36 条单发 PARAM_GET; 36×5+3=183B 仍在单帧内)
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get_all payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    if (ch == kAllChannels) {
        if (frame.len < 2) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "param_get_all(all channels) needs param_id", response, 512);
            return;
        }
        _emit_param_all_channels(frame, response, response_length);
        return;
    }

    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = ch;
    const uint16_t count_position = position++;

    uint8_t count = 0;
    for (uint8_t i = 0; i < kParamCount; i++) {
        uint32_t value = 0;
        if (!Psoc::getInstance()->get_param(ch, kParamIds[i], &value)) continue;
        resp.payload[position++] = kParamIds[i];
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

// "全通道单参数"批量回读: 响应 0xFF + param_id + count + [ch + value(u32 LE)]×count。
// 读不到的通道直接跳过(不占 pair), 上位机按 count 解析。
void SensorLink::_emit_param_all_channels(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    const uint8_t param_id = frame.payload[1];
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = kAllChannels;
    resp.payload[position++] = param_id;
    const uint16_t count_position = position++;

    uint8_t count = 0;
    for (uint8_t ch = 0; ch < SENSOR_LINK_CHANNELS; ch++) {
        uint32_t value = 0;
        if (!Psoc::getInstance()->get_param(ch, param_id, &value)) continue;
        resp.payload[position++] = ch;
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_calibrate(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE)。★单通道语义端到端透传★: 掩码恰好只有一位 ⇒ 把该通道号透传给
    // PSoC(它只校准该 widget 并只初始化该 widget 的基线); 多位/空/全 36 位 ⇒ 0xFF 全通道兼容语义。
    // 上位机的"全通道校准"已改为 host 侧可取消串行队列逐通道下发, 不再依赖固件内部 36 通道循环。
    const uint8_t cal_ch = _mask_to_single_ch(frame);
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法校准; 请先启用该通道",
                           cal_ch, frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(校准)", frame, response, response_length)) return;
    if (Psoc::getInstance()->calibrate(cal_ch)) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC calibrate failed", response, 512);
    }
}

void SensorLink::_handle_baseline_reset(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE); 单位掩码 ⇒ 只复位该通道基线(见 _mask_to_single_ch)。
    const uint8_t bsln_ch = _mask_to_single_ch(frame);
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法复位基线; 请先启用该通道",
                           bsln_ch, frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(基线复位)", frame, response, response_length)) return;
    if (Psoc::getInstance()->baseline_reset(bsln_ch)) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC baseline reset failed", response, 512);
    }
}

void SensorLink::_handle_auto_tune(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), pref(u8, 可选)]: ch 0..35=仅该通道下探, 0xFF=全 36 通道(旧行为);
    // 缺省(空 payload)=全通道; pref=灵敏度档位 1..7(缺省/越界→4), 越高=落档时往低频多让分频。
    // ★异步化★: 自适应最坏 20-25s, 端到端阻塞会让上位机干等且无从判断进度/异常。改为
    // 入队即回 ACK("已受理"), 阶段进度与最终结果(含 result/div)经 AUTO_TUNE_PROGRESS(0x2E)
    // 推送流上报, 完成帧发出后任务自取消。真相源写穿(note_param)随之搬到完成时刻。
    const uint8_t req_ch = (frame.len >= 1) ? frame.payload[0] : 0xFFu;
    uint8_t req_pref = (frame.len >= 2) ? frame.payload[1] : 4u;
    const bool defer_save = (frame.len >= 3) && (frame.payload[2] != 0u);
    if (req_pref < 1u || req_pref > 7u) req_pref = 4u;
    if (req_ch >= 36u && req_ch != 0xFFu) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "auto_tune channel out of range", response, 512);
        return;
    }
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法做频率自适应; 请先启用该通道",
                           req_ch, frame, response, response_length)) return;
    // ★首要嫌疑就是这条的堆叠★: 自适应单次 20s+, 期间再来一条只会背靠背排队, 把主循环整体拖长。
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(频率自适应)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    self->_at_req_ch = req_ch;
    // 归属键: 本轮进度/终态帧一律回显这个 seq(上位机据此丢弃上一轮的残留帧), 并折成 6 bit 标签
    // 随命令下到 PSoC, 使 RP↔PSoC 这一段也能认出陈旧结果。
    self->_at_req_seq = frame.seq;
    self->_at_defer_save = defer_save;
    self->_at_ticks = 0;
    if (!Psoc::getInstance()->auto_tune_start(req_ch, req_pref, frame.seq)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC auto_tune enqueue failed", response, 512);
        return;
    }
    TxScheduler::getInstance()->schedule(TX_TASK_AUTOTUNE, AUTOTUNE_INTERVAL_US, AUTOTUNE_LEASE_MS,
                                         &SensorLink::emit_autotune_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_mode_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = mode(u8)：0=自动校准/标准完整处理，非0=半自动手动
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "mode_set payload too short", response, 512);
        return;
    }
    if (Psoc::getInstance()->set_mode(frame.payload[0])) {
        CsdConfig::getInstance()->note_mode(frame.payload[0]);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC mode_set failed", response, 512);
    }
}

void SensorLink::_handle_csd_capture(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // AUTO 的实时参数由 CapSense 自动计算，回读固化会覆盖用户保存的半自动手动参数。
    CsdConfig* csd = CsdConfig::getInstance();
    if (csd->mode() != CSD_MODE_SEMI) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR,
            "自动模式由 PSoC 接管，禁止捕获以保护手动参数", response, 512);
        return;
    }
    if (!csd->capture_from_psoc(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC 参数读取失败", response, 512);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_cp_measure(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 0) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_measure payload must be empty", response, 512);
        return;
    }
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(Cp 测量)", frame, response, response_length)) return;
    if (Psoc::getInstance()->measure_cp()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC cp_measure failed", response, 512);
    }
}

void SensorLink::_handle_cp_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 1 || frame.payload[0] >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_get requires channel 0..35", response, 512);
        return;
    }

    const uint8_t ch = frame.payload[0];
    uint32_t cp_ff = 0;
    // ★不再 NAK★: PSoC 未测量/测量失败本就以哨兵 0xFFFFFF 表达; 读取(SPI 忙/超时)失败时也回同一
    // 哨兵的正常响应, 使上位机显示"未测量/测量失败"而不是刷 NAK 日志(实测 NAK 每秒 8~12 条刷屏)。
    // NAK 只保留给非法通道(上面已处理)。
    if (!Psoc::getInstance()->get_cp(ch, &cp_ff)) {
        cp_ff = CP_UNMEASURED_FF;
    }

    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::CP_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = ch;
    resp.payload[1] = static_cast<uint8_t>(cp_ff);
    resp.payload[2] = static_cast<uint8_t>(cp_ff >> 8);
    resp.payload[3] = static_cast<uint8_t>(cp_ff >> 16);
    resp.payload[4] = static_cast<uint8_t>(cp_ff >> 24);
    resp.len = 5;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_unsupported(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::NOT_IMPLEMENTED,
        "PSoC mutation is not implemented", response, 512);
}

// ---- 全局 CSD 配置命令 ----
namespace {
constexpr uint8_t kGlobalIds[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };  // ...IDAC_SENSE_CONFIG/AUTO_CALIBRATE_EN
constexpr uint8_t kGlobalCount = sizeof(kGlobalIds) / sizeof(kGlobalIds[0]);
}  // namespace

void SensorLink::_handle_global_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = gparam_id(u8) → 响应 [gparam_id, value(u32 LE)]
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global_get payload too short", response, 512);
        return;
    }
    const uint8_t gid = frame.payload[0];
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_global(gid, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_get failed", response, 512);
        return;
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::GLOBAL_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    resp.payload[p++] = gid;
    resp.payload[p++] = static_cast<uint8_t>(value);
    resp.payload[p++] = static_cast<uint8_t>(value >> 8);
    resp.payload[p++] = static_cast<uint8_t>(value >> 16);
    resp.payload[p++] = static_cast<uint8_t>(value >> 24);
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_global_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = gparam_id(u8) + value(u32 LE)。写 PSoC RAM 影子 + 写穿 CsdConfig + APPLY 重初始化生效。
    if (frame.len < 5) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global_set payload too short", response, 512);
        return;
    }
    const uint8_t gid = frame.payload[0];
    const uint32_t value = static_cast<uint32_t>(frame.payload[1]) |
                           (static_cast<uint32_t>(frame.payload[2]) << 8) |
                           (static_cast<uint32_t>(frame.payload[3]) << 16) |
                           (static_cast<uint32_t>(frame.payload[4]) << 24);
    // 合法性防护(与 PSoC 端一致): INACTIVE_SNS∈{1,2,4}; IDAC_GAIN_INIT 0..6; IDAC_MIN 0..127;
    // RAW_TARGET 1..99(0/≥100 会让自动校准发散→railed); MFS 分频 0..255(PSoC 侧字段是 uint8_t)。
    bool glegal = true;
    switch (gid) {
        case 0x01: glegal = (value == 1u) || (value == 2u) || (value == 4u); break; // INACTIVE_SNS
        case 0x02: glegal = (value <= 6u);                                   break; // IDAC_GAIN_INIT(0..6, 索引7越界崩溃)
        case 0x03: glegal = (value <= 127u);                                 break; // IDAC_MIN
        case 0x04: glegal = (value >= 1u) && (value <= 99u);                 break; // RAW_TARGET
        // ★补齐原先缺失的围栏★: MFS 偏移落在 PSoC 的 uint8_t 字段, 之前两端都不查, 写 300 会被
        // 静默截断成 44 且回显送回请求值, 上位机毫无察觉。现在与其它项同口径, 超范围直接 NAK。
        case 0x05:
        case 0x06: glegal = (value <= 255u);                                 break; // MFS_DIV_F1/F2
        case 0x07: glegal = (value <= 1u);                                   break; // IDAC_SENSE_CONFIG(0=sourcing,1=sinking)
        case 0x08: glegal = (value <= 1u);                                   break; // AUTO_CALIBRATE_EN(0/1)
        default: break;
    }
    if (!glegal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global value out of legal range (guarded)", response, 512);
        return;
    }
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->set_global(gid, value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_set failed", response, 512);
        return;
    }
    CsdConfig::getInstance()->note_global(gid, value);   // 写穿 RP2040 真相源
    // ★不再逐项 commit★: 只写影子。由上位机在批量下发全部全局项后发一次 GLOBAL_COMMIT 统一重初始化,
    // 避免"每项各触发一次完整 Init+Enable 重校准"的风暴(实测会拖垮 core0/USB → 掉线, 见 log.log)。
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

// 批量全局项下发完毕后, 单次触发 PSoC 完整重初始化(合并, 防反复重校准漂移/风暴)。
void SensorLink::_handle_global_commit(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(全局提交)", frame, response, response_length)) return;
    Psoc::getInstance()->global_commit();
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_global_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空 → [count(u8), (gparam_id, value u32 LE)×count]
    Psoc* psoc = Psoc::getInstance();
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::GLOBAL_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 1;   // count 占位
    uint8_t count = 0;
    for (uint8_t i = 0; i < kGlobalCount; i++) {
        uint32_t value = 0;
        if (!psoc->get_global(kGlobalIds[i], &value)) continue;
        resp.payload[p++] = kGlobalIds[i];
        resp.payload[p++] = static_cast<uint8_t>(value);
        resp.payload[p++] = static_cast<uint8_t>(value >> 8);
        resp.payload[p++] = static_cast<uint8_t>(value >> 16);
        resp.payload[p++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[0] = count;
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

// ---- JIT 算法引擎命令 ----
void SensorLink::_handle_algo_get_info(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [is_default(u8), psoc_valid(u8), len(u16 LE), crc16(u16 LE)]
    PsocAlgo* store = PsocAlgo::getInstance();
    bool psoc_valid = false;
    uint16_t psoc_len = 0;
    Psoc::getInstance()->get_algo_info(&psoc_valid, &psoc_len);   // 失败则 valid=false

    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_INFO);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    resp.payload[p++] = store->is_default() ? 1u : 0u;
    resp.payload[p++] = psoc_valid ? 1u : 0u;
    resp.payload[p++] = static_cast<uint8_t>(store->len());
    resp.payload[p++] = static_cast<uint8_t>(store->len() >> 8);
    resp.payload[p++] = static_cast<uint8_t>(store->crc16());
    resp.payload[p++] = static_cast<uint8_t>(store->crc16() >> 8);
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_upload(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = len(u16 LE) + crc16(u16 LE) + data[len]
    if (frame.len < 4) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload header too short", response, 512);
        return;
    }
    const uint16_t len = static_cast<uint16_t>(frame.payload[0]) |
                         (static_cast<uint16_t>(frame.payload[1]) << 8);
    const uint16_t crc16 = static_cast<uint16_t>(frame.payload[2]) |
                           (static_cast<uint16_t>(frame.payload[3]) << 8);
    if (len == 0u || len > PSOC_ALGO_MAX_LEN || (uint32_t)frame.len < 4u + (uint32_t)len) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload len invalid", response, 512);
        return;
    }
    // ★必须先挡住"上一次下发还没做完"★: blob 缓冲被 core1 持有, 此刻 set_algo 改写它会让 PSoC
    // 收到半新半旧的代码(=必然跑飞的算法)。下发已异步化, 故这里明确回 DEVICE_BUSY 让上位机重试。
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, 512);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    if (!store->set_algo(&frame.payload[4], len, crc16)) {   // 校验 crc16 一致才接受+持久化
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo crc16 mismatch", response, 512);
        return;
    }
    // 下发已受理(异步, 重活在 core1)。真实结果经 ALGO_GET_INFO 的 psoc_valid/len 回读对账,
    // 失败则由主循环上报 SELF_HEAL_EVENT(SH_ALGO_FALLBACK, detail=1)。
    // ★把失败原因分开, 不要都报成同一句 SENSOR_ERROR★
    // download_to_psoc 只有三个失败出口: 链路不可用 / 存储为空 / 入队失败。原先一律回
    // "algo download enqueue failed", 于是链路瞬断也被说成入队失败, 排查时完全指错方向(实测踩过)。
    // 链路类是【可重试】的, 必须回 DEVICE_BUSY 让上位机自动重试, 而不是 SENSOR_ERROR 让用户以为算法坏了。
    // 用去抖后的 link_alive(): 遥测流式期间瞬时 link_ok 频繁为 false, 会把上传全部拒掉。
    if (!Psoc::getInstance()->link_alive()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC 链路暂时不可用(可能正在重初始化), 请稍后重试", response, 512);
        return;
    }
    // 只入队代码下发即返回, ROM/cfg 由 PsocAlgo::tick() 补推(同步推会把 ACK 拖到几秒后)。
    if (!store->request_download(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "算法下发入队失败(core1 正忙于重校准/重初始化), 请稍后重试", response, 512);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_apply(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, 512);
        return;
    }
    // 同 ALGO_UPLOAD: 只入队, 不同步推 ROM/cfg。
    if (PsocAlgo::getInstance()->request_download(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "algo apply failed", response, 512);
    }
}

void SensorLink::_handle_algo_reset_default(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 同 ALGO_UPLOAD: reset_default 会改写 blob, 下发在途时必须先挡住。
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, 512);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    store->reset_default();                          // 回退内嵌默认 + 请求持久化
    store->request_download(Psoc::getInstance());    // 只入队(失败也回 ACK, 启动会重推)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_set_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), rom_lo, rom_hi] × count (count = len/3)
    if (frame.len == 0u || (frame.len % 3u) != 0u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_rom payload must be 3*N bytes", response, 512);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    Psoc* psoc = Psoc::getInstance();
    const uint16_t entries = (uint16_t)(frame.len / 3u);
    for (uint16_t i = 0; i < entries; ++i) {
        const uint16_t p = (uint16_t)(i * 3u);
        const uint8_t ch = frame.payload[p];
        const uint16_t rom = (uint16_t)frame.payload[p + 1] | ((uint16_t)frame.payload[p + 2] << 8);
        if (ch >= PSOC_ALGO_CHANNELS) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "algo_set_rom channel out of range", response, 512);
            return;
        }
        store->set_rom(ch, rom);           // 更新 RP 存储 + 请求持久化(真相源)
        psoc->set_algo_rom(ch, rom);       // 立即下发 PSoC
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 响应 36×u16 LE (RP 存储的每通道 ROM 表, 真相源)
    PsocAlgo* store = PsocAlgo::getInstance();
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_ROM);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    for (uint8_t ch = 0; ch < PSOC_ALGO_CHANNELS; ++ch) {
        const uint16_t v = store->rom(ch);
        resp.payload[p++] = static_cast<uint8_t>(v);
        resp.payload[p++] = static_cast<uint8_t>(v >> 8);
    }
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_get_trace(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), idx(u8)] → 响应 [ch, idx, out_active(u8), report(u16 LE)]
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t idx = frame.payload[1];
    if (ch >= PSOC_ALGO_CHANNELS || idx >= 4u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace ch/idx out of range", response, 512);
        return;
    }
    uint8_t active = 0;
    uint16_t report = 0;
    if (!Psoc::getInstance()->algo_get_trace(ch, idx, &active, &report)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_get_trace failed", response, 512);
        return;
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_TRACE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = ch;
    resp.payload[1] = idx;
    resp.payload[2] = active;
    resp.payload[3] = static_cast<uint8_t>(report);
    resp.payload[4] = static_cast<uint8_t>(report >> 8);
    resp.len = 5;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_set_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8), val(u8)]。写 PSoC 共享 cfg[idx] + PsocAlgo 持久化(随算法下发恢复)。
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg payload too short", response, 512);
        return;
    }
    const uint8_t idx = frame.payload[0];
    const uint8_t val = frame.payload[1];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg idx out of range", response, 512);
        return;
    }
    if (!Psoc::getInstance()->algo_set_cfg(idx, val)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_set_cfg failed", response, 512);
        return;
    }
    PsocAlgo::getInstance()->set_cfg(idx, val);   // 更新 RP 存储 + 请求持久化(真相源)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8)] → 响应 [idx, cfg(u8)]
    if (frame.len < 1u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg payload too short", response, 512);
        return;
    }
    const uint8_t idx = frame.payload[0];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg idx out of range", response, 512);
        return;
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CFG);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = idx;
    resp.payload[1] = PsocAlgo::getInstance()->cfg(idx);   // 直读 RP 存储的真相源(不下发 PSoC 查询)
    resp.len = 2;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_get_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 请求 [offset(u16 LE)](空 payload = offset 0) → 响应 [total(u16 LE), offset(u16 LE), chunk]。
    // 源最大 32KB, 单帧 payload 只有 4096, 故按 HOST_CMD_ALGO_SRC_CHUNK 分片, 由上位机按响应续请求。
    if (frame.len != 0u && frame.len != 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_src payload must be offset", response, 512);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint32_t total = store->src_len();
    uint32_t offset = 0u;
    if (frame.len >= 2u) {
        offset = static_cast<uint32_t>(frame.payload[0]) |
                 (static_cast<uint32_t>(frame.payload[1]) << 8);
    }
    if (offset > total) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_src offset past end", response, 512);
        return;
    }
    uint32_t n = total - offset;
    if (n > (uint32_t)HOST_CMD_ALGO_SRC_CHUNK) n = (uint32_t)HOST_CMD_ALGO_SRC_CHUNK;
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_SRC);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(total);
    resp.payload[1] = static_cast<uint8_t>(total >> 8);
    resp.payload[2] = static_cast<uint8_t>(offset);
    resp.payload[3] = static_cast<uint8_t>(offset >> 8);
    if (n > 0u) { memcpy(&resp.payload[4], store->src() + offset, n); }
    resp.len = static_cast<uint16_t>(4u + n);
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [offset(u16 LE), total(u16 LE), chunk]; 存算法 C 源(已由上位机滤注释)+持久化。
    // 严格连续分片: 只有最后一片才让新源生效, 中途断掉不会把半份源固化进 flash。
    if (frame.len < 4u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src header too short", response, 512);
        return;
    }
    const uint32_t offset = static_cast<uint32_t>(frame.payload[0]) |
                            (static_cast<uint32_t>(frame.payload[1]) << 8);
    const uint32_t total  = static_cast<uint32_t>(frame.payload[2]) |
                            (static_cast<uint32_t>(frame.payload[3]) << 8);
    const uint32_t n = static_cast<uint32_t>(frame.len) - 4u;
    if (total > PSOC_ALGO_SRC_MAX || offset > total || n > total - offset) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src chunk range invalid", response, 512);
        return;
    }
    if (!PsocAlgo::getInstance()->set_src_chunk(offset, total, &frame.payload[4], n)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src chunk not contiguous", response, 512);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_code(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [len(u16 LE), asm bytes] RP2040 存的算法 ASM 机器码回读(供反汇编查看)。
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint16_t n = store->len();
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CODE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(n);
    resp.payload[1] = static_cast<uint8_t>(n >> 8);
    if (n > 0u) { memcpy(&resp.payload[2], store->data(), n); }
    resp.len = static_cast<uint16_t>(2u + n);
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}
