#pragma once
#include <cstdint>
#include <hardware/structs/watchdog.h>
#include <hardware/timer.h>

// 调试计数器：经 vendor 控制请求(bRequest=0x50, EP0)读取。EP0 在 flash 写破坏 bulk 后仍存活，
// 可诊断"主循环是否存活/vendor OUT 是否 arm/rx 回调是否触发"等 bulk 死后的设备侧真相。
#pragma pack(push, 1)
struct UsbDebugCounters {
    uint16_t magic;              // 0xDB01
    uint16_t struct_len;         // sizeof(UsbDebugCounters)
    uint32_t loop_count;         // 主循环迭代次数(证明 loop 存活)
    uint32_t tud_task_count;     // tud_task 调用次数
    uint32_t vendor_rx_cb_count; // tud_vendor_rx_cb 触发次数(host->device OUT 完成)
    uint32_t vendor_rx_bytes;    // 已从 vendor 内部 fifo 搬出的总字节
    // 因 config_rx_buffer_ 环满而丢弃的字节数。非 0 = core0 曾长时间没来取命令(重操作阻塞),
    // 主机的命令帧被截断。原来这里是静默丢弃, 命令凭空消失且无从查证。
    uint32_t vendor_rx_dropped;
    uint32_t vendor_tx_calls;    // 实际写入 vendor TX FIFO 的次数
    uint32_t vendor_tx_bytes;    // 实际写入 vendor TX FIFO 的总字节
    uint32_t flash_write_count;  // flash 落地次数(config + csd)
    uint32_t loop_at_last_flash; // 最近一次 flash 落地时的 loop_count
    uint32_t rearm_count;        // vendor_service 实际重新武装 OUT 次数
    uint8_t  out_busy;           // 最近采样: OUT(0x01) usbd_edpt_busy
    uint8_t  out_stalled;        // 最近采样: OUT stalled
    uint8_t  mounted;            // tud_mounted
    uint8_t  debug_enabled;      // SET_DEBUG(0x51) 开关(默认0)
    // ★死前遗言★: 上次复位前 core0 停在哪个阶段(CrashStage, 0=正常/未知), 以及上次是否为看门狗复位。
    // 值取自 watchdog scratch[5], 跨复位保留 —— 崩溃后重连立刻能读到, 不必靠日志时间差反推。
    uint8_t  last_crash_stage;
    uint8_t  last_boot_was_wd;
    // ★flash 子系统真相★: flash_write_count 恒为 0 说明写从未成功过一次, 而崩溃点又在写配置里 ——
    // 必须先确认 LittleFS 是否真的挂上了。挂载失败仍去写 = 打到未初始化的文件系统上。
    uint8_t  lfs_ready;          // ConfigManager::_littlefs_ready
    uint8_t  save_entry_count;   // 进入 save_config_task 的次数(与 flash_write_count 对比即知死在哪半段)
    int8_t   last_save_stage;    // 1=已进入 2=JSON已生成 3=文件已打开 4=写完 5=已close 6=长度校验通过
    uint8_t  _pad;
    // NvStore 落盘状态：仅追加，旧字段偏移保持不变。
    uint8_t  nv_dirty_mask;      // bit0=KV bit1=CSD bit2=ALGO_BIN bit3=ALGO_SRC
    uint32_t nv_commit_ok;       // 成功写入一个 Region 的次数
    uint32_t nv_commit_fail;     // _write_slot 返回 false 的次数
    uint32_t nv_algo_src_len;    // ALGO_SRC 当前有效负载长度
    // ★主循环阻塞剖面★(纯测量, 不改任何行为)。看门狗 5000ms ⇒ 余量 = 5000000 - loop_max_us。
    // 掉线定位此前一直靠"从症状反推", 反复猜错; 这里改为直接量。仅追加, 旧字段偏移不变。
    uint32_t loop_max_us;                     // 整轮 loop() 最长耗时(us)
    uint32_t seg_max_us[8 /*LOOP_SEG_COUNT*/];// 各段各自的最长单轮耗时(us), 段义见 LoopSeg
    // 反堆叠闸门实证: 长周期 PSoC 指令因"已有一条在途"被拒的累计次数 + 当前是否在途。
    // >0 即**确证**主机侧确实存在长周期指令堆叠(此前只能靠日志时间线推断)。
    uint32_t psoc_heavy_rejects;
    uint8_t  psoc_heavy_busy;
    // ★跨复位死前遗言(增强)★: RAM 里的峰值随复位丢失, 于是"死掉那一次到底卡了多久"永远看不到。
    // 故把峰值同步写进 watchdog scratch[3](跨复位保留), 启动时取出。
    uint32_t last_boot_loop_max_us;
    // 1 = 上次复位由 hardfault 触发(自装 isr_hardfault 在 scratch[2] 留标记后主动软复位)。
    // ★这是唯一能把"跑得太慢被看门狗咬"与"跑飞了"区分开的判据★: 两者都保留 scratch,
    // 也都表现为 watchdog reason=TIMER(fault 默认落进 while(1), 最后仍由看门狗收尾)。
    uint8_t  last_boot_was_fault;
    uint8_t  last_reset_reason;   // watchdog_hw->reason 低位: bit0=TIMER bit1=FORCE
    // game_io 段内部再分 8 个子段的最长耗时(us)。实测复位停在 game_io(stage=0x14), 而该段里塞了
    // 两条协议 + 延迟线 + WS2812 七件事, 段级粒度不足以定位 ⇒ 直接量到子段。段义见 GameIoSeg。
    uint32_t seg2_max_us[8 /*GAMEIO_SEG_COUNT*/];
    uint32_t loop_t0_us;            // 本轮 loop 起点(段级"已耗时"基准), 供上位机核对时基
    uint16_t last_boot_stage_at_ms; // 上次复位: 最后一次进段时"本轮已耗时"(ms) —— 定位时间花在哪
    uint16_t last_boot_peak_ms;     // 上次复位: 已完成轮的最长耗时(ms)
    // NvStore 各区 flash 内容是否有效(bit0=KV bit1=CSD bit2=ALGO_BIN bit3=ALGO_SRC)。
    // 单份存储的语义"坏只坏在那一区"必须**看得见** —— 上一轮 gen_inconsistent 无人调用、无人上报,
    // 等于那条链是断的, 区失效只能靠猜。★只能追加在末尾★: 前面任何插入都会整体挪偏移。
    uint8_t  nv_valid_mask;
    // ★只能追加在末尾★(前面任何插入都会整体挪偏移, 旧上位机随即错位解析)。
    // 上次 hardfault 发生在哪个核: 0=core0, 1=core1, 0xFF=上次不是 hardfault。
    // 没有这一位时, 死前遗言的 stage 会因两核共写 scratch[1] 而指向错误的核。
    uint8_t  last_boot_fault_core;
    // 当前 core1 阶段码(CrashStage, core1 唯一写者, 见 core1_stage_set)。不跨复位保留。
    uint8_t  core1_stage;
    // ★只能追加在末尾★(前面任何插入都会整体挪偏移, 旧上位机随即错位解析)。
    // ★响应编码失败取证★: dispatch 返回 resp_len==0 意味着 HostCmdCodec::encode_* 拒绝组帧
    // (max_len 装不下, 见 _encode_into), USB 层随即不入队 ⇒ 主机侧表现为"命令发出去了但永远等不到
    // 响应/回读为空", 而设备侧此前完全静默, 只能靠对着协议逐条猜。所有已注册 handler 都必然赋值
    // resp_len, 遥测/自持恢复推送走各自的 _tx_buf 而不经 dispatch ⇒ 这里计数不会被正常无响应路径污染。
    uint32_t resp_encode_fail;      // 累计失败次数(封顶不回绕, 见 usb_comm.cpp 的 _note_resp_encode_fail)
    uint8_t  resp_fail_cmd;         // 最近一次失败的请求 cmd 码
    uint8_t  _resp_fail_pad;
    uint16_t resp_fail_req_len;     // 最近一次失败的请求 payload 长度
    // 主机命令分发观测：仅追加，保持前面已有字段偏移不变。
    uint32_t host_dispatch_count;             // HostCmdDispatcher::dispatch 累计次数
    uint32_t host_algo_info_dispatch_count;   // ALGO_GET_INFO 分发累计次数
    uint8_t  host_last_dispatch_cmd;          // 最近一次分发的 HostCmd 命令码
    uint8_t  host_last_dispatch_seq;          // 最近一次分发的帧序号
    uint16_t host_last_dispatch_resp_len;     // 最近一次分发返回的响应长度
};

// EP0 DEBUG_READ 固定尾部：P0..P7 的 GPIO drive mode(PC) 与 HSIOM PORT_SEL 快照。
// 旧主机只按返回 report_length 解析前缀；新主机仅在完整尾部存在且 read_ok=1 时采信这些寄存器值。
struct UsbDebugGpioTail {
    uint32_t gpio_pc[8];
    uint32_t hsiom_port_sel[8];
    uint32_t swd_status;
    uint8_t read_ok;
    uint8_t _reserved[3];
};

struct UsbDebugReport {
    UsbDebugCounters counters;
    UsbDebugGpioTail gpio;
};

/// core0 阶段码, 写入 watchdog scratch[5]。看门狗复位后由 setup() 取出上报。
enum CrashStage : uint8_t {
    CRASH_STAGE_NONE          = 0,
    CRASH_STAGE_CFG_FLASH     = 1,   // ConfigManager::save_config_task (LittleFS 写)
    CRASH_STAGE_CSD_FLASH     = 2,   // CsdConfig::save
    CRASH_STAGE_ALGO_FLASH    = 3,   // PsocAlgo::save
    CRASH_STAGE_PSOC_SUBMIT   = 4,   // Psoc::_submit 等 core1 结果
    CRASH_STAGE_PSOC_ENQUEUE  = 5,   // Psoc::_submit 等命令环空位
    CRASH_STAGE_USB_UPDATE    = 6,   // UsbComm::update (解析并分发主机命令)
    CRASH_STAGE_CORE1_CMD     = 7,   // core1 正在执行 SPI 命令(由 core1 写)
};

/// 写阶段码。inline 且只一条 store, 放在热路径上开销可忽略。
/// ★只能用 scratch[0..3]★: RP2040 的 watchdog scratch[4..7] 是 SDK 的 watchdog_reboot 机制自用
/// (存 magic / 入口地址 / 栈指针), 看门狗复位路径一走就会把里面的内容覆盖掉。
/// 先前把运行标记放 scratch[7]、阶段码放 scratch[5], 于是每次看门狗复位后判据都被冲成 0,
/// 被误读成"不是看门狗复位", 白绕了两轮。用户可用区只有 0..3。
#define CRASH_SCRATCH_RUN    0   /// 运行中标记(每轮 loop 重写)
#define CRASH_SCRATCH_STAGE  1   /// core0 阶段码
#define CRASH_SCRATCH_FAULT  2   /// hardfault 标记(isr_hardfault 写入后主动软复位)
/// 跨复位取证字(打包两个 ms 量): 高 16 位 = 最后一次进入某段时"本轮已耗时"(ms);
/// 低 16 位 = 已完成轮的最长耗时(ms)。
/// ★为什么要"进入段时的已耗时"★: stage 只说明复位那一刻停在哪一段, 说明不了时间花在哪一段 ——
/// 若某段先烧掉 4.9s, 看门狗极可能在紧随其后的短段里到期, stage 就指向一个无辜的段(实测已被骗一次:
/// 报 game_io/light, 而 light 本身每次只跑 58us, 内部也没有任何无界循环)。
/// ★为什么打包★: 用户可用 scratch 只有 0..3, RUN/STAGE/FAULT 已占三个, 只剩这一个字; ms 精度
/// 对"是否接近 5s"这个问题完全够用。
#define CRASH_SCRATCH_PM     3
#define CRASH_RUN_MAGIC      0x4D32524Eu   /// "M2RN"
/// hardfault 标记。★低字节留给"哪个核出错"★: 同一个 fault 处理程序被两个核共用, 只留一个
/// magic 就永远答不出"是谁跑飞了" —— 而这正是本轮排查最先需要的那一位信息。
/// 高 24 位固定 "FUL", 低 8 位 = get_core_num()。判定 fault 时只比高 24 位, 故与旧值兼容。
#define CRASH_FAULT_MAGIC    0x46554C54u   /// "FULT" = 高24位 0x46554C + core 0
#define CRASH_FAULT_MASK     0xFFFFFF00u
#define CRASH_FAULT_TAG      0x46554C00u   /// "FUL" << 8
static inline uint32_t crash_fault_word(uint32_t core) {
    return CRASH_FAULT_TAG | (core & 0xFFu);
}
/// 主循环分段阶段码 = 本基址 + LoopSeg。原有 CrashStage 只覆盖若干重操作, 其余整轮都报 0(=NONE),
/// 于是"死在 loop 的哪一段"完全看不出来 —— 实测一次复位就报 stage=0, 等于没信息。
#define CRASH_STAGE_LOOP_BASE 0x10u
/// game_io 子段阶段码基址 = 本基址 + GameIoSeg。
#define CRASH_STAGE_GAMEIO_BASE 0x20u
/// Mai2Light::task 内部语句级阶段码。已确证 >5s 卡在该函数内(stage=0x23 且进段时已耗时=0ms),
/// 但函数体本身无任何无界循环 ⇒ 必须再细一级, 定位到具体那一次调用。
#define PM_STAGE_LIGHT_RX_READ   0x30u  /// cdc_read(把 CDC 环搬进 chunk)
#define PM_STAGE_LIGHT_FEED      0x31u  /// _feed 逐字节(含 _dispatch/_ack_send)
#define PM_STAGE_LIGHT_ACK_FREE  0x32u  /// _ack_send: 查 TX FIFO 剩余空间
#define PM_STAGE_LIGHT_ACK_WRITE 0x33u  /// _ack_send: 写 TX FIFO
#define PM_STAGE_LIGHT_FADE      0x34u  /// _fade_step
#define PM_STAGE_CDC_AVAIL       0x36u  /// tud_cdc_n_write_available
#define PM_STAGE_CDC_WRITE       0x37u  /// tud_cdc_n_write
#define PM_STAGE_CDC_FLUSH       0x38u  /// tud_cdc_n_write_flush
#define PM_STAGE_CDC_READ        0x39u  /// tud_cdc_n_read(rx 回调内)
#define PM_STAGE_CDC_READ_DONE   0x3Au  /// tud_cdc_n_read 已返回, 准备搬进环
#define PM_STAGE_CDC_RX_DONE     0x3Bu  /// rx 回调整体结束, 控制权已交回 tud_task
#define PM_STAGE_TUD_TASK        0x3Cu  /// 即将进入 tud_task
#define PM_STAGE_TUD_TASK_DONE   0x3Du  /// tud_task 已返回

static inline void crash_stage_set(uint8_t stage) {
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = stage;
}

/// ★core1 专用阶段码, 绝不碰 scratch[CRASH_SCRATCH_STAGE]★
/// scratch[1] 只有一个, 原先两个核都往里写, 且 core1 写完 CORE1_CMD 后从不复位它 ⇒
/// core0 崩溃后 core1 仍在跑 1ms 周期, 会把自己的 7 盖到 core0 的真实阶段上, 于是
/// "死前遗言"恒指向 CORE1_CMD, 把排查引到错误的核上(本轮已被骗一次)。
/// 现在 core1 只写 RAM 里的自有字段: scratch[1] 恒为 core0 真相, core1 的阶段仍可经
/// DEBUG_READ 观察(仅不跨复位保留 —— 跨复位那一格已被 core0 的判据占满, 见 scratch 分配注释)。
extern volatile uint8_t g_core1_stage;
static inline void core1_stage_set(uint8_t stage) {
    g_core1_stage = stage;
}

/// 每轮主循环调用: 标记"正在运行"。看门狗/fault 复位会保留 scratch[0..3], 于是复位后仍能读到它。
static inline void crash_run_mark(void) {
    watchdog_hw->scratch[CRASH_SCRATCH_RUN] = CRASH_RUN_MAGIC;
}
#pragma pack(pop)

extern volatile UsbDebugCounters g_usb_dbg;

/// 主循环段划分。★只取各段自身最大值, 不记录"哪段最长"这种派生量★——派生量会与真相漂移,
/// 而 8 个独立最大值直接比较即可得出同样结论。索引必须与 seg_max_us[] 一一对应。
enum LoopSeg : uint8_t {
    LOOP_SEG_USB_TASK  = 0,   // HAL_USB::task + Mai2Bus::task
    LOOP_SEG_PSOC      = 1,   // PsocUpdater/救砖/XRES/provisioning 下发
    LOOP_SEG_HOST_CMD  = 2,   // UsbComm::update (主机命令解析与分发)
    LOOP_SEG_TX_SCHED  = 3,   // TxScheduler::tick (遥测/进度推送)
    LOOP_SEG_GAME_IO   = 4,   // BindingService::tick + GameIoService::task (两条 CDC)
    LOOP_SEG_KEYBOARD  = 5,   // KeyboardService::task
    LOOP_SEG_NV_COMMIT = 6,   // flash 落盘窗口(擦写 + lockout)
    LOOP_SEG_LED       = 7,   // 状态灯
    LOOP_SEG_COUNT     = 8,
};

/// 记一段耗时。单次调用 = 一次读时钟 + 一次比较, 热路径开销可忽略(实测 <1us)。
static inline void loop_prof_mark(uint8_t seg, uint32_t start_us) {
    const uint32_t dt = time_us_32() - start_us;
    if (dt > g_usb_dbg.seg_max_us[seg]) g_usb_dbg.seg_max_us[seg] = dt;
}

static inline void _pm_put(uint32_t elapsed_ms, uint32_t peak_ms) {
    if (elapsed_ms > 0xFFFFu) elapsed_ms = 0xFFFFu;
    if (peak_ms > 0xFFFFu) peak_ms = 0xFFFFu;
    watchdog_hw->scratch[CRASH_SCRATCH_PM] = (elapsed_ms << 16) | peak_ms;
}
static inline void _pm_set_elapsed_ms(uint32_t elapsed_us) {
    _pm_put(elapsed_us / 1000u, watchdog_hw->scratch[CRASH_SCRATCH_PM] & 0xFFFFu);
}
static inline void _pm_set_peak_ms(uint32_t peak_us) {
    _pm_put(watchdog_hw->scratch[CRASH_SCRATCH_PM] >> 16, peak_us / 1000u);
}

/// 每轮 loop 开头调用: 记本轮起点(段级"已耗时"以它为基准)。
static inline uint32_t loop_prof_begin(void) {
    const uint32_t now = time_us_32();
    g_usb_dbg.loop_t0_us = now;
    return now;
}

/// game_io 段内部子段。索引必须与 seg2_max_us[] 一一对应。
enum GameIoSeg : uint8_t {
    GIO_SEG_BINDING   = 0,   // BindingService::tick(绑定表 → 34 区)
    GIO_SEG_SERIAL_RX = 1,   // Mai2Serial::task(命令解析)
    GIO_SEG_SER_RESET = 2,   // _process_serial_reset({RSET} 触发的校准/基线复位)
    GIO_SEG_LIGHT     = 3,   // Mai2Light::task(收帧 + 应答 + 渐变)
    GIO_SEG_TOUCH_MAP = 4,   // touch_mask + map_to_areas + 延迟线
    GIO_SEG_SEND_TOUCH = 5,  // send_touch_data(写 CDC IN)
    GIO_SEG_LIGHT_STATE = 6, // _consume_light_state(协议色 → 映射服务)
    GIO_SEG_LEDMAP    = 7,   // LedMapService::task(WS2812 输出时隙)
    GAMEIO_SEG_COUNT  = 8,
};

/// 进入某段: 记阶段码 + 本轮已耗时(跨复位取证), 返回起始时刻。几条单周期指令, 热路径可忽略。
static inline uint32_t loop_seg_begin(uint8_t seg) {
    const uint32_t now = time_us_32();
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = (uint32_t)(CRASH_STAGE_LOOP_BASE + seg);
    _pm_set_elapsed_ms(now - g_usb_dbg.loop_t0_us);
    return now;
}

/// 进入 game_io 子段。★注意★: 子段码会盖掉外层的 game_io 段码, 死前遗言只保留最细一级——
/// 这正是想要的(0x14 只能说明"死在 game_io", 0x2x 直接指出死在哪一件事上)。
static inline uint32_t gio_seg_begin(uint8_t sub) {
    const uint32_t now = time_us_32();
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = (uint32_t)(CRASH_STAGE_GAMEIO_BASE + sub);
    _pm_set_elapsed_ms(now - g_usb_dbg.loop_t0_us);
    return now;
}

/// 只打阶段码, 不计时。用于语句级定位: 一条 store, 可以密集撒在热路径上。
static inline void pm_stage(uint8_t code) {
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = (uint32_t)code;
}

static inline void gio_seg_mark(uint8_t sub, uint32_t start_us) {
    const uint32_t dt = time_us_32() - start_us;
    if (dt > g_usb_dbg.seg2_max_us[sub]) g_usb_dbg.seg2_max_us[sub] = dt;
}

/// 记整轮耗时(≈相邻两次 watchdog_update 的间隔, 即真正与 5s 看门狗竞争的量)。
/// 峰值同步落 scratch[3]: 复位会清 RAM 但保留 scratch, 于是下次启动仍能回答"死那次卡了多久"。
static inline void loop_prof_total(uint32_t start_us) {
    const uint32_t dt = time_us_32() - start_us;
    if (dt > g_usb_dbg.loop_max_us) {
        g_usb_dbg.loop_max_us = dt;
        _pm_set_peak_ms(dt);
    }
}

/// 经 EP0 请求 0x53 清零, 使压测能在干净窗口内测峰值(峰值不可差分, 必须能复位)。
static inline void loop_prof_clear(void) {
    g_usb_dbg.loop_max_us = 0u;
    watchdog_hw->scratch[CRASH_SCRATCH_PM] = 0u;
    for (uint8_t i = 0; i < LOOP_SEG_COUNT; i++) g_usb_dbg.seg_max_us[i] = 0u;
    for (uint8_t i = 0; i < GAMEIO_SEG_COUNT; i++) g_usb_dbg.seg2_max_us[i] = 0u;
}

// Flash 写期间置位：LittleFS 的 XIP 擦写无法安全切片让出，TxScheduler 据此暂停遥测/进度等
// 可丢弃推送，避免在 USB 不能运行的窗口继续向 64B vendor IN FIFO 累积数据而触发 stall。
extern volatile uint8_t g_usb_flash_busy;

// 经 EP0 控制请求(bRequest=0x52)置位的 BOOTSEL 请求：因 EP0 在 bulk vendor 死后仍存活，
// 可在 vendor 卡死时仍软件触发进烧录模式，免去物理 BOOTSEL。loop() 检测到后 reset_usb_boot。
extern volatile uint8_t g_bootsel_request;

// 经 host_cmd(REBOOT_PSOC=0x06)置位的 PSoC 重启请求。loop() 检测到后脉冲 XRES 复位 PSoC 进运行态,
// 使"需重启生效"的 PSoC 改动(如全局 CSD 重初始化)真正生效; 链路重连后自动重新下发算法/CSD。
extern volatile uint8_t g_psoc_reboot_request;

// 最近一次收到主机(上位机)host_cmd 帧的 millis 时间戳。UsbComm 每次分发帧时更新。
// loop() 据此判定"主机已连接"(近 2s 内有帧)→ 绿灯常亮，否则心跳闪烁。
extern volatile uint32_t g_last_host_cmd_ms;
