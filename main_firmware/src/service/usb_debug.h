#pragma once
#include <cstdint>
#include <hardware/structs/watchdog.h>

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
#define CRASH_RUN_MAGIC      0x4D32524Eu   /// "M2RN"

static inline void crash_stage_set(uint8_t stage) {
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = stage;
}

/// 每轮主循环调用: 标记"正在运行"。看门狗/fault 复位会保留 scratch[0..3], 于是复位后仍能读到它。
static inline void crash_run_mark(void) {
    watchdog_hw->scratch[CRASH_SCRATCH_RUN] = CRASH_RUN_MAGIC;
}
#pragma pack(pop)

extern volatile UsbDebugCounters g_usb_dbg;

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
