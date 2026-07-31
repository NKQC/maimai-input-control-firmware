#pragma once

#include <stdint.h>
#include <string>
#include <hardware/pio.h>

/**
 * PIO状态机配置结构体
 * 统一管理所有PIO状态机配置参数
 */
struct PIOStateMachineConfig {
    // 引脚配置
    uint8_t out_base = 0;
    uint8_t out_count = 0;
    uint8_t in_base = 0;
    uint8_t set_base = 0;
    uint8_t set_count = 0;
    uint8_t sideset_base = 0;
    uint8_t sideset_bit_count = 0;
    bool sideset_optional = false;
    bool sideset_pindirs = false;
    
    // 时钟配置
    float clkdiv = 1.0f;
    
    // 程序配置
    uint8_t wrap_target = 0;
    uint8_t wrap = 31;
    
    // 程序偏移
    uint8_t program_offset = 0;

    // 移位寄存器配置（默认值等价 pico_get_default_sm_config：左移、无自动、阈值32）
    // 默认保持与原行为一致，避免影响 NeoPixel 等既有使用者
    bool out_shift_right = false;   // OUT 方向右移（LSB first）
    bool autopull = false;
    uint8_t pull_threshold = 32;
    bool in_shift_right = false;    // IN 方向右移（LSB first）
    bool autopush = false;
    uint8_t push_threshold = 32;

    // 是否启用状态机
    bool enabled = false;
};

/**
 * HAL层 - PIO接口抽象类
 * 提供底层PIO接口，支持PIO0和PIO1两个实例
 * 允许外部传入ASM程序实现自定义协议
 */

class HAL_PIO {
public:
    virtual ~HAL_PIO() = default;
    
    // 初始化PIO接口，同时初始化指定的GPIO引脚
    virtual bool init(uint8_t gpio_pin) = 0;
    
    // 释放PIO资源
    virtual void deinit() = 0;
    
    // 加载PIO程序
    virtual bool load_program(const pio_program_t* program, uint8_t* offset) = 0;
    
    // 卸载PIO程序
    virtual void unload_program(const pio_program_t* program, uint8_t offset) = 0;
    
    // 获取状态机
    virtual bool claim_sm(uint8_t* sm) = 0;
    
    // 释放状态机
    virtual void unclaim_sm(uint8_t sm) = 0;
    
    // 统一配置状态机（包含初始化和启动）
    virtual bool sm_configure(uint8_t sm, const PIOStateMachineConfig& config) = 0;
    
    // 启动/停止状态机
    virtual void sm_set_enabled(uint8_t sm, bool enabled) = 0;
    
    // 数据传输
    virtual void sm_put_blocking(uint8_t sm, uint32_t data) = 0;
    virtual bool sm_put_nonblocking(uint8_t sm, uint32_t data) = 0;  // 非阻塞发送，成功返回true
    virtual uint32_t sm_get_blocking(uint8_t sm) = 0;
    virtual bool sm_is_tx_fifo_full(uint8_t sm) = 0;
    virtual bool sm_is_rx_fifo_empty(uint8_t sm) = 0;

    // ---- 以下为多引脚 / bit-bang 协议（如 SWD）所需的通用扩展 ----
    // 直接执行单条 PIO 指令（用于 pindirs/jmp 等即时控制）
    virtual void sm_exec(uint8_t sm, uint16_t instr) = 0;
    // 清空指定状态机的 TX/RX FIFO
    virtual void sm_clear_fifos(uint8_t sm) = 0;
    // 重启状态机及其时钟分频计数
    virtual void sm_restart(uint8_t sm) = 0;
    // 将额外的 GPIO 交给本 PIO 实例（多引脚协议，如 SWD 的 SWCLK）
    virtual void init_pin(uint8_t gpio) = 0;
    // 将连续 count 个引脚（自 base 起）方向设为输出
    virtual void sm_set_pindirs_out(uint8_t sm, uint8_t base, uint8_t count) = 0;

    // ---- 以下为 DMA 直连 FIFO（把 PIO 当内存搬运器用）所需 ----
    // 取该状态机 FIFO 的【字节道】地址，供 8 位 DMA 直接对接内存字节流。
    // is_tx=true → TX FIFO：RP2040 的窄写会把该字节复制到全部 4 个字节道，故左移(MSB first)
    //   的 OSR 取高位仍能拿到这个字节；is_tx=false → RX FIFO：左移收满 8 bit 后字节落在 [7:0]，
    //   取第 0 字节道即得。
    virtual volatile void* sm_fifo_byte_addr(uint8_t sm, bool is_tx) = 0;
    // 取该状态机 FIFO 的 DREQ 编号（DMA 由外设节流，收发各一条）
    virtual uint8_t sm_dreq(uint8_t sm, bool is_tx) = 0;

    // 获取实例名称
    virtual std::string get_name() const = 0;
    
    // 检查PIO是否就绪
    virtual bool is_ready() const = 0;
};

// PIO0实例
class HAL_PIO0 : public HAL_PIO {
public:
    static HAL_PIO0* getInstance();
    ~HAL_PIO0();
    
    bool init(uint8_t gpio_pin) override;
    void deinit() override;
    bool load_program(const pio_program_t* program, uint8_t* offset) override;
    void unload_program(const pio_program_t* program, uint8_t offset) override;
    bool claim_sm(uint8_t* sm) override;
    void unclaim_sm(uint8_t sm) override;
    bool sm_configure(uint8_t sm, const PIOStateMachineConfig& config) override;
    void sm_set_enabled(uint8_t sm, bool enabled) override;
    void sm_put_blocking(uint8_t sm, uint32_t data) override;
    bool sm_put_nonblocking(uint8_t sm, uint32_t data) override;
    uint32_t sm_get_blocking(uint8_t sm) override;
    bool sm_is_tx_fifo_full(uint8_t sm) override;
    bool sm_is_rx_fifo_empty(uint8_t sm) override;
    void sm_exec(uint8_t sm, uint16_t instr) override;
    void sm_clear_fifos(uint8_t sm) override;
    void sm_restart(uint8_t sm) override;
    void init_pin(uint8_t gpio) override;
    void sm_set_pindirs_out(uint8_t sm, uint8_t base, uint8_t count) override;
    volatile void* sm_fifo_byte_addr(uint8_t sm, bool is_tx) override;
    uint8_t sm_dreq(uint8_t sm, bool is_tx) override;

    std::string get_name() const override { return "PIO0"; }
    bool is_ready() const override { return initialized_; }
    
private:
    bool initialized_;
    uint8_t gpio_pin_;          // 初始化时设置的GPIO引脚
    pio_sm_config configs_[4];  // 4个状态机的配置
    bool sm_claimed_[4];        // 状态机占用状态
    
    static HAL_PIO0* instance_;
    
    // 私有构造函数（单例模式）
    HAL_PIO0();
    HAL_PIO0(const HAL_PIO0&) = delete;
    HAL_PIO0& operator=(const HAL_PIO0&) = delete;
};

// PIO1实例
class HAL_PIO1 : public HAL_PIO {
public:
    static HAL_PIO1* getInstance();
    ~HAL_PIO1();
    
    bool init(uint8_t gpio_pin) override;
    void deinit() override;
    bool load_program(const pio_program_t* program, uint8_t* offset) override;
    void unload_program(const pio_program_t* program, uint8_t offset) override;
    bool claim_sm(uint8_t* sm) override;
    void unclaim_sm(uint8_t sm) override;
    bool sm_configure(uint8_t sm, const PIOStateMachineConfig& config) override;
    void sm_set_enabled(uint8_t sm, bool enabled) override;
    void sm_put_blocking(uint8_t sm, uint32_t data) override;
    bool sm_put_nonblocking(uint8_t sm, uint32_t data) override;
    uint32_t sm_get_blocking(uint8_t sm) override;
    bool sm_is_tx_fifo_full(uint8_t sm) override;
    bool sm_is_rx_fifo_empty(uint8_t sm) override;
    void sm_exec(uint8_t sm, uint16_t instr) override;
    void sm_clear_fifos(uint8_t sm) override;
    void sm_restart(uint8_t sm) override;
    void init_pin(uint8_t gpio) override;
    void sm_set_pindirs_out(uint8_t sm, uint8_t base, uint8_t count) override;
    volatile void* sm_fifo_byte_addr(uint8_t sm, bool is_tx) override;
    uint8_t sm_dreq(uint8_t sm, bool is_tx) override;

    std::string get_name() const override { return "PIO1"; }
    bool is_ready() const override { return initialized_; }
    
private:
    bool initialized_;
    uint8_t gpio_pin_;          // 初始化时设置的GPIO引脚
    pio_sm_config configs_[4];  // 4个状态机的配置
    bool sm_claimed_[4];        // 状态机占用状态
    
    static HAL_PIO1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_PIO1();
    HAL_PIO1(const HAL_PIO1&) = delete;
    HAL_PIO1& operator=(const HAL_PIO1&) = delete;
};