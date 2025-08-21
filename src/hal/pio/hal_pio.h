#pragma once

#include <stdint.h>
#include <string>
#include <hardware/pio.h>

/**
 * HAL层 - PIO接口抽象类
 * 提供底层PIO接口，支持PIO0和PIO1两个实例
 * 允许外部传入ASM程序实现自定义协议
 */

class HAL_PIO {
public:
    virtual ~HAL_PIO() = default;
    
    // 初始化PIO接口
    virtual bool init() = 0;
    
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
    
    // 配置状态机
    virtual void sm_config_set_out_pins(uint8_t sm, uint8_t out_base, uint8_t out_count) = 0;
    virtual void sm_config_set_in_pins(uint8_t sm, uint8_t in_base) = 0;
    virtual void sm_config_set_set_pins(uint8_t sm, uint8_t set_base, uint8_t set_count) = 0;
    virtual void sm_config_set_sideset_pins(uint8_t sm, uint8_t sideset_base) = 0;
    virtual void sm_config_set_clkdiv(uint8_t sm, float div) = 0;
    virtual void sm_config_set_wrap(uint8_t sm, uint8_t wrap_target, uint8_t wrap) = 0;
    
    // 启动/停止状态机
    virtual void sm_set_enabled(uint8_t sm, bool enabled) = 0;
    
    // 数据传输
    virtual void sm_put_blocking(uint8_t sm, uint32_t data) = 0;
    virtual uint32_t sm_get_blocking(uint8_t sm) = 0;
    virtual bool sm_is_tx_fifo_full(uint8_t sm) = 0;
    virtual bool sm_is_rx_fifo_empty(uint8_t sm) = 0;
    
    // GPIO配置
    virtual void gpio_init(uint8_t pin) = 0;
    virtual void gpio_set_function(uint8_t pin, uint8_t func) = 0;
    
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
    
    bool init() override;
    void deinit() override;
    bool load_program(const pio_program_t* program, uint8_t* offset) override;
    void unload_program(const pio_program_t* program, uint8_t offset) override;
    bool claim_sm(uint8_t* sm) override;
    void unclaim_sm(uint8_t sm) override;
    void sm_config_set_out_pins(uint8_t sm, uint8_t out_base, uint8_t out_count) override;
    void sm_config_set_in_pins(uint8_t sm, uint8_t in_base) override;
    void sm_config_set_set_pins(uint8_t sm, uint8_t set_base, uint8_t set_count) override;
    void sm_config_set_sideset_pins(uint8_t sm, uint8_t sideset_base) override;
    void sm_config_set_clkdiv(uint8_t sm, float div) override;
    void sm_config_set_wrap(uint8_t sm, uint8_t wrap_target, uint8_t wrap) override;
    void sm_set_enabled(uint8_t sm, bool enabled) override;
    void sm_put_blocking(uint8_t sm, uint32_t data) override;
    uint32_t sm_get_blocking(uint8_t sm) override;
    bool sm_is_tx_fifo_full(uint8_t sm) override;
    bool sm_is_rx_fifo_empty(uint8_t sm) override;
    void gpio_init(uint8_t pin) override;
    void gpio_set_function(uint8_t pin, uint8_t func) override;
    std::string get_name() const override { return "PIO0"; }
    bool is_ready() const override { return initialized_; }
    
private:
    bool initialized_;
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
    
    bool init() override;
    void deinit() override;
    bool load_program(const pio_program_t* program, uint8_t* offset) override;
    void unload_program(const pio_program_t* program, uint8_t offset) override;
    bool claim_sm(uint8_t* sm) override;
    void unclaim_sm(uint8_t sm) override;
    void sm_config_set_out_pins(uint8_t sm, uint8_t out_base, uint8_t out_count) override;
    void sm_config_set_in_pins(uint8_t sm, uint8_t in_base) override;
    void sm_config_set_set_pins(uint8_t sm, uint8_t set_base, uint8_t set_count) override;
    void sm_config_set_sideset_pins(uint8_t sm, uint8_t sideset_base) override;
    void sm_config_set_clkdiv(uint8_t sm, float div) override;
    void sm_config_set_wrap(uint8_t sm, uint8_t wrap_target, uint8_t wrap) override;
    void sm_set_enabled(uint8_t sm, bool enabled) override;
    void sm_put_blocking(uint8_t sm, uint32_t data) override;
    uint32_t sm_get_blocking(uint8_t sm) override;
    bool sm_is_tx_fifo_full(uint8_t sm) override;
    bool sm_is_rx_fifo_empty(uint8_t sm) override;
    void gpio_init(uint8_t pin) override;
    void gpio_set_function(uint8_t pin, uint8_t func) override;
    std::string get_name() const override { return "PIO1"; }
    bool is_ready() const override { return initialized_; }
    
private:
    bool initialized_;
    pio_sm_config configs_[4];  // 4个状态机的配置
    bool sm_claimed_[4];        // 状态机占用状态
    
    static HAL_PIO1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_PIO1();
    HAL_PIO1(const HAL_PIO1&) = delete;
    HAL_PIO1& operator=(const HAL_PIO1&) = delete;
};