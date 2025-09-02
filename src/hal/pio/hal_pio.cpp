#include "hal_pio.h"
#include <hardware/pio.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>

// HAL_PIO0 静态成员初始化
HAL_PIO0* HAL_PIO0::instance_ = nullptr;

// HAL_PIO0 实现
HAL_PIO0* HAL_PIO0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_PIO0();
    }
    return instance_;
}

HAL_PIO0::HAL_PIO0() : initialized_(false) {
    for (int i = 0; i < 4; i++) {
        sm_claimed_[i] = false;
    }
}

HAL_PIO0::~HAL_PIO0() {
    deinit();
}

bool HAL_PIO0::init(uint8_t gpio_pin) {
    if (initialized_) {
        return true;
    }
    
    // 保存GPIO引脚号
    gpio_pin_ = gpio_pin;
    
    // PIO0已经在系统启动时初始化，这里只需要标记为已初始化
    // 同时初始化指定的GPIO引脚
    pio_gpio_init(pio0, gpio_pin_);
    gpio_set_function(gpio_pin_, GPIO_FUNC_PIO0);
    
    initialized_ = true;
    return true;
}

void HAL_PIO0::deinit() {
    if (initialized_) {
        // 释放所有占用的状态机
        for (int i = 0; i < 4; i++) {
            if (sm_claimed_[i]) {
                unclaim_sm(i);
            }
        }
        initialized_ = false;
    }
}

bool HAL_PIO0::load_program(const pio_program_t* program, uint8_t* offset) {
    if (!initialized_) return false;
    
    uint program_offset = pio_add_program(pio0, program);
    if (program_offset == -1) {
        return false;
    }
    
    *offset = program_offset;
    return true;
}

void HAL_PIO0::unload_program(const pio_program_t* program, uint8_t offset) {
    if (initialized_) {
        pio_remove_program(pio0, program, offset);
    }
}

bool HAL_PIO0::claim_sm(uint8_t* sm) {
    if (!initialized_) return false;
    
    int claimed_sm = pio_claim_unused_sm(pio0, false);
    if (claimed_sm == -1) {
        return false;
    }
    
    *sm = claimed_sm;
    sm_claimed_[claimed_sm] = true;
    return true;
}

void HAL_PIO0::unclaim_sm(uint8_t sm) {
    if (initialized_ && sm < 4 && sm_claimed_[sm]) {
        pio_sm_unclaim(pio0, sm);
        sm_claimed_[sm] = false;
    }
}

bool HAL_PIO0::sm_configure(uint8_t sm, const PIOStateMachineConfig& config) {
    if (!initialized_ || sm >= 4) {
        return false;
    }
    
    // 获取默认配置
    configs_[sm] = pio_get_default_sm_config();
    
    // 配置引脚
    if (config.out_count > 0) {
        ::sm_config_set_out_pins(&configs_[sm], config.out_base, config.out_count);
    }
    if (config.in_base != 0) {
        ::sm_config_set_in_pins(&configs_[sm], config.in_base);
    }
    if (config.set_count > 0) {
        ::sm_config_set_set_pins(&configs_[sm], config.set_base, config.set_count);
    }
    if (config.sideset_bit_count > 0) {
        ::sm_config_set_sideset_pins(&configs_[sm], config.sideset_base);
        ::sm_config_set_sideset(&configs_[sm], config.sideset_bit_count, config.sideset_optional, config.sideset_pindirs);
    }
    
    // 配置时钟
    ::sm_config_set_clkdiv(&configs_[sm], config.clkdiv);
    
    // 配置程序包装
    ::sm_config_set_wrap(&configs_[sm], config.wrap_target, config.wrap);
    
    // 初始化状态机
    pio_sm_init(pio0, sm, config.program_offset, &configs_[sm]);
    
    // 启用状态机（如果配置要求）
    if (config.enabled) {
        pio_sm_set_enabled(pio0, sm, true);
    }
    
    return true;
}

void HAL_PIO0::sm_set_enabled(uint8_t sm, bool enabled) {
    if (initialized_ && sm < 4) {
        pio_sm_set_enabled(pio0, sm, enabled);
    }
}

void HAL_PIO0::sm_put_blocking(uint8_t sm, uint32_t data) {
    if (initialized_ && sm < 4) {
        pio_sm_put_blocking(pio0, sm, data);
    }
}

bool HAL_PIO0::sm_put_nonblocking(uint8_t sm, uint32_t data) {
    if (initialized_ && sm < 4) {
        if (!pio_sm_is_tx_fifo_full(pio0, sm)) {
            pio_sm_put(pio0, sm, data);
            return true;
        }
    }
    return false;
}

uint32_t HAL_PIO0::sm_get_blocking(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_get_blocking(pio0, sm);
    }
    return 0;
}

bool HAL_PIO0::sm_is_tx_fifo_full(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_is_tx_fifo_full(pio0, sm);
    }
    return true;
}

bool HAL_PIO0::sm_is_rx_fifo_empty(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_is_rx_fifo_empty(pio0, sm);
    }
    return true;
}



// HAL_PIO1 静态成员初始化
HAL_PIO1* HAL_PIO1::instance_ = nullptr;

// HAL_PIO1 实现
HAL_PIO1* HAL_PIO1::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_PIO1();
    }
    return instance_;
}

HAL_PIO1::HAL_PIO1() : initialized_(false) {
    for (int i = 0; i < 4; i++) {
        sm_claimed_[i] = false;
    }
}

HAL_PIO1::~HAL_PIO1() {
    deinit();
}

bool HAL_PIO1::init(uint8_t gpio_pin) {
    if (initialized_) {
        return true;
    }
    
    // 保存GPIO引脚号
    gpio_pin_ = gpio_pin;
    
    // PIO1已经在系统启动时初始化，这里只需要标记为已初始化
    // 同时初始化指定的GPIO引脚
    pio_gpio_init(pio1, gpio_pin_);
    gpio_set_function(gpio_pin_, GPIO_FUNC_PIO1);
    
    initialized_ = true;
    return true;
}

void HAL_PIO1::deinit() {
    if (initialized_) {
        // 释放所有占用的状态机
        for (int i = 0; i < 4; i++) {
            if (sm_claimed_[i]) {
                unclaim_sm(i);
            }
        }
        initialized_ = false;
    }
}

bool HAL_PIO1::load_program(const pio_program_t* program, uint8_t* offset) {
    if (!initialized_) return false;
    
    uint program_offset = pio_add_program(pio1, program);
    if (program_offset == -1) {
        return false;
    }
    
    *offset = program_offset;
    return true;
}

void HAL_PIO1::unload_program(const pio_program_t* program, uint8_t offset) {
    if (initialized_) {
        pio_remove_program(pio1, program, offset);
    }
}

bool HAL_PIO1::claim_sm(uint8_t* sm) {
    if (!initialized_) return false;
    
    int claimed_sm = pio_claim_unused_sm(pio1, false);
    if (claimed_sm == -1) {
        return false;
    }
    
    *sm = claimed_sm;
    sm_claimed_[claimed_sm] = true;
    return true;
}

void HAL_PIO1::unclaim_sm(uint8_t sm) {
    if (initialized_ && sm < 4 && sm_claimed_[sm]) {
        pio_sm_unclaim(pio1, sm);
        sm_claimed_[sm] = false;
    }
}

bool HAL_PIO1::sm_configure(uint8_t sm, const PIOStateMachineConfig& config) {
    if (!initialized_ || sm >= 4) {
        return false;
    }
    
    // 获取默认配置
    configs_[sm] = pio_get_default_sm_config();
    
    // 配置引脚
    if (config.out_count > 0) {
        ::sm_config_set_out_pins(&configs_[sm], config.out_base, config.out_count);
    }
    if (config.in_base != 0) {
        ::sm_config_set_in_pins(&configs_[sm], config.in_base);
    }
    if (config.set_count > 0) {
        ::sm_config_set_set_pins(&configs_[sm], config.set_base, config.set_count);
    }
    if (config.sideset_bit_count > 0) {
        ::sm_config_set_sideset_pins(&configs_[sm], config.sideset_base);
        ::sm_config_set_sideset(&configs_[sm], config.sideset_bit_count, config.sideset_optional, config.sideset_pindirs);
    }
    
    // 配置时钟
    ::sm_config_set_clkdiv(&configs_[sm], config.clkdiv);
    
    // 配置程序包装
    ::sm_config_set_wrap(&configs_[sm], config.wrap_target, config.wrap);
    
    // 初始化状态机
    pio_sm_init(pio1, sm, config.program_offset, &configs_[sm]);
    
    // 启用状态机（如果配置要求）
    if (config.enabled) {
        pio_sm_set_enabled(pio1, sm, true);
    }
    
    return true;
}

void HAL_PIO1::sm_set_enabled(uint8_t sm, bool enabled) {
    if (initialized_ && sm < 4) {
        pio_sm_set_enabled(pio1, sm, enabled);
    }
}

void HAL_PIO1::sm_put_blocking(uint8_t sm, uint32_t data) {
    if (initialized_ && sm < 4) {
        pio_sm_put_blocking(pio1, sm, data);
    }
}

bool HAL_PIO1::sm_put_nonblocking(uint8_t sm, uint32_t data) {
    if (initialized_ && sm < 4) {
        if (!pio_sm_is_tx_fifo_full(pio1, sm)) {
            pio_sm_put(pio1, sm, data);
            return true;
        }
    }
    return false;
}

uint32_t HAL_PIO1::sm_get_blocking(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_get_blocking(pio1, sm);
    }
    return 0;
}

bool HAL_PIO1::sm_is_tx_fifo_full(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_is_tx_fifo_full(pio1, sm);
    }
    return true;
}

bool HAL_PIO1::sm_is_rx_fifo_empty(uint8_t sm) {
    if (initialized_ && sm < 4) {
        return pio_sm_is_rx_fifo_empty(pio1, sm);
    }
    return true;
}