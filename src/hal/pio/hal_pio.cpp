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

bool HAL_PIO0::init() {
    if (initialized_) {
        return true;
    }
    
    // PIO0已经在系统启动时初始化，这里只需要标记为已初始化
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

void HAL_PIO0::sm_config_set_out_pins(uint8_t sm, uint8_t out_base, uint8_t out_count) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_out_pins(&configs_[sm], out_base, out_count);
    }
}

void HAL_PIO0::sm_config_set_in_pins(uint8_t sm, uint8_t in_base) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_in_pins(&configs_[sm], in_base);
    }
}

void HAL_PIO0::sm_config_set_set_pins(uint8_t sm, uint8_t set_base, uint8_t set_count) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_set_pins(&configs_[sm], set_base, set_count);
    }
}

void HAL_PIO0::sm_config_set_sideset_pins(uint8_t sm, uint8_t sideset_base) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_sideset_pins(&configs_[sm], sideset_base);
    }
}

void HAL_PIO0::sm_config_set_clkdiv(uint8_t sm, float div) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_clkdiv(&configs_[sm], div);
    }
}

void HAL_PIO0::sm_config_set_wrap(uint8_t sm, uint8_t wrap_target, uint8_t wrap) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_wrap(&configs_[sm], wrap_target, wrap);
    }
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

void HAL_PIO0::gpio_init(uint8_t pin) {
    pio_gpio_init(pio0, pin);
}

void HAL_PIO0::gpio_set_function(uint8_t pin, uint8_t func) {
    gpio_set_function(pin, func);
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

bool HAL_PIO1::init() {
    if (initialized_) {
        return true;
    }
    
    // PIO1已经在系统启动时初始化，这里只需要标记为已初始化
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

void HAL_PIO1::sm_config_set_out_pins(uint8_t sm, uint8_t out_base, uint8_t out_count) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_out_pins(&configs_[sm], out_base, out_count);
    }
}

void HAL_PIO1::sm_config_set_in_pins(uint8_t sm, uint8_t in_base) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_in_pins(&configs_[sm], in_base);
    }
}

void HAL_PIO1::sm_config_set_set_pins(uint8_t sm, uint8_t set_base, uint8_t set_count) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_set_pins(&configs_[sm], set_base, set_count);
    }
}

void HAL_PIO1::sm_config_set_sideset_pins(uint8_t sm, uint8_t sideset_base) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_sideset_pins(&configs_[sm], sideset_base);
    }
}

void HAL_PIO1::sm_config_set_clkdiv(uint8_t sm, float div) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_clkdiv(&configs_[sm], div);
    }
}

void HAL_PIO1::sm_config_set_wrap(uint8_t sm, uint8_t wrap_target, uint8_t wrap) {
    if (initialized_ && sm < 4) {
        ::sm_config_set_wrap(&configs_[sm], wrap_target, wrap);
    }
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

void HAL_PIO1::gpio_init(uint8_t pin) {
    pio_gpio_init(pio1, pin);
}

void HAL_PIO1::gpio_set_function(uint8_t pin, uint8_t func) {
    gpio_set_function(pin, func);
}