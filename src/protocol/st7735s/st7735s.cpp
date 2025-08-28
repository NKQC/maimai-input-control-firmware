#include "st7735s.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <functional>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"

// 移除静态帧缓冲区，改为直接传入缓冲区


ST7735S::ST7735S(HAL_SPI* spi_hal, ST7735S_Rotation rotation, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin, uint8_t bl_pin)
    : spi_hal_(spi_hal), cs_pin_(cs_pin), dc_pin_(dc_pin), rst_pin_(rst_pin), blk_pin_(bl_pin),
      initialized_(false), current_brightness_(255), rotation_(rotation),
      dma_busy_(false), current_callback_(nullptr) {}

ST7735S::~ST7735S() {
    deinit();
}

bool ST7735S::init() {
    if (initialized_) {
        return true;
    }
    
    if (!spi_hal_ || !spi_hal_->is_ready()) {
        return false;
    }
    
    // 配置引脚
    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    gpio_put(cs_pin_, 1);  // CS高电平（未选中）
    
    gpio_init(dc_pin_);
    gpio_set_dir(dc_pin_, GPIO_OUT);
    
    if (rst_pin_ != 255) {
        gpio_init(rst_pin_);
        gpio_set_dir(rst_pin_, GPIO_OUT);
        gpio_put(rst_pin_, 1);
    }
    
    // 配置背光引脚
    if (blk_pin_ != 255) {
        gpio_init(blk_pin_);
        gpio_set_function(blk_pin_, GPIO_FUNC_PWM);
        
        // 获取PWM slice和channel
        uint slice_num = pwm_gpio_to_slice_num(blk_pin_);
        uint channel = pwm_gpio_to_channel(blk_pin_);
        
        // 配置PWM: 125MHz / 62.5 = 2MHz, 2MHz / 1000 = 2kHz PWM频率
        pwm_set_clkdiv(slice_num, 62.5f);
        pwm_set_wrap(slice_num, 999);  // 0-999 = 1000 levels
        
        // 设置初始亮度为最大
        pwm_set_chan_level(slice_num, channel, (current_brightness_ * 999) / 255);
        
        // 启用PWM
        pwm_set_enabled(slice_num, true);
    }
    
    // 配置SPI
    spi_hal_->set_format(8, 0, 0);  // 8位，模式0
    
    // 硬件复位
    if (!reset()) {
        return false;
    }
    
    // 初始化寄存器
    if (!init_registers()) {
        return false;
    }
    initialized_ = true;
    return true;
}

void ST7735S::deinit() {
    if (initialized_) {
        display_on(false);
        
        // 关闭背光
        if (blk_pin_ != 255) {
            uint slice_num = pwm_gpio_to_slice_num(blk_pin_);
            pwm_set_enabled(slice_num, false);
            gpio_set_function(blk_pin_, GPIO_FUNC_NULL);
        }
        
        // 重置引脚
        gpio_put(cs_pin_, 1);
        
        initialized_ = false;
    }
}

bool ST7735S::is_ready() const {
    return initialized_ && spi_hal_ && spi_hal_->is_ready();
}

bool ST7735S::is_dma_busy() const {
    return dma_busy_ || (spi_hal_ && spi_hal_->is_busy());
}

bool ST7735S::reset() {
    if (rst_pin_ != 255) {
        gpio_put(rst_pin_, 0);
        sleep_ms(100);
        gpio_put(rst_pin_, 1);
        sleep_ms(50);
    } else {
        // 软件复位
        write_command(ST7735S_SWRESET);
        sleep_ms(120);
    }
    return true;
}

bool ST7735S::sleep(bool enable) {
    if (!is_ready()) {
        return false;
    }
    
    return write_command(enable ? ST7735S_SLPIN : ST7735S_SLPOUT);
}

bool ST7735S::display_on(bool enable) {
    if (!is_ready()) {
        return false;
    }
    
    return write_command(enable ? ST7735S_DISPON : ST7735S_DISPOFF);
}

bool ST7735S::invert_display(bool enable) {
    if (!is_ready()) {
        return false;
    }
    
    return write_command(enable ? ST7735S_INVON : ST7735S_INVOFF);
}

bool ST7735S::set_rotation(ST7735S_Rotation rotation) {
    uint8_t madctl = 0;

    switch (rotation) {
        case ST7735S_ROTATION_0:    // 竖屏 (USE_HORIZONTAL==0)
            madctl = 0x08;  // 对应厂家程序的0x08
            width_ = ST7735S_HEIGHT;    // 竖屏时宽度为80
            height_ = ST7735S_WIDTH;  // 竖屏时高度为160
            break;
        case ST7735S_ROTATION_90:   // 横屏 (USE_HORIZONTAL==2)
            madctl = 0x78;  // 对应厂家程序的0x78
            width_ = ST7735S_WIDTH;   // 横屏时宽度为160
            height_ = ST7735S_HEIGHT;   // 横屏时高度为80
            break;
        case ST7735S_ROTATION_180:  // 竖屏翻转180度 (USE_HORIZONTAL==1)
            madctl = 0xC8;  // 对应厂家程序的0xC8
            width_ = ST7735S_HEIGHT;
            height_ = ST7735S_WIDTH;
            break;
        case ST7735S_ROTATION_270:  // 横屏翻转180度 (USE_HORIZONTAL==3)
            madctl = 0xA8;  // 对应厂家程序的0xA8
            width_ = ST7735S_WIDTH;
            height_ = ST7735S_HEIGHT;
            break;
    }
    
    write_command(ST7735S_MADCTL);
    return write_data(madctl);
}

ST7735S_Rotation ST7735S::get_rotation() const {
    return rotation_;
}

uint16_t ST7735S::get_width() const {
    return width_;
}

uint16_t ST7735S::get_height() const {
    return height_;
}

bool ST7735S::write_command(uint8_t cmd) {
    if (!spi_hal_) {
        return false;
    }
    
    // 检查DMA状态，如果忙碌则立即失败，让调用方稍后重试
    if (is_dma_busy()) {
        return false;
    }
    
    gpio_put(dc_pin_, 0);  // 命令模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    // 命令始终使用同步传输
    bool result = spi_hal_->write(&cmd, 1) == 1;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

bool ST7735S::write_data(uint8_t data) {
    if (!spi_hal_) {
        return false;
    }
    
    // 检查DMA状态，如果忙碌则立即失败，让调用方稍后重试
    if (is_dma_busy()) {
        return false;
    }
    
    gpio_put(dc_pin_, 1);  // 数据模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    // 单字节数据使用同步传输
    bool result = spi_hal_->write(&data, 1) == 1;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

bool ST7735S::init_registers() {
    // 软件复位
    write_command(ST7735S_SLPOUT);  // 0x11
    sleep_ms(120);
    
    // 反色显示
    write_command(ST7735S_INVON);   // 0x21
    write_command(ST7735S_INVON);   // 0x21
    
    // 帧速率控制1 - 正常模式
    write_command(ST7735S_FRMCTR1); // 0xB1
    write_data(0x05);
    write_data(0x3A);
    write_data(0x3A);
    
    // 帧速率控制2 - 空闲模式
    write_command(ST7735S_FRMCTR2); // 0xB2
    write_data(0x05);
    write_data(0x3A);
    write_data(0x3A);
    
    // 帧速率控制3 - 部分模式
    write_command(ST7735S_FRMCTR3); // 0xB3
    write_data(0x05);
    write_data(0x3A);
    write_data(0x3A);
    write_data(0x05);
    write_data(0x3A);
    write_data(0x3A);
    
    // 列反转控制
    write_command(ST7735S_INVCTR);  // 0xB4
    write_data(0x03);
    
    // 电源控制1
    write_command(ST7735S_PWCTR1);  // 0xC0
    write_data(0x62);
    write_data(0x02);
    write_data(0x04);
    
    // 电源控制2
    write_command(ST7735S_PWCTR2);  // 0xC1
    write_data(0xC0);
    
    // 电源控制3
    write_command(ST7735S_PWCTR3);  // 0xC2
    write_data(0x0D);
    write_data(0x00);
    
    // 电源控制4
    write_command(ST7735S_PWCTR4);  // 0xC3
    write_data(0x8D);
    write_data(0x6A);
    
    // 电源控制5
    write_command(ST7735S_PWCTR5);  // 0xC4
    write_data(0x8D);
    write_data(0xEE);
    
    // VCOM控制
    write_command(ST7735S_VMCTR1);  // 0xC5
    write_data(0x0E);
    
    // Gamma正极性校正
    write_command(ST7735S_GMCTRP1); // 0xE0
    write_data(0x10);
    write_data(0x0E);
    write_data(0x02);
    write_data(0x03);
    write_data(0x0E);
    write_data(0x07);
    write_data(0x02);
    write_data(0x07);
    write_data(0x0A);
    write_data(0x12);
    write_data(0x27);
    write_data(0x37);
    write_data(0x00);
    write_data(0x0D);
    write_data(0x0E);
    write_data(0x10);
    
    // Gamma负极性校正
    write_command(ST7735S_GMCTRN1); // 0xE1
    write_data(0x10);
    write_data(0x0E);
    write_data(0x03);
    write_data(0x03);
    write_data(0x0F);
    write_data(0x06);
    write_data(0x02);
    write_data(0x08);
    write_data(0x0A);
    write_data(0x13);
    write_data(0x26);
    write_data(0x36);
    write_data(0x00);
    write_data(0x0D);
    write_data(0x0E);
    write_data(0x10);
    
    // 颜色模式设置为16位
    write_command(ST7735S_COLMOD);  // 0x3A
    write_data(0x05);  // 16位颜色 RGB565
    
    set_rotation(rotation_);
    write_command(ST7735S_DISPON);
    return true;
}

// 设置显示窗口
bool ST7735S::set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (!is_ready()) {
        return false;
    }
    
    // 根据显示方向设置正确的偏移量
    // 参考厂家程序LCD_Address_Set函数
    if (rotation_ > ST7735S_ROTATION_180) {
        // 竖屏模式：列偏移26，行偏移1
        write_command(ST7735S_CASET);
        write_data(0x00);
        write_data(x0+1);
        write_data(0x00);
        write_data(x1+1);
        
        write_command(ST7735S_RASET);
        write_data(0x00);
        write_data(y0+0x1A);
        write_data(0x00);
        write_data(y1+0x1A);

        write_command(ST7735S_RAMWR);
    } else {
        // 横屏模式：列偏移1，行偏移26
        write_command(ST7735S_CASET);
        write_data(0x00);
        write_data(x0+0x1A);
        write_data(0x00);
        write_data(x1+0x1A);
        
        write_command(ST7735S_RASET);
        write_data(0x00);
        write_data(y0+1);
        write_data(0x00);
        write_data(y1+1);
    }
    // 开始写入RAM
    write_command(ST7735S_RAMWR);

    return true;
}

// 新的直接写入接口实现
bool ST7735S::write_buffer(const uint16_t* buffer, size_t buffer_size, dma_callback_t callback) {
    if (!is_ready() || !buffer || buffer_size == 0) {
        return false;
    }
    
    // 如果DMA忙碌，返回false
    if (is_dma_busy()) {
        return false;
    }

    // 设置全屏窗口（此处会使用同步SPI命令，不能在dma_busy_置位后调用）
    if (!set_window(0, 0, width_ - 1, height_ - 1)) {
        return false;
    }
    
    // 选择数据模式
    select_data_mode();
    
    // 保存回调函数
    current_callback_ = callback;
    
    // 创建DMA完成回调，确保传输完成后取消片选
    auto dma_callback = [this](bool success) {
        this->on_dma_complete(success);
    };
    
    // 启动一次性DMA传输
    const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer);
    size_t data_size = buffer_size * sizeof(uint16_t);
    
    bool result = spi_hal_->start_dma_transfer(data, data_size, dma_callback);
    
    if (result) {
        // 仅在DMA成功启动后，才标记为忙碌，避免阻塞后续同步命令
        dma_busy_ = true;
    } else {
        // 传输启动失败，取消片选并清理状态
        deselect();
        current_callback_ = nullptr;
        dma_busy_ = false;
    }
    
    return result;
}

bool ST7735S::write_buffer_region(const uint16_t* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height, dma_callback_t callback) {
    if (!is_ready() || !buffer || width == 0 || height == 0) {
        return false;
    }
    
    // 检查边界
    if (x >= width_ || y >= height_ || (x + width) > width_ || (y + height) > height_) {
        return false;
    }
    
    // 如果DMA忙碌，返回false
    if (is_dma_busy()) {
        return false;
    }
    
    // 设置指定区域窗口
    if (!set_window(x, y, x + width - 1, y + height - 1)) {
        return false;
    }

    // 选择数据模式
    select_data_mode();
    
    // 保存回调函数
    current_callback_ = callback;
    
    // 创建DMA完成回调，确保传输完成后取消片选
    auto dma_callback = [this](bool success) {
        this->on_dma_complete(success);
    };
    
    // 启动一次性DMA传输
    const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer);
    size_t data_size = width * height * sizeof(uint16_t);
    
    bool result = spi_hal_->start_dma_transfer(data, data_size, dma_callback);
    
    if (result) {
        dma_busy_ = true;
    } else {
        // 传输启动失败，取消片选
        deselect();
        current_callback_ = nullptr;
    }
    
    return result;
}

bool ST7735S::set_backlight(uint8_t brightness) {
    if (blk_pin_ == 255) {
        // 没有配置背光引脚，直接返回成功
        current_brightness_ = brightness;
        return true;
    }
    
    // 获取PWM slice和channel
    uint slice_num = pwm_gpio_to_slice_num(blk_pin_);
    uint channel = pwm_gpio_to_channel(blk_pin_);
    
    // 将0-255亮度值映射到0-999 PWM级别
    uint16_t pwm_level = (brightness * 999) / 255;
    
    // 设置PWM占空比
    pwm_set_chan_level(slice_num, channel, pwm_level);
    
    current_brightness_ = brightness;
    return true;
}

uint8_t ST7735S::get_backlight() const {
    return current_brightness_;
}

bool ST7735S::write_data_16(uint16_t data) {
    uint8_t high = (data >> 8) & 0xFF;
    uint8_t low = data & 0xFF;
    return write_data(high) && write_data(low);
}

bool ST7735S::write_data_buffer(const uint8_t* buffer, size_t length) {
    if (!spi_hal_ || !buffer) {
        return false;
    }
    
    // 简化版本：直接使用同步传输
    gpio_put(dc_pin_, 1);  // 数据模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    bool result = spi_hal_->write(buffer, length) == length;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

// 旧的write_framebuffer_dma函数已移除，使用新的write_buffer接口

bool ST7735S::spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    if (!spi_hal_) {
        return false;
    }
    
    gpio_put(cs_pin_, 0);  // 选中设备
    
    bool result = spi_hal_->transfer(tx_data, rx_data, length) == length;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

void ST7735S::on_dma_complete(bool success) {
    dma_busy_ = false;
    deselect();  // 取消片选
    
    if (current_callback_) {
        auto callback = current_callback_;
        current_callback_ = nullptr;
        callback(success);
    }
}

void ST7735S::process_dma() {
    // 简化版本：DMA完成事件由HAL层的回调自动处理
    // 这个方法保留用于兼容性，实际不需要在主循环中调用
}



void ST7735S::select_cmd_mode() {
    gpio_put(cs_pin_, 0);
    gpio_put(dc_pin_, 0);
}

void ST7735S::select_data_mode() {
    gpio_put(cs_pin_, 0);
    gpio_put(dc_pin_, 1);
}

void ST7735S::deselect() {
    gpio_put(cs_pin_, 1);
}
