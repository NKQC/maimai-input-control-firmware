#include "st7735s.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"


ST7735S::ST7735S(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin, uint8_t bl_pin)
    : spi_hal_(spi_hal), cs_pin_(cs_pin), dc_pin_(dc_pin), rst_pin_(rst_pin), blk_pin_(bl_pin),
      initialized_(false), current_brightness_(255), rotation_(ST7735S_ROTATION_0),
      width_(ST7735S_WIDTH), height_(ST7735S_HEIGHT), framebuffer_(nullptr), use_framebuffer_(false),
      dma_busy_(false), current_callback_(nullptr), dma_queue_head_(0), dma_queue_tail_(0), dma_queue_count_(0) {
}

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
    
    // 配置显示
    if (!configure_display()) {
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
        framebuffer_ = nullptr;
        use_framebuffer_ = false;
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
        sleep_ms(10);
        gpio_put(rst_pin_, 1);
        sleep_ms(120);
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
    if (!is_ready()) {
        return false;
    }
    
    rotation_ = rotation;
    
    uint8_t madctl = 0;
    // 根据ST7735S数据手册和参考代码设置MADCTL寄存器
    // MX, MY, RGB模式控制
    switch (rotation) {
        case ST7735S_ROTATION_0:    // 竖屏
            madctl = 0xC8;  // MY=1, MX=1, MV=0, ML=0, RGB=1, MH=0
            width_ = ST7735S_WIDTH;
            height_ = ST7735S_HEIGHT;
            break;
        case ST7735S_ROTATION_90:   // 横屏
            madctl = 0xA8;  // MY=1, MX=0, MV=1, ML=0, RGB=1, MH=0
            width_ = ST7735S_HEIGHT;
            height_ = ST7735S_WIDTH;
            break;
        case ST7735S_ROTATION_180:  // 竖屏翻转180度
            madctl = 0x08;  // MY=0, MX=0, MV=0, ML=0, RGB=1, MH=0
            width_ = ST7735S_WIDTH;
            height_ = ST7735S_HEIGHT;
            break;
        case ST7735S_ROTATION_270:  // 横屏翻转180度
            madctl = 0x68;  // MY=0, MX=1, MV=1, ML=0, RGB=1, MH=0
            width_ = ST7735S_HEIGHT;
            height_ = ST7735S_WIDTH;
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
    
    gpio_put(dc_pin_, 0);  // 命令模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    bool result = spi_hal_->write(&cmd, 1) == 1;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

bool ST7735S::write_data(uint8_t data) {
    if (!spi_hal_) {
        return false;
    }
    
    gpio_put(dc_pin_, 1);  // 数据模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    bool result = spi_hal_->write(&data, 1) == 1;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

bool ST7735S::init_registers() {
    // 软件复位
    write_command(ST7735S_SWRESET);
    sleep_ms(150);
    
    // 退出睡眠模式
    write_command(ST7735S_SLPOUT);
    sleep_ms(120);
    
    // ST7735S 帧速率控制
    write_command(ST7735S_FRMCTR1);  // 正常模式帧速率
    write_data(0x01);
    write_data(0x2C);
    write_data(0x2D);
    
    write_command(ST7735S_FRMCTR2);  // 空闲模式帧速率
    write_data(0x01);
    write_data(0x2C);
    write_data(0x2D);
    
    write_command(ST7735S_FRMCTR3);  // 部分模式帧速率
    write_data(0x01);
    write_data(0x2C);
    write_data(0x2D);
    write_data(0x01);
    write_data(0x2C);
    write_data(0x2D);
    
    write_command(ST7735S_INVCTR);   // 列反转控制
    write_data(0x07);
    
    // ST7735S 电源序列
    write_command(ST7735S_PWCTR1);
    write_data(0xA2);
    write_data(0x02);
    write_data(0x84);
    
    write_command(ST7735S_PWCTR2);
    write_data(0xC5);
    
    write_command(ST7735S_PWCTR3);
    write_data(0x0A);
    write_data(0x00);
    
    write_command(ST7735S_PWCTR4);
    write_data(0x8A);
    write_data(0x2A);
    
    write_command(ST7735S_PWCTR5);
    write_data(0x8A);
    write_data(0xEE);
    
    // VCOM 控制
    write_command(ST7735S_VMCTR1);
    write_data(0x0E);
    
    // 颜色模式设置为16位
    write_command(ST7735S_COLMOD);
    write_data(0x05);  // 16位颜色 RGB565
    
    // Gamma 正极性校正
    write_command(ST7735S_GMCTRP1);
    write_data(0x0F);
    write_data(0x1A);
    write_data(0x0F);
    write_data(0x18);
    write_data(0x2F);
    write_data(0x28);
    write_data(0x20);
    write_data(0x22);
    write_data(0x1F);
    write_data(0x1B);
    write_data(0x23);
    write_data(0x37);
    write_data(0x00);
    write_data(0x07);
    write_data(0x02);
    write_data(0x10);
    
    // Gamma 负极性校正
    write_command(ST7735S_GMCTRN1);
    write_data(0x0F);
    write_data(0x1B);
    write_data(0x0F);
    write_data(0x17);
    write_data(0x33);
    write_data(0x2C);
    write_data(0x29);
    write_data(0x2E);
    write_data(0x30);
    write_data(0x30);
    write_data(0x39);
    write_data(0x3F);
    write_data(0x00);
    write_data(0x07);
    write_data(0x03);
    write_data(0x10);
    
    // 正常显示模式
    write_command(ST7735S_NORON);
    sleep_ms(10);
    
    return true;
}

bool ST7735S::configure_display() {
    // 设置默认方向
    set_rotation(ST7735S_ROTATION_0);
    
    // 开启显示
    display_on(true);
    
    return true;
}

// 设置显示窗口
bool ST7735S::set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (!is_ready()) {
        return false;
    }
    
    // 设置列地址
    write_command(ST7735S_CASET);
    write_data_16(x0);
    write_data_16(x1);
    
    // 设置行地址
    write_command(ST7735S_RASET);
    write_data_16(y0);
    write_data_16(y1);
    
    // 开始写入RAM
    write_command(ST7735S_RAMWR);
    
    return true;
}

bool ST7735S::fill_screen(ST7735S_Color color) {
    if (!is_ready()) {
        return false;
    }
    
    // 设置全屏窗口
    if (!set_window(0, 0, width_ - 1, height_ - 1)) {
        return false;
    }
    
    // 如果DMA不忙，使用DMA异步填充以提高效率
    if (!is_dma_busy()) {
        // 直接使用DMA异步填充，不等待完成
        if (fill_screen_async(color, nullptr)) {
            return true;  // DMA传输已启动，立即返回
        }
    }
    return false;
}

// RGB888相关函数已移除，现在只支持RGB565格式

bool ST7735S::set_framebuffer(uint16_t* buffer) {
    framebuffer_ = buffer;
    use_framebuffer_ = (buffer != nullptr);
    return true;
}

// 缓冲区更新函数（已移除，LVGL不需要）

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
    
    // 对于大数据量（>64字节）且DMA可用时，尝试使用DMA异步传输
    if (length > 64 && !is_dma_busy()) {
        // 直接使用DMA异步传输，不等待完成
        if (write_data_buffer_async(buffer, length, nullptr)) {
            return true;  // DMA传输已启动，立即返回
        }
        // 如果DMA失败，继续使用同步方式
    }
    
    // 使用同步SPI传输（小数据量或DMA不可用）
    gpio_put(dc_pin_, 1);  // 数据模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    bool result = spi_hal_->write(buffer, length) == length;
    
    gpio_put(cs_pin_, 1);  // 取消选中
    return result;
}

// 写入像素数据（用于LVGL集成）
bool ST7735S::write_pixel_data(uint16_t color565) {
    return write_data_16(color565);
}

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
    gpio_put(cs_pin_, 1);  // 取消选中设备
    
    if (current_callback_) {
        current_callback_(success);
        current_callback_ = nullptr;
    }
}

bool ST7735S::write_data_buffer_async(const uint8_t* buffer, size_t length, dma_callback_t callback) {
    if (!is_ready() || is_dma_busy() || !buffer || length == 0) {
        return false;
    }
    
    dma_busy_ = true;
    current_callback_ = callback;
    
    gpio_put(dc_pin_, 1);  // 数据模式
    gpio_put(cs_pin_, 0);  // 选中设备
    
    // 使用DMA异步写入
    auto dma_callback = [this](bool success) {
        this->on_dma_complete(success);
    };
    
    if (!spi_hal_->write_dma(buffer, length, dma_callback)) {
        dma_busy_ = false;
        current_callback_ = nullptr;
        gpio_put(cs_pin_, 1);  // 取消选中
        return false;
    }
    
    return true;
}

bool ST7735S::fill_screen_async(ST7735S_Color color, dma_callback_t callback) {
    if (!is_ready() || is_dma_busy()) {
        return false;
    }
    
    // 设置全屏窗口
    if (!set_window(0, 0, width_ - 1, height_ - 1)) {
        return false;
    }
    
    // 准备颜色数据缓冲区 - 增大缓冲区以减少DMA传输次数
    static uint8_t color_buffer[2048];  // 1024个像素的颜色数据
    uint8_t* buffer_ptr = color_buffer;  // 使用指针避免捕获static变量
    uint16_t color565 = color;
    
    // 填充缓冲区
    for (int i = 0; i < 1024; i++) {
        color_buffer[i * 2] = (color565 >> 8) & 0xFF;
        color_buffer[i * 2 + 1] = color565 & 0xFF;
    }
    
    // 计算需要发送的次数
    uint32_t total_pixels = width_ * height_;
    uint32_t buffer_pixels = 1024;  // 增大缓冲区像素数
    uint32_t remaining_pixels = total_pixels;
    
    // 创建一个递归回调来处理多次DMA传输
    std::function<void(bool)> fill_callback = [this, buffer_ptr, buffer_pixels, remaining_pixels, callback, &fill_callback](bool success) mutable {
        if (!success) {
            if (callback) callback(false);
            return;
        }
        
        remaining_pixels -= buffer_pixels;
        if (remaining_pixels > 0) {
            uint32_t next_pixels = (remaining_pixels > buffer_pixels) ? buffer_pixels : remaining_pixels;
            uint32_t next_bytes = next_pixels * 2;
            
            // 继续下一次DMA传输
            this->write_data_buffer_async(buffer_ptr, next_bytes, fill_callback);
        } else {
            // 全部完成
            if (callback) callback(true);
        }
    };
    
    // 开始第一次DMA传输
    uint32_t first_pixels = (remaining_pixels > buffer_pixels) ? buffer_pixels : remaining_pixels;
    uint32_t first_bytes = first_pixels * 2;
    
    return write_data_buffer_async(buffer_ptr, first_bytes, fill_callback);
}


bool ST7735S::draw_bitmap_rgb565_async(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* bitmap, dma_callback_t callback) {
    if (!is_ready() || !bitmap) {
        if (callback) callback(false);
        return false;
    }
    
    // 设置显示窗口
    if (!set_window(x, y, x + width - 1, y + height - 1)) {
        if (callback) callback(false);
        return false;
    }
    
    // 计算数据大小（RGB565格式）
    size_t data_size = width * height * 2;
    
    // 使用异步DMA传输
    return write_data_buffer_async(bitmap, data_size, callback);
}

// 辅助绘制函数和字体数据（已移除，LVGL不需要）