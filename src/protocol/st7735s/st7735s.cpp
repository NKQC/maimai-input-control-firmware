#include "st7735s.h"
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <cstring>

ST7735S::ST7735S(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin)
    : spi_hal_(spi_hal), cs_pin_(cs_pin), dc_pin_(dc_pin), rst_pin_(rst_pin),
      initialized_(false), rotation_(ST7735S_ROTATION_0), 
      width_(ST7735S_WIDTH), height_(ST7735S_HEIGHT),
      framebuffer_(nullptr), use_framebuffer_(false) {
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
    
    // 配置SPI
    spi_hal_->set_frequency(ST7735S_SPI_SPEED);
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
    switch (rotation) {
        case ST7735S_ROTATION_0:
            madctl = 0x00;
            width_ = ST7735S_WIDTH;
            height_ = ST7735S_HEIGHT;
            break;
        case ST7735S_ROTATION_90:
            madctl = 0x60;
            width_ = ST7735S_HEIGHT;
            height_ = ST7735S_WIDTH;
            break;
        case ST7735S_ROTATION_180:
            madctl = 0xC0;
            width_ = ST7735S_WIDTH;
            height_ = ST7735S_HEIGHT;
            break;
        case ST7735S_ROTATION_270:
            madctl = 0xA0;
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
    // 基本初始化序列
    write_command(ST7735S_SLPOUT);
    sleep_ms(120);
    
    write_command(ST7735S_COLMOD);
    write_data(0x05);  // 16位颜色
    
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
    
    // 填充整个屏幕
    for (uint32_t i = 0; i < width_ * height_; i++) {
        if (!write_pixel_data(color.value)) {
            return false;
        }
    }
    
    return true;
}

bool ST7735S::draw_bitmap_rgb888(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* bitmap) {
    if (!is_ready() || !bitmap) {
        return false;
    }
    
    // 设置显示窗口
    if (!set_window(x, y, x + width - 1, y + height - 1)) {
        return false;
    }
    
    // 转换RGB888到RGB565并发送
    for (uint32_t i = 0; i < width * height; i++) {
        uint8_t r = bitmap[i * 3];
        uint8_t g = bitmap[i * 3 + 1];
        uint8_t b = bitmap[i * 3 + 2];
        
        // 转换为RGB565
        uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        
        if (!write_pixel_data(color565)) {
            return false;
        }
    }
    
    return true;
}

bool ST7735S::set_framebuffer(uint16_t* buffer) {
    framebuffer_ = buffer;
    use_framebuffer_ = (buffer != nullptr);
    return true;
}

// 缓冲区更新函数（已移除，LVGL不需要）

bool ST7735S::set_backlight(uint8_t brightness) {
    return true;
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

// 辅助绘制函数和字体数据（已移除，LVGL不需要）