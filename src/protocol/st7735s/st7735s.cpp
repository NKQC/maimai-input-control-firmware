#include "st7735s.h"
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <cstring>

ST7735S::ST7735S(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin)
    : spi_hal_(spi_hal), cs_pin_(cs_pin), dc_pin_(dc_pin), rst_pin_(rst_pin),
      initialized_(false), rotation_(ST7735S_ROTATION_0), 
      width_(ST7735S_WIDTH), height_(ST7735S_HEIGHT),
      framebuffer_(nullptr), use_framebuffer_(false),
      dma_busy_(false), dma_queue_head_(0), dma_queue_tail_(0), dma_queue_count_(0) {
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
    
    // 如果DMA不忙，使用DMA异步填充以提高效率
    if (!is_dma_busy()) {
        // 直接使用DMA异步填充，不等待完成
        if (fill_screen_async(color, nullptr)) {
            return true;  // DMA传输已启动，立即返回
        }
        // 如果DMA失败，继续使用传统方式
    }
    
    // 传统方式：逐个像素填充（DMA忙碌或不可用时）
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
    
    uint32_t pixel_count = width * height;
    
    // 对于大图像且DMA可用时，使用DMA优化传输
    if (pixel_count > 32 && !is_dma_busy()) {
        // 分配临时缓冲区用于RGB565数据
        static uint8_t rgb565_buffer[1024];  // 512个像素的RGB565数据
        const uint32_t buffer_pixels = 512;
        
        uint32_t processed_pixels = 0;
        
        auto process_chunk = [&]() -> bool {
            uint32_t chunk_pixels = (pixel_count - processed_pixels > buffer_pixels) ? 
                                   buffer_pixels : (pixel_count - processed_pixels);
            
            // 转换当前块的RGB888到RGB565
            for (uint32_t i = 0; i < chunk_pixels; i++) {
                uint32_t src_idx = (processed_pixels + i) * 3;
                uint8_t r = bitmap[src_idx];
                uint8_t g = bitmap[src_idx + 1];
                uint8_t b = bitmap[src_idx + 2];
                
                uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
                rgb565_buffer[i * 2] = (color565 >> 8) & 0xFF;
                rgb565_buffer[i * 2 + 1] = color565 & 0xFF;
            }
            
            // 直接使用DMA异步传输，不等待完成
            if (write_data_buffer_async(rgb565_buffer, chunk_pixels * 2, nullptr)) {
                processed_pixels += chunk_pixels;
                return true;  // DMA传输已启动，立即返回
            }
            return false;
        };
        
        // 处理所有块
        while (processed_pixels < pixel_count) {
            if (!process_chunk()) {
                // DMA失败，回退到传统方式处理剩余像素
                break;
            }
        }
        
        // 如果全部通过DMA完成
        if (processed_pixels >= pixel_count) {
            return true;
        }
        
        // 处理剩余像素（如果有的话）
        for (uint32_t i = processed_pixels; i < pixel_count; i++) {
            uint8_t r = bitmap[i * 3];
            uint8_t g = bitmap[i * 3 + 1];
            uint8_t b = bitmap[i * 3 + 2];
            
            uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            
            if (!write_pixel_data(color565)) {
                return false;
            }
        }
        
        return true;
    }
    
    // 传统方式：逐个像素转换并发送（小图像或DMA不可用时）
    for (uint32_t i = 0; i < pixel_count; i++) {
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
    uint16_t color565 = color.value;
    
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

bool ST7735S::draw_bitmap_async(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* bitmap, dma_callback_t callback) {
    if (!is_ready() || is_dma_busy() || !bitmap) {
        return false;
    }
    
    // 设置绘制窗口
    if (!set_window(x, y, x + width - 1, y + height - 1)) {
        return false;
    }
    
    // 计算数据大小（假设是RGB565格式）
    size_t data_size = width * height * 2;
    
    // 直接使用DMA异步传输位图数据
    return write_data_buffer_async(bitmap, data_size, callback);
}

// 辅助绘制函数和字体数据（已移除，LVGL不需要）