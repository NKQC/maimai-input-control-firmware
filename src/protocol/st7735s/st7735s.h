#pragma once

#include "../../hal/spi/hal_spi.h"
#include <stdint.h>
#include <vector>
#include <functional>
#include <queue>

// RGB565类型定义

// RGB565别名，保持向后兼容
using RGB565 = uint16_t;

/**
 * 协议层 - ST7735S TFT LCD显示屏
 * 基于SPI通信的彩色LCD控制器
 * 支持128x160分辨率，16位RGB565颜色
 */

// ST7735S显示参数 - 根据厂家程序更新
#define ST7735S_WIDTH           160  // 横屏模式宽度
#define ST7735S_HEIGHT          80   // 横屏模式高度
#define ST7735S_WIDTH_PORTRAIT  80   // 竖屏模式宽度
#define ST7735S_HEIGHT_PORTRAIT 160  // 竖屏模式高度

// ST7735S命令定义
#define ST7735S_NOP             0x00
#define ST7735S_SWRESET         0x01
#define ST7735S_RDDID           0x04
#define ST7735S_RDDST           0x09
#define ST7735S_SLPIN           0x10
#define ST7735S_SLPOUT          0x11
#define ST7735S_PTLON           0x12
#define ST7735S_NORON           0x13
#define ST7735S_INVOFF          0x20
#define ST7735S_INVON           0x21
#define ST7735S_DISPOFF         0x28
#define ST7735S_DISPON          0x29
#define ST7735S_CASET           0x2A
#define ST7735S_RASET           0x2B
#define ST7735S_RAMWR           0x2C
#define ST7735S_RAMRD           0x2E
#define ST7735S_PTLAR           0x30
#define ST7735S_COLMOD          0x3A
#define ST7735S_MADCTL          0x36
#define ST7735S_FRMCTR1         0xB1
#define ST7735S_FRMCTR2         0xB2
#define ST7735S_FRMCTR3         0xB3
#define ST7735S_INVCTR          0xB4
#define ST7735S_DISSET5         0xB6
#define ST7735S_PWCTR1          0xC0
#define ST7735S_PWCTR2          0xC1
#define ST7735S_PWCTR3          0xC2
#define ST7735S_PWCTR4          0xC3
#define ST7735S_PWCTR5          0xC4
#define ST7735S_VMCTR1          0xC5
#define ST7735S_RDID1           0xDA
#define ST7735S_RDID2           0xDB
#define ST7735S_RDID3           0xDC
#define ST7735S_RDID4           0xDD
#define ST7735S_PWCTR6          0xFC
#define ST7735S_GMCTRP1         0xE0
#define ST7735S_GMCTRN1         0xE1

// 颜色定义（RGB565格式）
#define ST7735S_BLACK           0x0000
#define ST7735S_BLUE            0x001F
#define ST7735S_RED             0xF800
#define ST7735S_GREEN           0x07E0
#define ST7735S_CYAN            0x07FF
#define ST7735S_MAGENTA         0xF81F
#define ST7735S_YELLOW          0xFFE0
#define ST7735S_WHITE           0xFFFF

// 显示方向
enum ST7735S_Rotation {
    ST7735S_ROTATION_0 = 0,     // 0度
    ST7735S_ROTATION_90 = 1,    // 90度
    ST7735S_ROTATION_180 = 2,   // 180度
    ST7735S_ROTATION_270 = 3    // 270度
};

// 字体大小枚举（已移除，LVGL不需要）

// 使用统一的RGB颜色结构体
using ST7735S_Color = RGB565;


class ST7735S {
public:
    using dma_callback_t = std::function<void(bool success)>;
    
    ST7735S(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin = 255, uint8_t blk_pin = 255);
    ~ST7735S();
    
    // 初始化显示屏
    bool init();
    
    // 释放资源
    void deinit();
    
    // 检查是否已初始化
    bool is_ready() const;
    
    // 检查DMA是否忙碌
    bool is_dma_busy() const;
    
    // 基本控制
    bool reset();
    bool sleep(bool enable);
    bool display_on(bool enable);
    bool invert_display(bool enable);
    
    // 显示方向和区域
    bool set_rotation(ST7735S_Rotation rotation);
    ST7735S_Rotation get_rotation() const;
    bool set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    
    // 基础像素操作（仅保留LVGL需要的功能）
    bool fill_screen(ST7735S_Color color);
    // 像素数据写入（用于LVGL集成）
    bool write_pixel_data(uint16_t color565);
    
    // 设置帧缓冲区（可选）
    bool set_framebuffer(uint16_t* buffer);
    
    // 获取显示尺寸
    uint16_t get_width() const;
    uint16_t get_height() const;
    
    // 背光控制（如果支持）
    bool set_backlight(uint8_t brightness);  // 0-255
    uint8_t get_backlight() const;           // 获取当前背光亮度
    
    // 异步DMA操作 - 非阻塞接口
    bool write_data_buffer_async(const uint8_t* buffer, size_t length, dma_callback_t callback = nullptr);
    bool fill_screen_async(ST7735S_Color color, dma_callback_t callback = nullptr);
    bool draw_bitmap_rgb565_async(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* bitmap, dma_callback_t callback = nullptr);
    
    // 检查DMA队列状态
    size_t get_dma_queue_size() const { return dma_queue_count_; }
    bool has_pending_transfers() const { return dma_queue_count_ > 0; }
    
    // 处理DMA队列（需要在主循环中调用）
    void process_dma_queue();
    
private:
    HAL_SPI* spi_hal_;
    uint8_t cs_pin_;
    uint8_t dc_pin_;    // 数据/命令选择引脚
    uint8_t rst_pin_;   // 复位引脚（可选）
    uint8_t blk_pin_;   // 背光引脚（可选）
    bool initialized_;
    uint8_t current_brightness_;  // 当前背光亮度 (0-255)
    
    // 显示参数
    ST7735S_Rotation rotation_;
    uint16_t width_;
    uint16_t height_;
    
    // 帧缓冲区相关
    uint16_t* framebuffer_;
    bool use_framebuffer_;
    
    // DMA异步相关
    volatile bool dma_busy_;
    dma_callback_t current_callback_;
    
    // 分块传输状态
    // DMA传输队列管理
    
    // DMA传输队列优化
    struct DMATransfer {
        const uint8_t* data;
        size_t length;
        dma_callback_t callback;
    };
    static constexpr size_t MAX_DMA_QUEUE = 4;
    DMATransfer dma_queue_[MAX_DMA_QUEUE];
    size_t dma_queue_head_;
    size_t dma_queue_tail_;
    size_t dma_queue_count_;
    
    // 私有方法
    bool write_command(uint8_t cmd);
    bool write_data(uint8_t data);
    bool write_data_16(uint16_t data);
    bool write_data_buffer(const uint8_t* buffer, size_t length);
    bool spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);
    
    // DMA回调处理
    void on_dma_complete(bool success);
    
    // 初始化相关
    bool init_registers();
    bool configure_display();
};