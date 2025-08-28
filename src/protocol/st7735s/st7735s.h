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

// ST7735S显示参数
#define ST7735S_WIDTH           160  // 横屏模式宽度
#define ST7735S_HEIGHT          80   // 横屏模式高度

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
    ST7735S_ROTATION_180 = 1,   // 180度
    ST7735S_ROTATION_90 = 2,    // 90度
    ST7735S_ROTATION_270 = 3    // 270度
};

// 字体大小枚举（已移除，LVGL不需要）

// 使用统一的RGB颜色结构体
using ST7735S_Color = RGB565;


class ST7735S {
public:
    using dma_callback_t = std::function<void(bool success)>;
    
    ST7735S(HAL_SPI* spi_hal, ST7735S_Rotation rotation, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin = 255, uint8_t blk_pin = 255);
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
    
    ST7735S_Rotation get_rotation() const;
    
    
    // 直接写入接口 - 传入指针和大小的立即刷写方式
    bool write_buffer(const uint16_t* buffer, size_t buffer_size, dma_callback_t callback = nullptr);
    bool write_buffer_region(const uint16_t* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height, dma_callback_t callback = nullptr);
    
    // 获取显示尺寸
    uint16_t get_width() const;
    uint16_t get_height() const;
    
    // 背光控制（如果支持）
    bool set_backlight(uint8_t brightness);  // 0-255
    uint8_t get_backlight() const;           // 获取当前背光亮度
    
    // DMA处理
    void process_dma();  // 处理DMA完成事件
    
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
    
    // DMA传输状态
    volatile bool dma_busy_;
    dma_callback_t current_callback_;
    
    inline void select_cmd_mode();

    inline void select_data_mode();

    inline void deselect();

    // 私有方法
    bool write_command(uint8_t cmd);
    bool write_data(uint8_t data);
    bool write_data_16(uint16_t data);
    bool write_data_buffer(const uint8_t* buffer, size_t length);
    bool spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);
    bool set_rotation(ST7735S_Rotation rotation);
    bool set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    // DMA回调处理
    void on_dma_complete(bool success);
    
    // 显示初始化
    bool init_registers();
};