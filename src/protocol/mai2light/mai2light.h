#pragma once

#include "../../hal/uart/hal_uart.h"
#include <stdint.h>
#include <vector>
#include <functional>
#include <cstring>

/**
 * 协议层 - Mai2Light LED控制器
 * 基于UART通信的LED控制协议
 * 支持11个LED的颜色控制和EEPROM配置
 */

// Mai2Light协议常量
#define MAI2LIGHT_NUM_LEDS          11      // LED数量
#define MAI2LIGHT_SYNC_BYTE         0xE0    // 同步字节
#define MAI2LIGHT_MARKER_BYTE       0xD0    // 标记字节
#define MAI2LIGHT_MAX_PACKET_SIZE   64      // 最大数据包大小
#define MAI2LIGHT_DEFAULT_BAUD_RATE 115200  // 默认波特率

// Mai2Light命令定义
enum class Mai2Light_Command : uint8_t {
    SET_LED_GS_8BIT         = 0x01,     // 设置单个LED 8位灰度
    SET_LED_GS_8BIT_MULTI   = 0x02,     // 设置多个LED 8位灰度
    SET_LED_RGB             = 0x03,     // 设置单个LED RGB
    SET_LED_RGB_MULTI       = 0x04,     // 设置多个LED RGB
    SET_ALL_LEDS            = 0x05,     // 设置所有LED
    SET_BRIGHTNESS          = 0x06,     // 设置亮度
    SET_FADE_TIME           = 0x07,     // 设置渐变时间
    GET_LED_STATUS          = 0x10,     // 获取LED状态
    GET_BOARD_INFO          = 0x11,     // 获取板卡信息
    GET_PROTOCOL_VERSION    = 0x12,     // 获取协议版本
    SET_EEPROM              = 0x20,     // 设置EEPROM
    GET_EEPROM              = 0x21,     // 获取EEPROM
    SAVE_TO_EEPROM          = 0x22,     // 保存到EEPROM
    LOAD_FROM_EEPROM        = 0x23,     // 从EEPROM加载
    RESET_BOARD             = 0x30,     // 重置板卡
    ENTER_BOOTLOADER        = 0x31,     // 进入引导程序
    UNKNOWN                 = 0xFF      // 未知命令
};

// 应答状态定义
enum class Mai2Light_AckStatus : uint8_t {
    OK                      = 0x00,     // 成功
    SUM_ERROR              = 0x01,     // 校验和错误
    INVALID_COMMAND        = 0x02,     // 无效命令
    INVALID_PARAMETER      = 0x03,     // 无效参数
    EEPROM_ERROR           = 0x04,     // EEPROM错误
    HARDWARE_ERROR         = 0x05      // 硬件错误
};

// 应答报告定义
enum class Mai2Light_AckReport : uint8_t {
    OK                      = 0x00,     // 正常
    WARNING                = 0x01,     // 警告
    ERROR                  = 0x02      // 错误
};

// RGB颜色结构
struct Mai2Light_RGB {
    uint8_t r;              // 红色分量 (0-255)
    uint8_t g;              // 绿色分量 (0-255)
    uint8_t b;              // 蓝色分量 (0-255)
    
    Mai2Light_RGB() : r(0), g(0), b(0) {}
    Mai2Light_RGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    
    // 从HSV转换
    static Mai2Light_RGB from_hsv(uint16_t hue, uint8_t saturation, uint8_t value);
    
    // 颜色混合
    Mai2Light_RGB blend(const Mai2Light_RGB& other, uint8_t ratio) const;
};

// LED状态结构
struct Mai2Light_LEDStatus {
    Mai2Light_RGB color;    // 当前颜色
    uint8_t brightness;     // 亮度 (0-255)
    bool enabled;           // 是否启用
    
    Mai2Light_LEDStatus() : brightness(255), enabled(true) {}
};

// 板卡信息结构
struct Mai2Light_BoardInfo {
    uint16_t board_id;      // 板卡ID
    uint8_t hardware_version; // 硬件版本
    uint8_t firmware_version; // 固件版本
    uint16_t led_count;     // LED数量
    uint32_t serial_number; // 序列号
    
    Mai2Light_BoardInfo() : board_id(0), hardware_version(0), firmware_version(0), 
                           led_count(MAI2LIGHT_NUM_LEDS), serial_number(0) {}
};

// 请求数据包结构
struct Mai2Light_PacketReq {
    uint8_t sync;           // 同步字节 (0xE0)
    uint8_t node_id;        // 节点ID
    uint8_t length;         // 数据长度
    Mai2Light_Command command; // 命令
    uint8_t data[32];       // 数据
    uint8_t checksum;       // 校验和
    
    Mai2Light_PacketReq() : sync(MAI2LIGHT_SYNC_BYTE), node_id(0), length(0), 
                           command(Mai2Light_Command::UNKNOWN), checksum(0) {
        memset(data, 0, sizeof(data));
    }
};

// 应答数据包结构
struct Mai2Light_PacketAck {
    uint8_t sync;           // 同步字节 (0xE0)
    uint8_t node_id;        // 节点ID
    uint8_t length;         // 数据长度
    Mai2Light_Command command; // 命令
    Mai2Light_AckStatus status; // 状态
    Mai2Light_AckReport report; // 报告
    uint8_t data[32];       // 数据
    uint8_t checksum;       // 校验和
    
    Mai2Light_PacketAck() : sync(MAI2LIGHT_SYNC_BYTE), node_id(0), length(0), 
                           command(Mai2Light_Command::UNKNOWN), 
                           status(Mai2Light_AckStatus::OK), 
                           report(Mai2Light_AckReport::OK), checksum(0) {
        memset(data, 0, sizeof(data));
    }
};

// 配置结构
struct Mai2Light_Config {
    uint32_t baud_rate;     // 波特率
    uint8_t node_id;        // 节点ID
    uint8_t global_brightness; // 全局亮度 (0-255)
    uint16_t fade_time_ms;  // 渐变时间 (毫秒)
    bool auto_save;         // 自动保存到EEPROM
    bool enable_fade;       // 启用渐变效果
    
    Mai2Light_Config() {
        baud_rate = MAI2LIGHT_DEFAULT_BAUD_RATE;
        node_id = 0;
        global_brightness = 255;
        fade_time_ms = 100;
        auto_save = false;
        enable_fade = true;
    }
};

// 回调函数类型
typedef std::function<void(Mai2Light_Command command, const uint8_t* data, uint8_t length)> Mai2Light_CommandCallback;
typedef std::function<void(const std::string& message)> Mai2Light_LogCallback;

class Mai2Light {
public:
    Mai2Light(HAL_UART* uart_hal, uint8_t node_id = 0);
    ~Mai2Light();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 配置管理
    bool set_config(const Mai2Light_Config& config);
    bool get_config(Mai2Light_Config& config);
    
    // LED控制
    bool set_led_color(uint8_t led_index, const Mai2Light_RGB& color);          // 设置单个LED颜色
    bool set_led_color(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);     // 设置单个LED颜色
    bool set_led_brightness(uint8_t led_index, uint8_t brightness);             // 设置单个LED亮度
    bool set_all_leds(const Mai2Light_RGB& color);                             // 设置所有LED颜色
    
    // 全局控制
    bool set_global_brightness(uint8_t brightness);                            // 设置全局亮度
    bool set_fade_time(uint16_t fade_time_ms);                                 // 设置渐变时间
    bool clear_all_leds();                                                     // 清除所有LED
    
    // 状态查询
    bool get_led_status(uint8_t led_index, Mai2Light_LEDStatus& status);       // 获取LED状态
    bool get_all_led_status(Mai2Light_LEDStatus status_array[MAI2LIGHT_NUM_LEDS]); // 获取所有LED状态
    const Mai2Light_LEDStatus* get_led_status_array() const;                   // 获取LED状态数组指针
    bool get_board_info(Mai2Light_BoardInfo& info);                           // 获取板卡信息
    uint8_t get_protocol_version();                                            // 获取协议版本
    
    // EEPROM操作
    bool save_to_eeprom();                                                     // 保存当前状态到EEPROM
    bool load_from_eeprom();                                                   // 从EEPROM加载状态
    bool set_eeprom_data(uint16_t address, const uint8_t* data, uint8_t length); // 设置EEPROM数据
    bool get_eeprom_data(uint16_t address, uint8_t* data, uint8_t length);     // 获取EEPROM数据
    
    // 系统控制
    bool reset_board();                                                        // 重置板卡
    bool enter_bootloader();                                                   // 进入引导程序
    
    // 回调设置
    void set_command_callback(Mai2Light_CommandCallback callback);
    void set_log_callback(Mai2Light_LogCallback callback);
    
    // 任务处理
    void task();
    
private:
    HAL_UART* uart_hal_;
    bool initialized_;
    
    // 配置和状态
    Mai2Light_Config config_;
    Mai2Light_LEDStatus led_status_[MAI2LIGHT_NUM_LEDS];
    Mai2Light_BoardInfo board_info_;
    
    // 接收缓冲区
    uint8_t rx_buffer_[MAI2LIGHT_MAX_PACKET_SIZE];
    uint8_t rx_buffer_pos_;
    
    // 回调函数
    Mai2Light_CommandCallback command_callback_;
    Mai2Light_LogCallback log_callback_;
    
    // 虚拟EEPROM存储
    static const uint16_t EEPROM_SIZE = 256;  // EEPROM大小
    uint8_t virtual_eeprom_[EEPROM_SIZE];     // 虚拟EEPROM数据
    
    // 内部方法
    bool send_packet(const Mai2Light_PacketReq& packet);
    bool receive_packet(Mai2Light_PacketAck& packet, uint32_t timeout_ms = 1000);
    bool send_command(Mai2Light_Command command, const uint8_t* data = nullptr, uint8_t data_length = 0);
    
    // 数据包处理
    void process_received_data();
    void process_packet(const Mai2Light_PacketReq& packet);
    bool parse_packet(const uint8_t* buffer, uint8_t length, Mai2Light_PacketReq& packet);
    
    // 校验和计算
    uint8_t calculate_checksum(const uint8_t* data, uint8_t length);
    bool verify_checksum(const uint8_t* data, uint8_t length, uint8_t expected_checksum);
    
    // 命令处理
    void handle_set_led_command(const Mai2Light_PacketReq& packet);
    void handle_get_status_command(const Mai2Light_PacketReq& packet);
    void handle_eeprom_command(const Mai2Light_PacketReq& packet);
    void handle_system_command(const Mai2Light_PacketReq& packet);
    
    // 应答发送
    void send_ack(Mai2Light_Command command, Mai2Light_AckStatus status, 
                  Mai2Light_AckReport report = Mai2Light_AckReport::OK, 
                  const uint8_t* data = nullptr, uint8_t data_length = 0);
    
    // 日志输出
    void log_message(const std::string& message);
    
    // 渐变效果
    void update_fade_effects();
    bool is_fading_;
    uint32_t fade_start_time_;
    Mai2Light_RGB fade_start_colors_[MAI2LIGHT_NUM_LEDS];
    Mai2Light_RGB fade_target_colors_[MAI2LIGHT_NUM_LEDS];
};