#pragma once

#include "../../hal/uart/hal_uart.h"
#include <stdint.h>
#include <string>
#include <functional>
#include <cstring>

/**
 * 协议模块 - Mai2Serial
 * 实现maimai街机的串口通信协议
 * 用于发送触摸数据和接收配置命令
 */

// Mai2Serial协议常量
#define MAI2SERIAL_COMMAND_LENGTH   8
#define MAI2SERIAL_TOUCH_POINTS     34
#define MAI2SERIAL_DEFAULT_BAUD     115200

// Mai2 区域映射枚举 - 对应maimai街机的34个触摸区域
// 参考设计文档中的区域定义：A1-A8, B1-B8, C1-C2, D1-D8, E1-E8
enum Mai2_TouchArea {
    // A区 (外环) 1-8
    MAI2_AREA_A1 = 1, MAI2_AREA_A2, MAI2_AREA_A3, MAI2_AREA_A4,
    MAI2_AREA_A5, MAI2_AREA_A6, MAI2_AREA_A7, MAI2_AREA_A8,
    
    // B区 (内环) 9-16  
    MAI2_AREA_B1, MAI2_AREA_B2, MAI2_AREA_B3, MAI2_AREA_B4,
    MAI2_AREA_B5, MAI2_AREA_B6, MAI2_AREA_B7, MAI2_AREA_B8,
    
    // C区 (中心) 17-18
    MAI2_AREA_C1, MAI2_AREA_C2,
    
    // D区 (外环扩展) 19-26
    MAI2_AREA_D1, MAI2_AREA_D2, MAI2_AREA_D3, MAI2_AREA_D4,
    MAI2_AREA_D5, MAI2_AREA_D6, MAI2_AREA_D7, MAI2_AREA_D8,
    
    // E区 (内环扩展) 27-34
    MAI2_AREA_E1, MAI2_AREA_E2, MAI2_AREA_E3, MAI2_AREA_E4,
    MAI2_AREA_E5, MAI2_AREA_E6, MAI2_AREA_E7, MAI2_AREA_E8,
    MAI2_NO_USED
};

// 区域名称映射表 - 用于调试和显示
__attribute__((unused)) static const char* mai2_area_names[35] = {
    "NONE",
    "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8",
    "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", 
    "C1", "C2",
    "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
    "E1", "E2", "E3", "E4", "E5", "E6", "E7", "E8"
};

// Mai2Serial 指令类型 - 严格按照maimai协议
enum Mai2Serial_Command {
    MAI2SERIAL_CMD_RSET = 0x45,  // E - 重置指令
    MAI2SERIAL_CMD_HALT = 0x4C,  // L - 停止指令
    MAI2SERIAL_CMD_STAT = 0x41,  // A - 状态指令
    MAI2SERIAL_CMD_RATIO = 0x72, // r - 比例指令
    MAI2SERIAL_CMD_SENS = 0x6B   // k - 灵敏度指令
};

// 指令包格式常量
#define MAI2SERIAL_CMD_START_BYTE   '{'
#define MAI2SERIAL_CMD_END_BYTE     '}'
#define MAI2SERIAL_TOUCH_START_BYTE '('
#define MAI2SERIAL_TOUCH_END_BYTE   ')'

// 波特率选项
enum Mai2Serial_BaudRate {
    MAI2_BAUD_9600 = 0,
    MAI2_BAUD_115200 = 1,
    MAI2_BAUD_250000 = 2,
    MAI2_BAUD_500000 = 3,
    MAI2_BAUD_1000000 = 4,
    MAI2_BAUD_1500000 = 5,
    MAI2_BAUD_2000000 = 6
};

// 命令枚举
enum class Command {
    UNKNOWN,
    START,
    STOP,
    RESET,
    SET_BAUD_RATE
};

// 触摸点数据结构
// Mai2Serial_TouchPoint结构体已删除 - 不再需要坐标和压力数据

// 36位触摸状态数据类型
union Mai2Serial_TouchState {
    struct {
        uint32_t state1;  // 位 0-31
        uint8_t state2;   // 位 32-35 (只使用低4位)
    } parts;
    struct {
        uint64_t full : 36;  // 完整的36位状态
    } bitmap;
    
    Mai2Serial_TouchState() { parts.state1 = 0; parts.state2 = 0; }
    Mai2Serial_TouchState(uint32_t s1, uint8_t s2) { parts.state1 = s1; parts.state2 = s2 & 0x0F; }
};

// 简化的触摸数据结构 - 只包含36位状态bitmap
struct Mai2Serial_TouchData {
    Mai2Serial_TouchState touch_state;
    bool valid;
    
    Mai2Serial_TouchData() : valid(false) {}
    Mai2Serial_TouchData(uint32_t state1, uint8_t state2) : touch_state(state1, state2), valid(true) {}
};

// 简化的配置数据结构 - 移除不必要的压力和灵敏度数组
struct Mai2Serial_Config {
    uint32_t baud_rate;
    uint8_t sample_time;                            // 采样时间 0-7
    uint16_t sample_time_ms;                        // 采样时间(毫秒)
    
    Mai2Serial_Config() {
        baud_rate = MAI2SERIAL_DEFAULT_BAUD;
        sample_time = 3;
        sample_time_ms = 10;
    }
};

// 命令回调函数类型
typedef std::function<void(const char* cmd, uint8_t* params, uint8_t param_count)> Mai2Serial_CommandCallback;
typedef std::function<void(const std::string& message)> Mai2Serial_LogCallback;
typedef std::function<void(const Mai2Serial_TouchData& touch_data)> Mai2Serial_TouchCallback;

// 状态枚举
enum class Status {
    STOPPED,
    READY,
    RUNNING,
    ERROR
};

class Mai2Serial {
public:
    Mai2Serial(HAL_UART* uart_hal);
    ~Mai2Serial();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 配置管理
    bool set_config(const Mai2Serial_Config& config);
    Mai2Serial_Config get_config() const;
    bool save_config();
    bool load_config();
    
    // 触摸数据发送
    bool send_touch_data(const Mai2Serial_TouchData& touch_data);
    bool send_touch_state(uint32_t state1, uint32_t state2);
    
    // 指令处理
    void process_command_packet(const uint8_t* packet, size_t length);
    void send_command_response(uint8_t lr, uint8_t sensor, uint8_t cmd, uint8_t value);
    
    // DMA接收处理
    void start_dma_receive();
    void process_dma_received_data(const uint8_t* data, size_t length);
    
    // 串口命令处理
    void process_commands();
    bool send_response(const std::string& response);
    
    // 回调设置
    void set_command_callback(Mai2Serial_CommandCallback callback);
    void set_log_callback(Mai2Serial_LogCallback callback);
    void set_touch_callback(Mai2Serial_TouchCallback callback);
    
    // 状态控制
    void set_serial_ok(bool ok);
    bool get_serial_ok() const;
    void start_sampling();
    void stop_sampling();
    bool start();
    bool stop();
    bool reset();
    
    // 配置设置
    bool set_baud_rate(Mai2Serial_BaudRate baud);
    bool set_baud_rate(uint32_t baud_rate);
    bool set_sample_time(uint8_t time);                     // time: 0-7
    bool set_sample_time(uint16_t sample_time_ms);
    
    // 信息显示
    void show_all_bind_points();
    void show_help();
    void show_bind_info();
    void show_status_info();
    
    // 任务处理
    void task();
    
private:
    HAL_UART* uart_hal_;
    bool initialized_;
    bool serial_ok_;
    
    // 配置数据
    Mai2Serial_Config config_;
    
    // 状态管理
    Status status_;
    
    // 串口发送控制
    Mai2Serial_TouchData last_touch_data_;
    
    // 命令处理
    char command_buffer_[MAI2SERIAL_COMMAND_LENGTH];
    uint8_t command_buffer_pos_;
    uint8_t command_index_;
    bool in_command_;
    
    // 回调函数
    Mai2Serial_CommandCallback command_callback_;
    Mai2Serial_LogCallback log_callback_;
    Mai2Serial_TouchCallback touch_callback_;
    
    // 内部方法
    void process_received_data();
    void parse_command(const std::string& command);
    void process_received_byte(uint8_t byte);
    void process_single_command();
    void handle_command(uint8_t* packet);
    
    // 命令处理函数
    void handle_reset_command();
    void handle_halt_command();
    void handle_ratio_command(uint8_t* packet);
    void handle_stat_command();
    void handle_uart_baud_command(uint8_t* packet);

    void handle_start_command();
    void handle_reset_touch_command();
    void handle_save_config_command();
    void handle_show_bind_command();
    
    // 工具函数
    uint32_t get_baud_rate_value(Mai2Serial_BaudRate baud);
    Mai2Serial_BaudRate get_baud_rate_enum(uint32_t baud);
    int parse_point_number(char c1, char c2);
    int parse_value(char c1, char c2 = '\0');
    void clear_command_buffer();
};