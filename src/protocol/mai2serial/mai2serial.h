#pragma once
#include <cstdint>
#include <string>
#include "src/hal/uart/hal_uart.h"

// 固定长度命令包格式: { L R c v }  不管分隔符
#define MAI2SERIAL_COMMAND_LENGTH 6
#define MAI2SERIAL_CMD_START_BYTE '{'
#define MAI2SERIAL_CMD_END_BYTE   '}'

// 用于流式处理的内部缓冲区大小：4倍指令长度
#define MAI2SERIAL_STREAM_BUFFER_SIZE (MAI2SERIAL_COMMAND_LENGTH * 4)

// 触摸数据帧
#define MAI2SERIAL_TOUCH_START_BYTE '(' 
#define MAI2SERIAL_TOUCH_END_BYTE   ')'

// 命令类型枚举
enum Mai2Serial_Command {
    MAI2SERIAL_CMD_RSET = 'E',   // Reset
    MAI2SERIAL_CMD_HALT = 'L',   // Halt
    MAI2SERIAL_CMD_STAT = 'A',   // Start/Status
    MAI2SERIAL_CMD_RATIO = 'r',  // Ratio
    MAI2SERIAL_CMD_SENS = 'k'    // Sensitivity
};

// 触摸状态数据结构 - 支持34分区设备
struct Mai2Serial_TouchState {
    union {
        uint64_t raw;  // 原始64位数据，支持34分区
        struct {
            uint32_t state1 : 32;  // 第一部分状态位（分区1-32）
            uint32_t state2 : 3;  // 第二部分状态位（分区33-34及扩展）
        } parts;
    };
    
    Mai2Serial_TouchState() : raw(0) {}
    Mai2Serial_TouchState(uint64_t value) : raw(value) {}
    Mai2Serial_TouchState(uint32_t state1, uint32_t state2) {
        parts.state1 = state1;
        parts.state2 = state2;
    }
};

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

struct Mai2Serial_Config {
    uint32_t baud_rate = 115200;
};

class Mai2Serial {
public:
    enum class Status {
        STOPPED,
        READY,
        RUNNING
    };

    using Mai2Serial_CommandCallback = void (*)(Mai2Serial_Command, const uint8_t*, uint8_t);

    Mai2Serial(HAL_UART* uart_hal);
    ~Mai2Serial();

    bool init();
    void deinit();

    inline bool is_ready() const;

    bool set_config(const Mai2Serial_Config& config);
    Mai2Serial_Config get_config() const;
    void set_command_callback(Mai2Serial_CommandCallback callback);

    bool start();
    bool stop();
    bool reset();

    bool set_baud_rate(uint32_t baud_rate);

    inline void task() {
        process_commands();
    };

    // 数据发送
    void send_touch_data(Mai2Serial_TouchState& touch_data);
    void send_command_response(uint8_t lr, uint8_t sensor, uint8_t cmd, uint8_t value);

    // 状态
    void set_serial_ok(bool ok);
    bool get_serial_ok() const;

    // 设置触发指定区域
    void manually_triggle_area(Mai2_TouchArea area);
    void clear_manually_triggle_area();

    // DMA接收处理（流式）
    void process_commands();
    void process_dma_received_data(const uint8_t* data, size_t length);

private:
    bool send_response(const std::string& response);
    void process_received_byte(const std::string& command_str);
    void parse_command(const std::string& command_str);
    void process_command_packet(const uint8_t* packet, size_t length);
    void calculate_packet_transmission_time();  // 计算数据包传输时间

    HAL_UART* uart_hal_;
    bool initialized_;
    bool serial_ok_;
    Mai2Serial_Config config_;
    Status status_;
    
    // 流控相关成员变量
    uint32_t packet_transmission_time_us_;  // 单个数据包传输时间(微秒)
    uint32_t next_send_time_us_;           // 下次发送时间戳(微秒)

    // 新增：流式接收缓冲区（4倍固定长度，滑动窗口解析）
    uint8_t rx_stream_buffer_[MAI2SERIAL_STREAM_BUFFER_SIZE];
    size_t rx_stream_pos_;

    // 触摸覆盖层 用于绑定或其他情况下手动触发指定区域
    Mai2Serial_TouchState triggle_touch_data_;

    // 回调
    Mai2Serial_CommandCallback command_callback_ = nullptr;
};