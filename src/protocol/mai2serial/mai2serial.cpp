#include "mai2serial.h"
#include "pico/time.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

// Mai2Serial构造函数
Mai2Serial::Mai2Serial(HAL_UART* uart_hal) 
    : uart_hal_(uart_hal)
    , initialized_(false)
    , serial_ok_(false)
    , config_()
    , status_(Status::STOPPED)
    , command_buffer_pos_(0) {
    
    for (int i = 0; i < MAI2SERIAL_COMMAND_LENGTH; i++) {
        command_buffer_[i] = 0;
    }

    // 新增：初始化流式接收缓冲区
    rx_stream_pos_ = 0;
    std::memset(rx_stream_buffer_, 0, sizeof(rx_stream_buffer_));
}

// Mai2Serial析构函数
Mai2Serial::~Mai2Serial() {
    deinit();
}

// 初始化
bool Mai2Serial::init() {
    if (!uart_hal_ || initialized_) {
        return false;
    }
    
    initialized_ = true;
    status_ = Status::READY;
    
    return true;
}

// 释放资源
void Mai2Serial::deinit() {
    if (initialized_) {
        uart_hal_->deinit();
        initialized_ = false;
        status_ = Status::STOPPED;
        
        // 清理回调
        command_callback_ = nullptr;
    }
}

// 检查是否就绪
bool Mai2Serial::is_ready() const {
    return status_ > Status::STOPPED;
}

// 设置配置
bool Mai2Serial::set_config(const Mai2Serial_Config& config) {
    config_ = config;
    
    if (initialized_) {
        // 重新配置UART波特率
        return set_baud_rate(config.baud_rate);
    }
    
    return true;
}

// 获取配置
Mai2Serial_Config Mai2Serial::get_config() const {
    return config_;
}

// 发送触摸数据
bool Mai2Serial::send_touch_data(Mai2Serial_TouchState& touch_data) {
    if (!is_ready() || !serial_ok_) {
        return false;
    }
    
    touch_data.raw |= triggle_touch_data_.raw;
    
    // 按照标准实现：预先组装完整数据包，避免循环和多次write调用
    static touch_data_packet _packet;
    _packet.parts.state1 = touch_data.parts.state1;
    _packet.parts.state2 = touch_data.parts.state2;


    // 使用新的DMA接口：写入TX缓冲区会自动处理DMA传输
    size_t bytes_written = uart_hal_->write_to_tx_buffer(_packet.data, 9);
    bool result = (bytes_written == 9);
    return result;
}

// 处理命令 - 使用DMA接收
void Mai2Serial::process_commands() {
    if (!is_ready()) {
        return;
    }
    static uint8_t size;
    static uint8_t buffer[MAI2SERIAL_COMMAND_LENGTH * 4];  // 支持多个指令包
    size = uart_hal_->read_from_rx_buffer(buffer, sizeof(buffer));
    // 检查接收缓冲区 - 使用新的DMA接口
    
    if (size) {
        // 处理接收到的数据
        process_dma_received_data(buffer, size);
    }
}

// 发送响应
bool Mai2Serial::send_response(const std::string& response) {
    if (!is_ready()) {
        return false;
    }
    
    std::string full_response = response + "\r\n";
    // 使用新的DMA接口：写入TX缓冲区会自动处理DMA传输
    size_t bytes_written = uart_hal_->write_to_tx_buffer((uint8_t*)full_response.c_str(), full_response.length());
    return (bytes_written == full_response.length());
}

void Mai2Serial::set_command_callback(Mai2Serial_CommandCallback callback) {
    command_callback_ = callback;
}

// 控制状态
bool Mai2Serial::start() {
    if (!is_ready()) {
        return false;
    }
    
    status_ = Status::RUNNING;
    
    send_response("OK");
    return true;
}

bool Mai2Serial::stop() {
    if (!initialized_) {
        return false;
    }
    
    status_ = Status::READY;
    send_response("STOPPED");
    return true;
}

bool Mai2Serial::reset() {
    if (!initialized_) {
        return false;
    }
    
    // 重置配置为默认值
    config_ = Mai2Serial_Config();
    status_ = Status::READY;
    
    send_response("RESET OK");
    return true;
}

// 设置参数
bool Mai2Serial::set_baud_rate(uint32_t baud_rate) {
    if (!initialized_) {
        return false;
    }
    const uint32_t valid_baud_rates[] = {9600, 115200, 250000, 500000, 1000000, 1500000, 2000000};
    bool valid_baud = false;
    for (size_t i = 0; i < sizeof(valid_baud_rates)/sizeof(valid_baud_rates[0]); i++) {
        if (valid_baud_rates[i] == baud_rate) {
            valid_baud = true;
            break;
        }
    }
    if (!valid_baud) {
        return false;
    }
    config_.baud_rate = baud_rate;
    uart_hal_->set_baudrate(baud_rate);
    return true;
}

// 处理指令包
void Mai2Serial::process_command_packet(const uint8_t* packet, size_t length) {
    if (length < 5 || packet[0] != MAI2SERIAL_CMD_START_BYTE) {
        return;  // 无效包格式
    }
    
    // 查找结束字节
    bool found_end = false;
    for (size_t i = 1; i < length; i++) {
        if (packet[i] == MAI2SERIAL_CMD_END_BYTE) {
            found_end = true;
            break;
        }
    }
    
    if (!found_end || length < 5) {
        return;  // 包不完整
    }
    
    uint8_t lr = packet[1];      // L/R 标识
    uint8_t sensor = packet[2];  // 传感器编号
    uint8_t cmd = packet[3];     // 指令类型
    uint8_t value = packet[4];   // 参数值
    
    switch (cmd) {
        case MAI2SERIAL_CMD_RSET:  // E - 重置
            // 重置传感器和系统状态
            reset();
            serial_ok_ = false;  // 重置时停止发送触摸数据
            // 通知命令回调
            if (command_callback_) {
                command_callback_(MAI2SERIAL_CMD_RSET, nullptr, 0);
            }
            break;
            
        case MAI2SERIAL_CMD_HALT:  // L - 停止
            // 停止操作
            stop();
            serial_ok_ = false;  // 进入设置模式，停止发送触摸数据
            // 通知命令回调
            if (command_callback_) {
                command_callback_(MAI2SERIAL_CMD_HALT, nullptr, 0);
            }
            break;
            
        case MAI2SERIAL_CMD_RATIO: // r - 比例设置
            // 发送比例响应，格式: (LRr值)
            send_command_response(lr, sensor, 'r', value);
            // 通知命令回调
            if (command_callback_) {
                uint8_t params[2] = {sensor, value};
                command_callback_(MAI2SERIAL_CMD_RATIO, params, 2);
            }
            break;
            
        case MAI2SERIAL_CMD_SENS:  // k - 灵敏度设置
            // 发送灵敏度响应，格式: (Rsk值)，注意lr固定为'R'
            send_command_response('R', sensor, 'k', value);
            // 通知命令回调
            if (command_callback_) {
                uint8_t params[2] = {sensor, value};
                command_callback_(MAI2SERIAL_CMD_SENS, params, 2);
            }
            break;
            
        case MAI2SERIAL_CMD_STAT:  // A - 状态
            // 启动采样状态
            start();
            serial_ok_ = true;   // 开始发送触摸数据
            // 通知命令回调
            if (command_callback_) {
                command_callback_(MAI2SERIAL_CMD_STAT, nullptr, 0);
            }
            break;
            
        default:
            // 未知指令，忽略
            break;
    }
}

// 发送指令响应
void Mai2Serial::send_command_response(uint8_t lr, uint8_t sensor, uint8_t cmd, uint8_t value) {
    if (!is_ready()) {
        return;
    }
    
    // 响应格式: '(' + lr + sensor + cmd + value + ')'
    uint8_t response[6];
    response[0] = MAI2SERIAL_TOUCH_START_BYTE;  // '('
    response[1] = lr;
    response[2] = sensor;
    response[3] = cmd;
    response[4] = value;
    response[5] = MAI2SERIAL_TOUCH_END_BYTE;    // ')'
    
    // 使用新的DMA接口：写入TX缓冲区会自动处理DMA传输
    (void)uart_hal_->write_to_tx_buffer(response, sizeof(response));
}

// 处理DMA接收的数据（流式解析，逐字节滑动窗口）
void Mai2Serial::process_dma_received_data(const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return;
    }

    // 先将所有数据追加到内部流式缓冲区，若溢出则丢弃最旧的1字节以腾出空间
    for (size_t i = 0; i < length; ++i) {
        if (rx_stream_pos_ >= MAI2SERIAL_STREAM_BUFFER_SIZE) {
            // 丢弃最前面1字节，整体左移
            std::memmove(rx_stream_buffer_, rx_stream_buffer_ + 1, MAI2SERIAL_STREAM_BUFFER_SIZE - 1);
            rx_stream_pos_ = MAI2SERIAL_STREAM_BUFFER_SIZE - 1;
        }
        rx_stream_buffer_[rx_stream_pos_++] = data[i];
    }

    // 滑动窗口按字节解析
    while (rx_stream_pos_ > 0) {
        // 若遇到'{'则等待凑齐固定长度再处理
        if (rx_stream_buffer_[0] == (uint8_t)MAI2SERIAL_CMD_START_BYTE) {
            if (rx_stream_pos_ < MAI2SERIAL_COMMAND_LENGTH) {
                // 数据不足，等待下次DMA补齐
                break;
            }
            // 检查前8字节内是否包含结束符'}'，若没有则认为还未凑齐（避免误消费）
            bool has_end = false;
            for (int i = 1; i < MAI2SERIAL_COMMAND_LENGTH; ++i) {
                if (rx_stream_buffer_[i] == (uint8_t)MAI2SERIAL_CMD_END_BYTE) { has_end = true; break; }
            }
            if (!has_end) {
                // 仍未完整，等待更多字节
                break;
            }
            // 处理完整的固定长度指令
            process_command_packet(rx_stream_buffer_, MAI2SERIAL_COMMAND_LENGTH);
            // 丢弃整个指令长度，继续解析后续数据
            std::memmove(rx_stream_buffer_, rx_stream_buffer_ + MAI2SERIAL_COMMAND_LENGTH,
                         rx_stream_pos_ - MAI2SERIAL_COMMAND_LENGTH);
            rx_stream_pos_ -= MAI2SERIAL_COMMAND_LENGTH;
            continue;
        }

        // 非'{'开头的字节作为字符串命令流式传入
        process_received_byte(rx_stream_buffer_[0]);
        // 丢弃最前面1字节
        std::memmove(rx_stream_buffer_, rx_stream_buffer_ + 1, rx_stream_pos_ - 1);
        rx_stream_pos_ -= 1;
    }
}

void Mai2Serial::process_received_byte(uint8_t byte) {
    // 处理接收到的字节
    if (byte == '\r' || byte == '\n') {
        if (command_buffer_pos_ > 0) {
            command_buffer_[command_buffer_pos_] = '\0';
            parse_command(std::string(command_buffer_));
            command_buffer_pos_ = 0;
        }
    } else if (command_buffer_pos_ < sizeof(command_buffer_) - 1) {
        command_buffer_[command_buffer_pos_++] = byte;
    } else {
        // 缓冲区溢出，重置
        command_buffer_pos_ = 0;
    }
}

void Mai2Serial::parse_command(const std::string& command_str) {
    if (command_str.empty()) {
        return;
    }
    
    char cmd = command_str[0];
    std::string param = command_str.length() > 1 ? command_str.substr(1) : "";
    
    switch (cmd) {
        case '/': // help指令
            if (param == "help") {
                send_response("Mai2Serial Module Commands:\n"
                    "1=L/R\n"
                    "2=sensor\n"
                    "3=cmd\n"
                    "4=value\n"
                    "{  E } - Reset\n"
                    "{  L } - Halt\n"
                    "{  A } - Start\n"
                    "{  r } - Ratio\n"
                    "{  k } - Sensitivity\n"
                    "/help - Show this help");
            }
            break;
            
        default:
            // 忽略所有其他命令
            break;
    }
    
    // 通知命令回调
    if (command_callback_) {
        Mai2Serial_Command cmd_enum = MAI2SERIAL_CMD_STAT; // 默认值
        switch (cmd) {
            case 'E': cmd_enum = MAI2SERIAL_CMD_RSET; break;
            case 'L': cmd_enum = MAI2SERIAL_CMD_HALT; break;
            case 'A': cmd_enum = MAI2SERIAL_CMD_STAT; break;
            case 'r': cmd_enum = MAI2SERIAL_CMD_RATIO; break;
            case 'k': cmd_enum = MAI2SERIAL_CMD_SENS; break;
        }
        uint8_t* param_data = param.empty() ? nullptr : (uint8_t*)param.c_str();
        uint8_t param_count = param.empty() ? 0 : param.length();
        command_callback_(cmd_enum, param_data, param_count);
    }
}

// 设置串口发送状态
void Mai2Serial::set_serial_ok(bool ok) {
    serial_ok_ = ok;
}

// 获取串口发送状态
bool Mai2Serial::get_serial_ok() const {
    return serial_ok_;
}

void Mai2Serial::manually_triggle_area(Mai2_TouchArea area) {
    triggle_touch_data_.parts.state1 = 0;
    triggle_touch_data_.parts.state2 = 0;
    if (area >= 1 && area <= 34) {
        uint8_t bit_index = area - 1;
        if (bit_index < 32) {
            triggle_touch_data_.parts.state1 |= (1UL << bit_index);
        } else {
            bit_index -= 32;
            triggle_touch_data_.parts.state2 |= (1 << bit_index);
        }
    }
}

void Mai2Serial::clear_manually_triggle_area() {
    triggle_touch_data_.parts.state1 = 0;
    triggle_touch_data_.parts.state2 = 0;
}
