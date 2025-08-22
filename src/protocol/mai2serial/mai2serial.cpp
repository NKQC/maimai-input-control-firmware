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
    last_touch_data_ = Mai2Serial_TouchData();
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
    
    // 配置UART - 使用HAL_UART的正确接口
    if (!uart_hal_->init(0, 1, config_.baud_rate, false)) {
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
        touch_callback_ = nullptr;
        command_callback_ = nullptr;
    }
}

// 检查是否就绪
bool Mai2Serial::is_ready() const {
    return initialized_ && (status_ == Status::READY || status_ == Status::RUNNING);
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

// 保存配置
bool Mai2Serial::save_config() {
    // 这里应该保存到EEPROM或Flash
    // 暂时返回true，实际实现需要根据硬件平台
    return true;
}

// 加载配置
bool Mai2Serial::load_config() {
    // 这里应该从EEPROM或Flash加载
    // 暂时返回true，使用默认配置
    return true;
}

// 发送触摸数据
bool Mai2Serial::send_touch_data(const Mai2Serial_TouchData& touch_data) {
    if (!is_ready()) {
        return false;
    }
    
    // 检查串口发送状态，只有在收到STAT命令后才发送触摸数据
    if (!serial_ok_) {
        return false;
    }
    
    uint32_t send1 = touch_data.touch_state.parts.state1;
    uint32_t send2 = (uint32_t)touch_data.touch_state.parts.state2;
    
    // 按照标准实现：预先组装完整数据包，避免循环和多次write调用
    uint8_t packet[9] = {
        '(',
        (uint8_t)(send1 & 0b11111),
        (uint8_t)((send1 >> 5) & 0b11111),
        (uint8_t)((send1 >> 10) & 0b11111),
        (uint8_t)((send1 >> 15) & 0b11111),
        (uint8_t)((send1 >> 20) & 0b11111),
        (uint8_t)(send2 & 0b11111),
        (uint8_t)((send2 >> 5) & 0b11111),
        ')'
    };
    
    // 使用新的DMA接口：先写入TX缓冲区，然后触发DMA传输
    size_t bytes_written = uart_hal_->write_to_tx_buffer(packet, 9);
    if (bytes_written > 0) {
        uart_hal_->trigger_tx_dma();
    }
    bool result = (bytes_written == 9);
    if (result) {
        last_touch_data_ = touch_data;
    }
    return result;
}

// 处理命令 - 使用DMA接收
void Mai2Serial::process_commands() {
    if (!is_ready()) {
        return;
    }
    
    // 检查接收缓冲区 - 使用新的DMA接口
    uint8_t buffer[MAI2SERIAL_COMMAND_LENGTH * 4];  // 支持多个指令包
    size_t bytes_received = uart_hal_->read_from_rx_buffer(buffer, sizeof(buffer));
    
    if (bytes_received > 0) {
        // 处理接收到的数据
        process_dma_received_data(buffer, bytes_received);
    }
}

// 发送响应
bool Mai2Serial::send_response(const std::string& response) {
    if (!is_ready()) {
        return false;
    }
    
    std::string full_response = response + "\r\n";
    // 使用新的DMA接口：先写入TX缓冲区，然后触发DMA传输
    size_t bytes_written = uart_hal_->write_to_tx_buffer((uint8_t*)full_response.c_str(), full_response.length());
    if (bytes_written > 0) {
        uart_hal_->trigger_tx_dma();
    }
    return (bytes_written == full_response.length());
}

// 设置回调
void Mai2Serial::set_touch_callback(Mai2Serial_TouchCallback callback) {
    touch_callback_ = callback;
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
    
    // 启动DMA接收模式
    start_dma_receive();
    
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
    
    // 检查波特率是否有效
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
    
    // 重新初始化UART以应用新的波特率
    uart_hal_->deinit();
    return uart_hal_->init(0, 1, baud_rate, false);
}

bool Mai2Serial::set_sample_time(uint16_t sample_time_ms) {
    config_.sample_time = (uint8_t)(sample_time_ms / 10); // 转换为内部单位
    
    // 通知回调
    if (command_callback_) {
        uint8_t sample_time_val = config_.sample_time;
        command_callback_("SET_SAMPLE_TIME", &sample_time_val, 1);
    }
    
    return true;
}

// 显示信息
void Mai2Serial::show_bind_info() {
    send_response("=== Touch Point Binding ===");
    for (int i = 0; i < 34; i++) {  // 34个触摸点
        char info[64];
        snprintf(info, sizeof(info), "Point %d: ACTIVE", i);
        send_response(info);
    }
}

// 移除了show_sensitivity_info函数

void Mai2Serial::show_status_info() {
    const char* status_str = "UNKNOWN";
    switch (status_) {
        case Status::STOPPED: status_str = "STOPPED"; break;
        case Status::READY: status_str = "READY"; break;
        case Status::RUNNING: status_str = "RUNNING"; break;
        case Status::ERROR: status_str = "ERROR"; break;
    }
    
    char info[128];
    snprintf(info, sizeof(info), "Status: %s, Baud: %u", 
             status_str, config_.baud_rate);
    send_response(info);
}

// 任务处理
void Mai2Serial::task() {
    if (!initialized_) {
        return;
    }
    
    // 处理接收到的命令
    process_commands();
    
    // 如果有触摸回调且状态为运行，定期检查触摸数据
    if (status_ == Status::RUNNING && touch_callback_) {
        static uint32_t last_check_time = 0;
        uint32_t current_time = time_us_32() / 1000;
        
        if (current_time - last_check_time >= config_.sample_time_ms) {
            // 这里可以主动请求触摸数据
            // 实际实现中，触摸数据通常由外部模块推送
            last_check_time = current_time;
        }
    }
}

// 发送触摸状态 - 使用新的bitmap数据结构
bool Mai2Serial::send_touch_state(uint32_t state1, uint32_t state2) {
    // 直接创建Mai2Serial_TouchData对象并调用send_touch_data
    Mai2Serial_TouchData touch_data(state1, (uint8_t)(state2 & 0x0F));
    return send_touch_data(touch_data);
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
            break;
            
        case MAI2SERIAL_CMD_HALT:  // L - 停止
            // 停止操作
            stop();
            serial_ok_ = false;  // 进入设置模式，停止发送触摸数据
            break;
            
        case MAI2SERIAL_CMD_RATIO: // r - 比例设置
            // 发送比例响应
            send_command_response(lr, sensor, 'r', value);
            break;
            
        case MAI2SERIAL_CMD_SENS:  // k - 灵敏度设置
            // 模拟灵敏度设置响应（不实际修改）
            send_command_response('R', sensor, 'k', value);
            break;
            
        case MAI2SERIAL_CMD_STAT:  // A - 状态
            // 启动采样状态
            start();
            serial_ok_ = true;   // 开始发送触摸数据
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
    
    // 使用新的DMA接口：先写入TX缓冲区，然后触发DMA传输
    size_t written = uart_hal_->write_to_tx_buffer(response, sizeof(response));
    if (written > 0) {
        uart_hal_->trigger_tx_dma();
    }
}

// 启动DMA接收
void Mai2Serial::start_dma_receive() {
    if (!is_ready()) {
        return;
    }
    
    // 启动DMA接收模式 - 使用环形缓冲区持续接收
    // HAL_UART已经在初始化时启用了中断接收到环形缓冲区
    // 这里不需要额外操作，数据会自动进入缓冲区
}

// 处理DMA接收的数据
void Mai2Serial::process_dma_received_data(const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return;
    }
    
    // 查找指令包
    for (size_t i = 0; i < length; i++) {
        if (data[i] == MAI2SERIAL_CMD_START_BYTE) {
            // 找到指令开始，处理这个包
            size_t remaining = length - i;
            if (remaining >= MAI2SERIAL_COMMAND_LENGTH) {
                process_command_packet(&data[i], MAI2SERIAL_COMMAND_LENGTH);
            }
        }
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
        case 'E': // 重置指令 (commandRSET)
            reset();
            serial_ok_ = false;  // 重置时停止发送触摸数据
            break;
            
        case 'L': // 停止指令 (commandHALT)
            stop();
            serial_ok_ = false;  // 进入设置模式，停止发送触摸数据
            break;
            
        case 'A': // 状态指令 (commandSTAT)
            start();
            serial_ok_ = true;   // 开始发送触摸数据
            break;
            
        case 'r': // 比例指令 (commandRatio)
            send_response("OK");
            break;
            
        case 'k': // 灵敏度指令 (commandSens) - 不实际执行灵敏度设置
            // 解析参数但不实际设置灵敏度
            if (!param.empty()) {
                // 模拟成功响应
                send_response("OK");
            } else {
                send_response("ERROR: Missing parameter");
            }
            break;
            
        default:
            // 忽略所有其他命令
            break;
    }
    
    // 通知命令回调
    if (command_callback_) {
        const char* cmd_str = "UNKNOWN";
        switch (cmd) {
            case 'E': cmd_str = "RESET"; break;
            case 'L': cmd_str = "HALT"; break;
            case 'A': cmd_str = "STAT"; break;
            case 'r': cmd_str = "RATIO"; break;
            case 'k': cmd_str = "SENS"; break;
        }
        uint8_t* param_data = param.empty() ? nullptr : (uint8_t*)param.c_str();
        uint8_t param_count = param.empty() ? 0 : param.length();
        command_callback_(cmd_str, param_data, param_count);
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