#include "mai2light.h"
#include "pico/time.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cmath>

// RGB颜色静态方法实现
Mai2Light_RGB Mai2Light_RGB::from_hsv(uint16_t hue, uint8_t saturation, uint8_t value) {
    Mai2Light_RGB rgb;
    
    if (saturation == 0) {
        rgb.r = rgb.g = rgb.b = value;
        return rgb;
    }
    
    uint8_t region = hue / 43;
    uint8_t remainder = (hue - (region * 43)) * 6;
    
    uint8_t p = (value * (255 - saturation)) >> 8;
    uint8_t q = (value * (255 - ((saturation * remainder) >> 8))) >> 8;
    uint8_t t = (value * (255 - ((saturation * (255 - remainder)) >> 8))) >> 8;
    
    switch (region) {
        case 0: rgb.r = value; rgb.g = t; rgb.b = p; break;
        case 1: rgb.r = q; rgb.g = value; rgb.b = p; break;
        case 2: rgb.r = p; rgb.g = value; rgb.b = t; break;
        case 3: rgb.r = p; rgb.g = q; rgb.b = value; break;
        case 4: rgb.r = t; rgb.g = p; rgb.b = value; break;
        default: rgb.r = value; rgb.g = p; rgb.b = q; break;
    }
    
    return rgb;
}

Mai2Light_RGB Mai2Light_RGB::blend(const Mai2Light_RGB& other, uint8_t ratio) const {
    Mai2Light_RGB result;
    result.r = (r * (255 - ratio) + other.r * ratio) / 255;
    result.g = (g * (255 - ratio) + other.g * ratio) / 255;
    result.b = (b * (255 - ratio) + other.b * ratio) / 255;
    return result;
}

// Mai2Light构造函数
Mai2Light::Mai2Light(HAL_UART* uart_hal, uint8_t node_id) 
    : uart_hal_(uart_hal)
    , initialized_(false)
    , rx_buffer_pos_(0)
    , is_fading_(false)
    , fade_start_time_(0)
    , string_cmd_pos_(0) {
    
    config_.node_id = node_id;
    
    // 初始化LED状态
    for (int32_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        led_status_[i] = Mai2Light_LEDStatus();
        fade_start_colors_[i] = Mai2Light_RGB();
        fade_target_colors_[i] = Mai2Light_RGB();
    }
    
    // 初始化板卡信息
    board_info_.board_id = 0x1507;
    board_info_.hardware_version = 4;
    board_info_.firmware_version = 1;
    board_info_.led_count = MAI2LIGHT_NUM_LEDS;
    board_info_.serial_number = 0x12345678;
    
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(string_cmd_buffer_, 0, sizeof(string_cmd_buffer_));
    
    // 初始化虚拟EEPROM
    memset(virtual_eeprom_, 0, EEPROM_SIZE);
}

// Mai2Light析构函数
Mai2Light::~Mai2Light() {
    deinit();
}

// 初始化
bool Mai2Light::init() {
    if (!uart_hal_ || initialized_) {
        return false;
    }
    
    uart_hal_->set_baudrate(config_.baud_rate);
    initialized_ = true;

    // 发送初始化完成消息
    log_debug("Mai2Light initialized");
    
    return initialized_;
}

// 释放资源
void Mai2Light::deinit() {
    if (initialized_) {
        // 清除所有LED
        clear_all_leds();
        
        uart_hal_->deinit();
        initialized_ = false;
        
        // 清理回调
        command_callback_ = nullptr;
    }
}

// 检查是否就绪
bool Mai2Light::is_ready() const {
    return initialized_;
}

// 设置配置
bool Mai2Light::set_config(const Mai2Light_Config& config) {
    config_ = config;
    
    if (initialized_) {
        // 重新配置UART波特率 - HAL_UART不支持动态重配置
        // 需要重新初始化UART
        uart_hal_->deinit();
        return uart_hal_->set_baudrate(config.baud_rate);
    }
    
    return true;
}

// 获取配置
bool Mai2Light::get_config(Mai2Light_Config& config) {
    config = config_;
    return true;
}

// 设置单个LED颜色
bool Mai2Light::set_led_color(uint8_t led_index, const Mai2Light_RGB& color) {
    if (!is_ready() || led_index >= MAI2LIGHT_NUM_LEDS) {
        return false;
    }
    
    if (config_.enable_fade && config_.fade_time_ms > 0) {
        // 启动渐变效果
        fade_start_colors_[led_index] = led_status_[led_index].color;
        fade_target_colors_[led_index] = color;
        
        if (!is_fading_) {
            is_fading_ = true;
            fade_start_time_ = time_us_32() / 1000;
        }
    } else {
        // 直接设置颜色
        led_status_[led_index].color = color;
    }
    
    return true;
}

// 设置单个LED颜色（RGB分量）
bool Mai2Light::set_led_color(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    return set_led_color(led_index, Mai2Light_RGB(r, g, b));
}

// 设置单个LED亮度
bool Mai2Light::set_led_brightness(uint8_t led_index, uint8_t brightness) {
    if (!is_ready() || led_index >= MAI2LIGHT_NUM_LEDS) {
        return false;
    }
    
    led_status_[led_index].brightness = brightness;
    return true;
}

// 设置所有LED颜色
bool Mai2Light::set_all_leds(const Mai2Light_RGB& color) {
    if (!is_ready()) {
        return false;
    }
    
    for (int32_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        set_led_color(i, color);
    }
    
    return true;
}

// 设置全局亮度
bool Mai2Light::set_global_brightness(uint8_t brightness) {
    if (!is_ready()) {
        return false;
    }
    
    config_.global_brightness = brightness;
    
    // 应用到所有LED
    for (int32_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        led_status_[i].brightness = brightness;
    }
    
    return true;
}

// 设置渐变时间
bool Mai2Light::set_fade_time(uint16_t fade_time_ms) {
    config_.fade_time_ms = fade_time_ms;
    return true;
}

// 获取LED状态
bool Mai2Light::get_led_status(uint8_t led_index, Mai2Light_LEDStatus& status) {
    if (led_index >= MAI2LIGHT_NUM_LEDS) {
        return false;
    }
    
    status = led_status_[led_index];
    return true;
}

// 获取所有LED状态
bool Mai2Light::get_all_led_status(Mai2Light_LEDStatus status_array[MAI2LIGHT_NUM_LEDS]) {
    if (!status_array) {
        return false;
    }
    
    for (uint8_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        status_array[i] = led_status_[i];
    }
    return true;
}

// 获取LED状态数组指针
const Mai2Light_LEDStatus* Mai2Light::get_led_status_array() const {
    return led_status_;
}

// 获取板卡信息
bool Mai2Light::get_board_info(Mai2Light_BoardInfo& info) {
    info = board_info_;
    return true;
}

// 获取协议版本
uint8_t Mai2Light::get_protocol_version() {
    return 0x10; // 版本 1.0
}

// 保存到EEPROM
bool Mai2Light::save_to_eeprom() {
    log_debug("Saving configuration to virtual EEPROM");
    
    // 将当前配置保存到虚拟EEPROM
    uint16_t offset = 0;
    
    // 保存配置结构体
    if (offset + sizeof(config_) <= EEPROM_SIZE) {
        memcpy(&virtual_eeprom_[offset], &config_, sizeof(config_));
        offset += sizeof(config_);
    } else {
        log_error("EEPROM save failed: config too large");
        return false;
    }
    
    // 保存LED状态
    if (offset + sizeof(led_status_) <= EEPROM_SIZE) {
        memcpy(&virtual_eeprom_[offset], led_status_, sizeof(led_status_));
        offset += sizeof(led_status_);
    } else {
        log_error("EEPROM save failed: LED status too large");
        return false;
    }
    
    // 保存板卡信息
    if (offset + sizeof(board_info_) <= EEPROM_SIZE) {
        memcpy(&virtual_eeprom_[offset], &board_info_, sizeof(board_info_));
        offset += sizeof(board_info_);
    } else {
        log_error("EEPROM save failed: board info too large");
        return false;
    }
    
    log_debug("Configuration saved to EEPROM (" + std::to_string(offset) + " bytes)");
    return true;
}

// 从EEPROM加载
bool Mai2Light::load_from_eeprom() {
    log_debug("Loading configuration from virtual EEPROM");
    
    uint16_t offset = 0;
    
    // 加载配置结构体
    if (offset + sizeof(config_) <= EEPROM_SIZE) {
        memcpy(&config_, &virtual_eeprom_[offset], sizeof(config_));
        offset += sizeof(config_);
    } else {
        log_error("EEPROM load failed: config size mismatch");
        return false;
    }
    
    // 加载LED状态
    if (offset + sizeof(led_status_) <= EEPROM_SIZE) {
        memcpy(led_status_, &virtual_eeprom_[offset], sizeof(led_status_));
        offset += sizeof(led_status_);
    } else {
        log_error("EEPROM load failed: LED status size mismatch");
        return false;
    }
    
    // 加载板卡信息
    if (offset + sizeof(board_info_) <= EEPROM_SIZE) {
        memcpy(&board_info_, &virtual_eeprom_[offset], sizeof(board_info_));
        offset += sizeof(board_info_);
    } else {
        log_error("EEPROM load failed: board info size mismatch");
        return false;
    }
    
    log_debug("Configuration loaded from EEPROM (" + std::to_string(offset) + " bytes)");
    return true;
}

// 设置EEPROM数据
bool Mai2Light::set_eeprom_data(uint16_t address, const uint8_t* data, uint8_t length) {
    if (!data || length == 0) {
        log_error("EEPROM write failed: invalid parameters");
        return false;
    }
    
    if (address + length > EEPROM_SIZE) {
        log_error("EEPROM write failed: address out of range (" + std::to_string(address) +
                   " + " + std::to_string(length) + " > " + std::to_string(sizeof(virtual_eeprom_)) + ")");
        return false;
    }
    
    memcpy(&virtual_eeprom_[address], data, length);
    log_debug("EEPROM write: " + std::to_string(length) + " bytes at address " + std::to_string(address));
    return true;
}

// 获取EEPROM数据
bool Mai2Light::get_eeprom_data(uint16_t address, uint8_t* data, uint8_t length) {
    if (!data || length == 0) {
        log_error("EEPROM read failed: invalid parameters");
        return false;
    }
    
    if (address + length > EEPROM_SIZE) {
        log_error("EEPROM read failed: address out of range (" + std::to_string(address) +
                   " + " + std::to_string(length) + " > " + std::to_string(sizeof(virtual_eeprom_)) + ")");
        return false;
    }
    
    memcpy(data, &virtual_eeprom_[address], length);
    log_debug("EEPROM read: " + std::to_string(length) + " bytes from address " + std::to_string(address));
    return true;
}

// 重置板卡
bool Mai2Light::reset_board() {
    log_debug("Resetting board");
    
    // 清除所有LED
    clear_all_leds();
    
    // 重置配置为默认值
    config_ = Mai2Light_Config();
    
    return true;
}

// 清除所有LED - mai2light只管理自己的LED状态
bool Mai2Light::clear_all_leds() {
    log_debug("Clearing all LEDs");
    
    // 清除mai2light管理的所有LED状态
    Mai2Light_RGB black_color = {0, 0, 0};
    for (uint8_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        led_status_[i].color = black_color;
        led_status_[i].brightness = 0;
    }
    
    return true;
}

// 进入引导程序
bool Mai2Light::enter_bootloader() {
    log_debug("Entering bootloader mode");
    // 这里应该实现进入引导程序的逻辑
    return true;
}

// 设置回调
void Mai2Light::set_command_callback(Mai2Light_CommandCallback callback) {
    command_callback_ = callback;
}

// 任务处理
void Mai2Light::task() {
    if (!initialized_) {
        return;
    }
    
    // 处理接收到的数据
    process_received_data();
    
    // 更新渐变效果
    if (is_fading_) {
        update_fade_effects();
    }
}

// 发送数据包
bool Mai2Light::send_packet(const Mai2Light_PacketReq& packet) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t buffer[MAI2LIGHT_MAX_PACKET_SIZE];
    uint8_t pos = 0;
    
    buffer[pos++] = packet.sync;
    buffer[pos++] = packet.node_id;
    buffer[pos++] = packet.length;
    buffer[pos++] = (uint8_t)packet.command;
    
    // 复制数据
    for (int32_t i = 0; i < packet.length && pos < sizeof(buffer) - 1; i++) {
        buffer[pos++] = packet.data[i];
    }
    
    // 计算校验和
    buffer[pos] = calculate_checksum(buffer, pos);
    pos++;
    
    // 使用新的DMA接口：写入TX缓冲区会自动处理DMA传输
    uart_hal_->write_to_tx_buffer(buffer, pos);
    return true;
}

// 发送命令
bool Mai2Light::send_command(Mai2Light_Command command, const uint8_t* data, uint8_t data_length) {
    Mai2Light_PacketReq packet;
    packet.node_id = config_.node_id;
    packet.command = command;
    packet.length = data_length;
    
    if (data && data_length > 0) {
        memcpy(packet.data, data, std::min(data_length, (uint8_t)sizeof(packet.data)));
    }
    
    return send_packet(packet);
}

// 处理接收到的数据
void Mai2Light::process_received_data() {
    uint8_t buffer[32];
    
    size_t bytes_read = uart_hal_->read_from_rx_buffer(buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
        std::string hex_str;
        for(size_t i = 0; i < bytes_read; i++) {
            hex_str += buffer[i];
        }
        log_debug("Received=" + hex_str  + " bytes=" + std::to_string(bytes_read));
        // 先处理字符串指令
        process_string_commands(buffer, bytes_read);
        
        for (size_t i = 0; i < bytes_read; i++) {
            rx_buffer_[rx_buffer_pos_++] = buffer[i];
            
            // 检查缓冲区溢出
            if (rx_buffer_pos_ >= sizeof(rx_buffer_)) {
                rx_buffer_pos_ = 0;
            }
            
            // 尝试解析数据包
            if (rx_buffer_pos_ >= 4) {
                uint8_t expected_length = rx_buffer_[2] + 5; // length + header + checksum
                if (rx_buffer_pos_ >= expected_length) {
                    Mai2Light_PacketReq packet;
                    if (parse_packet(rx_buffer_, expected_length, packet)) {
                        process_packet(packet);
                    }
                    
                    // 移除已处理的数据包
                    memmove(rx_buffer_, rx_buffer_ + expected_length, rx_buffer_pos_ - expected_length);
                    rx_buffer_pos_ -= expected_length;
                }
            }
        }
    }
}

// 处理数据包
void Mai2Light::process_packet(const Mai2Light_PacketReq& packet) {
    // 检查节点ID
    if (packet.node_id != config_.node_id && packet.node_id != 0xFF) {
        return; // 不是发给我们的数据包
    }
    
    switch (packet.command) {
        case Mai2Light_Command::SET_LED_GS_8BIT:
        case Mai2Light_Command::SET_LED_GS_8BIT_MULTI:
        case Mai2Light_Command::SET_LED_RGB:
        case Mai2Light_Command::SET_LED_RGB_MULTI:
        case Mai2Light_Command::SET_ALL_LEDS:
        case Mai2Light_Command::SET_BRIGHTNESS:
        case Mai2Light_Command::SET_FADE_TIME:
            handle_set_led_command(packet);
            break;
            
        case Mai2Light_Command::GET_LED_STATUS:
        case Mai2Light_Command::GET_BOARD_INFO:
        case Mai2Light_Command::GET_PROTOCOL_VERSION:
            handle_get_status_command(packet);
            break;
            
        case Mai2Light_Command::SET_EEPROM:
        case Mai2Light_Command::GET_EEPROM:
        case Mai2Light_Command::SAVE_TO_EEPROM:
        case Mai2Light_Command::LOAD_FROM_EEPROM:
            handle_eeprom_command(packet);
            break;
            
        case Mai2Light_Command::GET_HELP:
            handle_get_status_command(packet);
            break;
            
        case Mai2Light_Command::RESET_BOARD:
        case Mai2Light_Command::ENTER_BOOTLOADER:
            handle_system_command(packet);
            break;
            
        default:
            send_ack(packet.command, Mai2Light_AckStatus::INVALID_COMMAND);
            break;
    }
    
    // 通知回调
    if (command_callback_) {
        command_callback_(packet.command, packet.data, packet.length);
    }
}

// 解析数据包
bool Mai2Light::parse_packet(const uint8_t* buffer, uint8_t length, Mai2Light_PacketReq& packet) {
    if (!buffer || length < 5) {
        return false;
    }
    
    // 检查同步字节
    if (buffer[0] != MAI2LIGHT_SYNC_BYTE) {
        return false;
    }
    
    // 验证校验和
    if (!verify_checksum(buffer, length - 1, buffer[length - 1])) {
        return false;
    }
    
    packet.sync = buffer[0];
    packet.node_id = buffer[1];
    packet.length = buffer[2];
    packet.command = (Mai2Light_Command)buffer[3];
    
    // 复制数据
    for (int32_t i = 0; i < packet.length && i < sizeof(packet.data); i++) {
        packet.data[i] = buffer[4 + i];
    }
    
    packet.checksum = buffer[length - 1];
    
    return true;
}

// 计算校验和
uint8_t Mai2Light::calculate_checksum(const uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (int32_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 验证校验和
bool Mai2Light::verify_checksum(const uint8_t* data, uint8_t length, uint8_t expected_checksum) {
    return calculate_checksum(data, length) == expected_checksum;
}

// 处理LED设置命令
void Mai2Light::handle_set_led_command(const Mai2Light_PacketReq& packet) {
    bool success = false;
    
    switch (packet.command) {
        case Mai2Light_Command::SET_LED_RGB:
            if (packet.length >= 4) {
                uint8_t led_index = packet.data[0];
                uint8_t r = packet.data[1];
                uint8_t g = packet.data[2];
                uint8_t b = packet.data[3];
                success = set_led_color(led_index, r, g, b);
            }
            break;
            
        case Mai2Light_Command::SET_ALL_LEDS:
            if (packet.length >= 3) {
                uint8_t r = packet.data[0];
                uint8_t g = packet.data[1];
                uint8_t b = packet.data[2];
                success = set_all_leds(Mai2Light_RGB(r, g, b));
            }
            break;
            
        case Mai2Light_Command::SET_BRIGHTNESS:
            if (packet.length >= 1) {
                success = set_global_brightness(packet.data[0]);
            }
            break;
            
        case Mai2Light_Command::SET_FADE_TIME:
            if (packet.length >= 2) {
                uint16_t fade_time = (packet.data[1] << 8) | packet.data[0];
                success = set_fade_time(fade_time);
            }
            break;
            
        default:
            break;
    }
    
    send_ack(packet.command, success ? Mai2Light_AckStatus::OK : Mai2Light_AckStatus::INVALID_PARAMETER);
}

// 处理状态查询命令
void Mai2Light::handle_get_status_command(const Mai2Light_PacketReq& packet) {
    uint8_t response_data[32];
    uint8_t response_length = 0;
    
    switch (packet.command) {
        case Mai2Light_Command::GET_BOARD_INFO:
            response_data[0] = board_info_.board_id & 0xFF;
            response_data[1] = (board_info_.board_id >> 8) & 0xFF;
            response_data[2] = board_info_.hardware_version;
            response_data[3] = board_info_.firmware_version;
            response_data[4] = board_info_.led_count & 0xFF;
            response_data[5] = (board_info_.led_count >> 8) & 0xFF;
            response_length = 6;
            break;
            
        case Mai2Light_Command::GET_PROTOCOL_VERSION:
            response_data[0] = get_protocol_version();
            response_length = 1;
            break;
            
        case Mai2Light_Command::GET_LED_STATUS:
            if (packet.length >= 1) {
                uint8_t led_index = packet.data[0];
                if (led_index < MAI2LIGHT_NUM_LEDS) {
                    response_data[0] = led_status_[led_index].color.r;
                    response_data[1] = led_status_[led_index].color.g;
                    response_data[2] = led_status_[led_index].color.b;
                    response_data[3] = led_status_[led_index].brightness;
                    response_data[4] = led_status_[led_index].enabled ? 1 : 0;
                    response_length = 5;
                }
            }
            break;
            
        case Mai2Light_Command::GET_HELP: {
            const char* help_text = "Mai2Light Module Commands: SET_LED_RGB(0x03), SET_ALL_LEDS(0x05), SET_BRIGHTNESS(0x06), GET_LED_STATUS(0x10), GET_BOARD_INFO(0x11), RESET_BOARD(0x30), GET_HELP(0x2F)";
            uint8_t help_len = strlen(help_text);
            if (help_len > 32) help_len = 32; // 限制长度
            memcpy(response_data, help_text, help_len);
            response_length = help_len;
            break;
        }
            
        default:
            break;
    }
    
    send_ack(packet.command, Mai2Light_AckStatus::OK, Mai2Light_AckReport::OK, response_data, response_length);
}

// 处理EEPROM命令
void Mai2Light::handle_eeprom_command(const Mai2Light_PacketReq& packet) {
    bool success = false;
    
    switch (packet.command) {
        case Mai2Light_Command::SAVE_TO_EEPROM:
            success = save_to_eeprom();
            break;
            
        case Mai2Light_Command::LOAD_FROM_EEPROM:
            success = load_from_eeprom();
            break;
            
        case Mai2Light_Command::SET_EEPROM:
            if (packet.length >= 3) {
                uint16_t address = (packet.data[1] << 8) | packet.data[0];
                uint8_t length = packet.data[2];
                success = set_eeprom_data(address, &packet.data[3], length);
            }
            break;
            
        case Mai2Light_Command::GET_EEPROM:
            if (packet.length >= 3) {
                uint16_t address = (packet.data[1] << 8) | packet.data[0];
                uint8_t length = packet.data[2];
                uint8_t eeprom_data[16];
                if (get_eeprom_data(address, eeprom_data, length)) {
                    send_ack(packet.command, Mai2Light_AckStatus::OK, Mai2Light_AckReport::OK, eeprom_data, length);
                    return;
                }
            }
            break;
            
        default:
            break;
    }
    
    send_ack(packet.command, success ? Mai2Light_AckStatus::OK : Mai2Light_AckStatus::EEPROM_ERROR);
}

// 处理系统命令
void Mai2Light::handle_system_command(const Mai2Light_PacketReq& packet) {
    bool success = false;
    
    switch (packet.command) {
        case Mai2Light_Command::RESET_BOARD:
            success = reset_board();
            break;
            
        case Mai2Light_Command::ENTER_BOOTLOADER:
            success = enter_bootloader();
            break;
            
        default:
            break;
    }
    
    send_ack(packet.command, success ? Mai2Light_AckStatus::OK : Mai2Light_AckStatus::HARDWARE_ERROR);
}

// 发送应答
void Mai2Light::send_ack(Mai2Light_Command command, Mai2Light_AckStatus status, 
                        Mai2Light_AckReport report, const uint8_t* data, uint8_t data_length) {
    Mai2Light_PacketAck ack;
    ack.node_id = config_.node_id;
    ack.command = command;
    ack.status = status;
    ack.report = report;
    ack.length = data_length + 2; // status + report + data
    
    if (data && data_length > 0) {
        memcpy(ack.data, data, std::min(data_length, (uint8_t)sizeof(ack.data)));
    }
    
    // 构建应答数据包
    uint8_t buffer[MAI2LIGHT_MAX_PACKET_SIZE];
    uint8_t pos = 0;
    
    buffer[pos++] = ack.sync;
    buffer[pos++] = ack.node_id;
    buffer[pos++] = ack.length;
    buffer[pos++] = (uint8_t)ack.command;
    buffer[pos++] = (uint8_t)ack.status;
    buffer[pos++] = (uint8_t)ack.report;
    
    // 复制数据
    for (int32_t i = 0; i < data_length && pos < sizeof(buffer) - 1; i++) {
        buffer[pos++] = ack.data[i];
    }
    
    // 计算校验和
    buffer[pos] = calculate_checksum(buffer, pos);
    pos++;
    
    // 使用新的DMA接口：写入TX缓冲区会自动处理DMA传输
    (void)uart_hal_->write_to_tx_buffer(buffer, pos);
}

// 日志输出
void Mai2Light::log_debug(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->debug(message, "Mai2Light");
    }
}

void Mai2Light::log_error(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->error(message, "Mai2Light");
    }
}

// 字符串指令解析实现
void Mai2Light::process_string_commands(const uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        char c = (char)buffer[i];
        
        log_debug("Char: '" + std::string(1, c) + "' (0x" + std::to_string((uint8_t)c) + "), pos=" + std::to_string(string_cmd_pos_));
        
        // 处理换行符，解析完整指令
        if (c == '\n' || c == '\r') {
            if (string_cmd_pos_ > 0) {
                string_cmd_buffer_[string_cmd_pos_] = '\0';
                std::string command_str(string_cmd_buffer_);
                parse_string_command(command_str);
                string_cmd_pos_ = 0;
            }
        }
        // 累积字符到缓冲区
        else if (string_cmd_pos_ < 63) {  // 64-1，为null终止符预留空间
            string_cmd_buffer_[string_cmd_pos_++] = c;
        }
        // 缓冲区溢出，重置
        else {
            log_error("String command buffer overflow, resetting");
            string_cmd_pos_ = 0;
        }
    }
}

void Mai2Light::parse_string_command(const std::string& command_str) {
    // 去除命令字符串的前后空白字符
    std::string trimmed_cmd = command_str;
    // 去除尾部空白字符
    trimmed_cmd.erase(trimmed_cmd.find_last_not_of(" \t\r\n") + 1);
    // 去除头部空白字符
    trimmed_cmd.erase(0, trimmed_cmd.find_first_not_of(" \t\r\n"));
    
    // 检查是否为/help指令
    if (trimmed_cmd == "/help") {
        std::string help_text = "Mai2Light Help:\r\n";
        help_text += "Commands:\r\n";
        help_text += "  /help - Show this help message\r\n";
        help_text += "  /status - Get board status\r\n";
        help_text += "  /reset - Reset the board\r\n";
        help_text += "  /version - Get firmware version\r\n";
        help_text += "Protocol Commands:\r\n";
        help_text += "  SET_LED_RGB - Set LED RGB color\r\n";
        help_text += "  SET_ALL_LEDS - Set all LEDs color\r\n";
        help_text += "  GET_LED_STATUS - Get LED status\r\n";
        help_text += "  GET_BOARD_INFO - Get board information\r\n";
        help_text += "  RESET_BOARD - Reset the board\r\n";
        help_text += "\r\n";
        
        send_string_response(help_text);
    }
    // 添加其他字符串命令处理
    else if (trimmed_cmd == "/status") {
        std::string status_text = "Board Status: OK\r\n";
        status_text += "Node ID: " + std::to_string(config_.node_id) + "\r\n";
        status_text += "Initialized: " + std::string(initialized_ ? "Yes" : "No") + "\r\n";
        send_string_response(status_text);
    }
    else if (trimmed_cmd == "/version") {
        std::string version_text = "Mai2Light Firmware v1.0\r\n";
        version_text += "Protocol Version: " + std::to_string(get_protocol_version()) + "\r\n";
        send_string_response(version_text);
    }
    else if (trimmed_cmd == "/reset") {
        send_string_response("Resetting board...\r\n");
        reset_board();
    }
    else {
        send_string_response("Unknown command: " + trimmed_cmd + "\r\nType /help for available commands\r\n");
    }
}

bool Mai2Light::send_string_response(const std::string& response) {
    log_debug("Sending string response: '" + response + "' (" + std::to_string(response.length()) + " bytes)");
    
    if (uart_hal_) {
        size_t written = uart_hal_->write_to_tx_buffer((const uint8_t*)response.c_str(), response.length());
        log_debug("UART write result: " + std::to_string(written) + "/" + std::to_string(response.length()) + " bytes");
        
        if (written != response.length()) {
            log_error("Failed to write all bytes to UART TX buffer");
        }
        
        return written == response.length();
    }
    
    log_error("UART HAL is null, cannot send response");
    return false;
}

// 更新渐变效果
void Mai2Light::update_fade_effects() {
    if (!is_fading_) {
        return;
    }
    
    uint32_t current_time = time_us_32() / 1000;
    uint32_t elapsed_time = current_time - fade_start_time_;
    
    if (elapsed_time >= config_.fade_time_ms) {
        // 渐变完成
        for (int32_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
            led_status_[i].color = fade_target_colors_[i];
        }
        is_fading_ = false;
    } else {
        // 计算渐变进度
        uint8_t progress = (elapsed_time * 255) / config_.fade_time_ms;
        
        for (int32_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
            led_status_[i].color = fade_start_colors_[i].blend(fade_target_colors_[i], progress);
        }
    }
}