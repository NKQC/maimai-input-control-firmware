#include "usb_serial_logs.h"
#include "pico/time.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <algorithm>
#include "pico/stdlib.h"

// 静态全局实例
USB_SerialLogs* USB_SerialLogs::global_instance_ = nullptr;

// ANSI颜色代码
static const char* ANSI_RESET = "\033[0m";
static const char* ANSI_RED = "\033[31m";
static const char* ANSI_GREEN = "\033[32m";
static const char* ANSI_YELLOW = "\033[33m";
static const char* ANSI_CYAN = "\033[36m";
static const char* ANSI_WHITE = "\033[37m";
static const char* ANSI_BRIGHT_RED = "\033[91m";

// 构造函数
USB_SerialLogs::USB_SerialLogs(HAL_USB* usb_hal)
    : usb_hal_(usb_hal)
    , initialized_(false)
    , last_flush_time_(0) {
}

// 析构函数
USB_SerialLogs::~USB_SerialLogs() {
    deinit();
    
    // 如果是全局实例，清除引用
    if (global_instance_ == this) {
        global_instance_ = nullptr;
    }
}

// 初始化
bool USB_SerialLogs::init() {
    if (!usb_hal_ || initialized_) {
        return false;
    }
    
    initialized_ = true;
    last_flush_time_ = time_us_32() / 1000;
    
    // 发送初始化消息
    info("USB Serial Logs initialized", "USB_LOGS");
    
    return true;
}

// 释放资源
void USB_SerialLogs::deinit() {
    if (initialized_) {
        // 刷新缓冲区
        flush();

        initialized_ = false;
        
        // 清理环形缓冲区
        log_buffer_.clear();
        
        // 清理回调
        log_callback_ = nullptr;
        error_callback_ = nullptr;
    }
}

// 检查是否就绪
bool USB_SerialLogs::is_ready() const {
    return initialized_ && 
           usb_hal_ != nullptr && 
           usb_hal_->is_connected();
}

// 设置配置
bool USB_SerialLogs::set_config(const USB_SerialLogs_Config config) {
    config_ = config;
    return true;
}

// 获取配置
bool USB_SerialLogs::get_config(USB_SerialLogs_Config& config) {
    config = config_;
    return true;
}

// 通用日志输出
void USB_SerialLogs::log(USB_LogLevel level, const std::string& message, const std::string& tag) {
    if (!should_log(level)) {
        return;
    }
    
    USB_LogEntry entry(level, message, tag);
    
    add_to_queue(entry);
    
    update_statistics(level);
    
    // 通知回调
    if (log_callback_) {
        log_callback_(entry);
    }
}

// 各级别日志方法
void USB_SerialLogs::debug(const std::string& message, const std::string& tag) {
    log(USB_LogLevel::DEBUG, message, tag);
}

void USB_SerialLogs::info(const std::string& message, const std::string& tag) {
    log(USB_LogLevel::INFO, message, tag);
}

void USB_SerialLogs::warning(const std::string& message, const std::string& tag) {
    log(USB_LogLevel::WARNING, message, tag);
}

void USB_SerialLogs::error(const std::string& message, const std::string& tag) {
    log(USB_LogLevel::ERROR, message, tag);
}

void USB_SerialLogs::critical(const std::string& message, const std::string& tag) {
    log(USB_LogLevel::CRITICAL, message, tag);
}

// 格式化日志输出
void USB_SerialLogs::logf(USB_LogLevel level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    log(level, message);
}

void USB_SerialLogs::debugf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    debug(message);
}

void USB_SerialLogs::infof(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    info(message);
}

void USB_SerialLogs::warningf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    warning(message);
}

void USB_SerialLogs::errorf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    error(message);
}

void USB_SerialLogs::criticalf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::string message = format_string(format, args);
    va_end(args);
    
    critical(message);
}

// 原始数据输出
bool USB_SerialLogs::write_raw(const uint8_t* data, size_t length) {
    if (!is_ready() || !data || length == 0) {
        return false;
    }
    
    // 使用CDC写入方法
    bool result = usb_hal_->cdc_write(data, length);
    
    if (result) {
        stats_.bytes_sent += length;
        return true;
    }
    
    return false;
}

bool USB_SerialLogs::write_string(const std::string& str) {
    return write_raw(reinterpret_cast<const uint8_t*>(str.c_str()), str.length());
}

bool USB_SerialLogs::write_line(const std::string& line) {
    std::string line_with_newline = line + "\r\n";
    return write_string(line_with_newline);
}

// 刷新缓冲区
void USB_SerialLogs::flush() {
    if (!is_ready()) {
        return;
    }
    static uint32_t max_oneshot = USB_LOGS_MAX_ONESHOT;
    max_oneshot = USB_LOGS_MAX_ONESHOT;
    
    // 从字节级环形缓冲区读取并发送日志
    while (!log_buffer_.is_empty() && max_oneshot--) {
        // 检查是否有足够的数据读取头部
        if (log_buffer_.get_used_space() < sizeof(USB_LogEntryHeader)) {
            break;  // 数据不完整，等待更多数据
        }
        
        // 读取头部信息
        USB_LogEntryHeader header;
        size_t read_pos = log_buffer_.read_index;
        for (size_t i = 0; i < sizeof(USB_LogEntryHeader); i++) {
            reinterpret_cast<uint8_t*>(&header)[i] = log_buffer_.buffer[read_pos];
            read_pos = (read_pos + 1) % USB_LOGS_BUFFER_SIZE;
        }
        
        // 检查是否有足够的数据读取完整条目
        if (log_buffer_.get_used_space() < header.size) {
            break;  // 数据不完整，等待更多数据
        }
        
        // 读取标签
        std::string tag;
        tag.reserve(header.tag_length);
        for (size_t i = 0; i < header.tag_length; i++) {
            tag += static_cast<char>(log_buffer_.buffer[read_pos]);
            read_pos = (read_pos + 1) % USB_LOGS_BUFFER_SIZE;
        }
        
        // 读取消息
        std::string message;
        size_t msg_len = header.full_message_length;
        message.reserve(msg_len);
        for (size_t i = 0; i < msg_len; i++) {
            message += static_cast<char>(log_buffer_.buffer[read_pos]);
            read_pos = (read_pos + 1) % USB_LOGS_BUFFER_SIZE;
        }
        
        // 重构USB_LogEntry对象
        USB_LogEntry entry;
        entry.timestamp = header.timestamp;
        entry.level = header.level;
        entry.tag = std::move(tag);
        entry.message = std::move(message);
        
        // 格式化日志条目
        std::string formatted = format_log_entry(entry);
        formatted += "\r\n";
        
        // 检查消息长度是否合理
        if (formatted.length() > USB_LOGS_MAX_LINE_LENGTH) {
            // 消息过长，截断处理
            formatted = formatted.substr(0, USB_LOGS_MAX_LINE_LENGTH - 10) + "...\r\n";
        }
        
        // 直接发送，如果失败则停止处理
        if (usb_hal_->cdc_write((const uint8_t*)formatted.c_str(), formatted.length())) {
            stats_.bytes_sent += formatted.length();
            // 更新读取指针和已使用字节数
            log_buffer_.read_index = read_pos;
            log_buffer_.used_bytes -= header.size;
        } else {
            // 发送失败，停止处理，保留缓冲区中的数据
            break;
        }
    }
    
    last_flush_time_ = time_us_32() / 1000;
}

// 清除环形缓冲区
void USB_SerialLogs::clear_buffer() {
    log_buffer_.clear();
}

// 获取缓冲区已使用大小
size_t USB_SerialLogs::get_buffer_size() const {
    return log_buffer_.get_used_space();
}

// 检查缓冲区是否已满
bool USB_SerialLogs::is_buffer_full() const {
    return log_buffer_.is_full();
}

// 获取统计信息
bool USB_SerialLogs::get_statistics(Statistics& stats) {
    stats = stats_;
    return true;
}

// 重置统计信息
void USB_SerialLogs::reset_statistics() {
    stats_ = Statistics();
}

// 设置回调
void USB_SerialLogs::set_log_callback(USB_LogCallback callback) {
    log_callback_ = callback;
}

void USB_SerialLogs::set_error_callback(USB_ErrorCallback callback) {
    error_callback_ = callback;
}

// 任务处理
void USB_SerialLogs::task() {
    if (!initialized_) {
        return;
    }
    
    // 自动刷新
    if (config_.auto_flush) {
        uint32_t current_time = time_us_32() / 1000;
        if (current_time - last_flush_time_ >= config_.flush_interval_ms) {
            flush();
        }
    }
}

// 静态方法实现
void USB_SerialLogs::set_global_instance(USB_SerialLogs* instance) {
    global_instance_ = instance;
}

USB_SerialLogs* USB_SerialLogs::get_global_instance() {
    return global_instance_;
}

void USB_SerialLogs::global_log(USB_LogLevel level, const std::string& message, const std::string& tag) {
    if (global_instance_) {
        global_instance_->log(level, message, tag);
    }
}

// 私有方法实现
bool USB_SerialLogs::should_log(USB_LogLevel level) const {
    return static_cast<uint8_t>(level) >= static_cast<uint8_t>(config_.min_level);
}

std::string USB_SerialLogs::format_log_entry(const USB_LogEntry& entry) {
    std::string formatted;
    
    switch (config_.format) {
        case USB_LogFormat::SIMPLE:
            formatted = entry.message;
            break;
            
        case USB_LogFormat::TIMESTAMP:
            formatted = "[" + get_timestamp_string(entry.timestamp) + "] " + entry.message;
            break;
            
        case USB_LogFormat::FULL:
        default:
            formatted = "[" + get_timestamp_string(entry.timestamp) + "] ";
            
            if (config_.enable_colors) {
                formatted += get_level_color(entry.level);
            }
            
            formatted += "[" + get_level_string(entry.level) + "]";
            
            if (config_.enable_colors) {
                formatted += ANSI_RESET;
            }
            
            if (!entry.tag.empty()) {
                formatted += " [" + entry.tag + "]";
            }
            
            formatted += " " + entry.message;
            break;
    }
    
    return formatted;
}

std::string USB_SerialLogs::get_level_string(USB_LogLevel level) const {
    switch (level) {
        case USB_LogLevel::DEBUG:    return "DEBUG";
        case USB_LogLevel::INFO:     return "INFO";
        case USB_LogLevel::WARNING:  return "WARN";
        case USB_LogLevel::ERROR:    return "ERROR";
        case USB_LogLevel::CRITICAL: return "CRIT";
        default:                     return "UNKNOWN";
    }
}

std::string USB_SerialLogs::get_level_color(USB_LogLevel level) const {
    switch (level) {
        case USB_LogLevel::DEBUG:    return ANSI_CYAN;
        case USB_LogLevel::INFO:     return ANSI_GREEN;
        case USB_LogLevel::WARNING:  return ANSI_YELLOW;
        case USB_LogLevel::ERROR:    return ANSI_RED;
        case USB_LogLevel::CRITICAL: return ANSI_BRIGHT_RED;
        default:                     return ANSI_WHITE;
    }
}

std::string USB_SerialLogs::get_timestamp_string(uint32_t timestamp) const {
    uint32_t seconds = timestamp / 1000;
    uint32_t milliseconds = timestamp % 1000;
    uint32_t minutes = seconds / 60;
    seconds = seconds % 60;
    uint32_t hours = minutes / 60;
    minutes = minutes % 60;
    
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu.%03lu", 
             hours, minutes, seconds, milliseconds);
    
    return std::string(buffer);
}

void USB_SerialLogs::add_to_queue(const USB_LogEntry& entry) {
    // 计算所需空间：头部 + 标签 + 消息
    size_t tag_len = entry.tag.length();
    size_t msg_len = entry.message.length();
    size_t total_size = sizeof(USB_LogEntryHeader) + tag_len + msg_len;
    
    // 检查剩余容量，如果不足则直接丢弃
    if (total_size > log_buffer_.get_free_space()) {
        stats_.dropped_logs++;
        return;  // 直接丢弃，不加入缓冲区
    }
    
    // 准备头部数据
    USB_LogEntryHeader header;
    header.size = static_cast<uint16_t>(total_size);
    header.timestamp = entry.timestamp;
    header.level = entry.level;
    header.tag_length = static_cast<uint8_t>(std::min(tag_len, size_t(255)));
    header.message_length = static_cast<uint8_t>(msg_len & 0xFF);
    header.full_message_length = static_cast<uint16_t>(std::min(msg_len, size_t(65535)));
    
    // 写入头部
    size_t write_pos = log_buffer_.write_index;
    for (size_t i = 0; i < sizeof(USB_LogEntryHeader); i++) {
        log_buffer_.buffer[write_pos] = reinterpret_cast<const uint8_t*>(&header)[i];
        write_pos = (write_pos + 1) % USB_LOGS_BUFFER_SIZE;
    }
    
    // 写入标签
    for (size_t i = 0; i < tag_len; i++) {
        log_buffer_.buffer[write_pos] = static_cast<uint8_t>(entry.tag[i]);
        write_pos = (write_pos + 1) % USB_LOGS_BUFFER_SIZE;
    }
    
    // 写入消息
    for (size_t i = 0; i < msg_len; i++) {
        log_buffer_.buffer[write_pos] = static_cast<uint8_t>(entry.message[i]);
        write_pos = (write_pos + 1) % USB_LOGS_BUFFER_SIZE;
    }
    
    // 更新写入指针和已使用字节数
    log_buffer_.write_index = write_pos;
    log_buffer_.used_bytes += total_size;
}

void USB_SerialLogs::update_statistics(USB_LogLevel level) {
    stats_.total_logs++;
    
    switch (level) {
        case USB_LogLevel::DEBUG:
            stats_.debug_count++;
            break;
        case USB_LogLevel::INFO:
            stats_.info_count++;
            break;
        case USB_LogLevel::WARNING:
            stats_.warning_count++;
            break;
        case USB_LogLevel::ERROR:
            stats_.error_count++;
            break;
        case USB_LogLevel::CRITICAL:
            stats_.critical_count++;
            break;
    }
}

void USB_SerialLogs::handle_error(const std::string& error_msg) {
    if (error_callback_) {
        error_callback_(error_msg);
    }
}

std::string USB_SerialLogs::format_string(const char* format, va_list args) {
    char buffer[USB_LOGS_MAX_LINE_LENGTH];
    int result = vsnprintf(buffer, sizeof(buffer), format, args);
    
    if (result < 0) {
        return "[FORMAT ERROR]";
    } else if (result >= sizeof(buffer)) {
        return std::string(buffer) + "[TRUNCATED]";
    } else {
        return std::string(buffer);
    }
}