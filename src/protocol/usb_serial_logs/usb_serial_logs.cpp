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
    , buffer_pos_(0)
    , last_flush_time_(0) {
    
    memset(write_buffer_, 0, sizeof(write_buffer_));
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
    
    // 初始化USB HAL（使用现有接口）
    if (!usb_hal_->init()) {
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
        
        usb_hal_->deinit();
        initialized_ = false;
        
        // 清理队列
        while (!log_queue_.empty()) {
            log_queue_.pop();
        }
        
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
bool USB_SerialLogs::set_config(const USB_SerialLogs_Config& config) {
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
    
    if (config_.enable_buffering) {
        add_to_queue(entry);
    } else {
        send_log_entry(entry);
    }
    
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
    
    // 处理队列中的日志
    process_queue();
    
    // 刷新写缓冲区
    if (buffer_pos_ > 0) {
        // 直接发送整个缓冲区，让HAL层处理分块和重试
        if (usb_hal_->cdc_write(write_buffer_, buffer_pos_)) {
            stats_.bytes_sent += buffer_pos_;
            buffer_pos_ = 0;
        }
        // 如果发送失败，保留缓冲区内容，下次再试
    }
    
    last_flush_time_ = time_us_32() / 1000;
}

// 清除缓冲区
void USB_SerialLogs::clear_buffer() {
    while (!log_queue_.empty()) {
        log_queue_.pop();
    }
    buffer_pos_ = 0;
}

// 获取缓冲区大小
size_t USB_SerialLogs::get_buffer_size() const {
    return log_queue_.size();
}

// 检查缓冲区是否满
bool USB_SerialLogs::is_buffer_full() const {
    return log_queue_.size() >= USB_LOGS_QUEUE_SIZE;
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
    if (is_buffer_full()) {
        // 队列满，丢弃最旧的日志
        log_queue_.pop();
        stats_.dropped_logs++;
    }
    
    log_queue_.push(entry);
}

void USB_SerialLogs::process_queue() {
    while (!log_queue_.empty() && is_ready()) {
        const USB_LogEntry& entry = log_queue_.front();
        
        if (send_log_entry(entry)) {
            log_queue_.pop();
        } else {
            // 发送失败，停止处理
            break;
        }
    }
}

bool USB_SerialLogs::send_log_entry(const USB_LogEntry& entry) {
    if (!is_ready()) {
        return false;
    }
    
    std::string formatted = format_log_entry(entry);
    formatted += "\r\n";
    
    // 检查消息长度是否合理
    if (formatted.length() > USB_LOGS_MAX_LINE_LENGTH) {
        // 消息过长，截断处理
        formatted = formatted.substr(0, USB_LOGS_MAX_LINE_LENGTH - 10) + "...\r\n";
    }
    
    // 检查是否需要缓冲
    if (config_.enable_buffering && formatted.length() < USB_LOGS_BUFFER_SIZE - buffer_pos_) {
        // 添加到写缓冲区
        memcpy(write_buffer_ + buffer_pos_, formatted.c_str(), formatted.length());
        buffer_pos_ += formatted.length();
        
        // 如果缓冲区接近满，立即刷新
        if (buffer_pos_ > USB_LOGS_BUFFER_SIZE * 0.8) {
            flush();
        }
        
        return true;
    } else {
        // 直接发送，让HAL层处理超时和重试
        return write_string(formatted);
    }
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