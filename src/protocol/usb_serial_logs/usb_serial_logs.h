#ifndef USB_SERIAL_LOGS_H
#define USB_SERIAL_LOGS_H

#include "../../hal/usb/hal_usb.h"
#include <string>
#include <functional>
#include <cstdint>
#include <cstring>
#include "pico/time.h"

// 配置宏定义
#define USB_LOGS_MAX_LINE_LENGTH 256
#define USB_LOGS_BUFFER_BLOCKS 128  // 128个日志块
#define USB_LOGS_BLOCK_SIZE (USB_LOGS_MAX_LINE_LENGTH + 64)  // 每块预留最长字节+头部空间

// 日志级别
enum class USB_LogLevel : uint8_t {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

// 日志格式选项
enum class USB_LogFormat : uint8_t {
    SIMPLE = 0,     // 仅消息内容
    TIMESTAMP = 1,  // 时间戳 + 消息
    FULL = 2        // 时间戳 + 级别 + 消息
};

// 日志配置结构
struct USB_SerialLogs_Config {
    USB_LogLevel min_level;       // 最小日志级别
    USB_LogFormat format;         // 日志格式
    bool enable_colors;           // 启用颜色输出
    uint16_t flush_interval_ms;   // 刷新间隔
    bool auto_flush;              // 自动刷新
    
    USB_SerialLogs_Config() 
        : min_level(USB_LogLevel::INFO)
        , format(USB_LogFormat::FULL)
        , enable_colors(true)

        , flush_interval_ms(100)
        , auto_flush(true) {}
};

// 日志条目结构
struct USB_LogEntry {
    uint32_t timestamp;           // 时间戳
    USB_LogLevel level;           // 日志级别
    std::string message;          // 消息内容
    std::string tag;              // 标签
    
    USB_LogEntry() : timestamp(0), level(USB_LogLevel::INFO) {}
    
    USB_LogEntry(USB_LogLevel lvl, const std::string& msg, const std::string& t = "")
        : level(lvl), message(msg), tag(t) {
        timestamp = time_us_32() / 1000;
    }
};

// 日志块结构
struct USB_LogBlock {
    bool valid;                             // 块是否有效
    uint32_t timestamp;                     // 时间戳
    USB_LogLevel level;                     // 日志级别
    uint8_t tag_length;                     // 标签长度
    uint16_t message_length;                // 消息长度
    char data[USB_LOGS_BLOCK_SIZE - 16];    // 数据区域（标签+消息）
    
    USB_LogBlock() : valid(false), timestamp(0), level(USB_LogLevel::INFO), 
                     tag_length(0), message_length(0) {
        memset(data, 0, sizeof(data));
    }
} __attribute__((packed));

// 块级环形缓冲区结构
struct USB_LogRingBuffer {
    USB_LogBlock blocks[USB_LOGS_BUFFER_BLOCKS];  // 块级缓冲区
    volatile size_t write_index;                  // 写入索引
    volatile size_t read_index;                   // 读取索引
    volatile size_t used_blocks;                  // 已使用块数
    
    USB_LogRingBuffer() : write_index(0), read_index(0), used_blocks(0) {}
    
    // 检查是否为空
    bool is_empty() const {
        return used_blocks == 0;
    }
    
    // 检查是否已满
    bool is_full() const {
        return used_blocks >= USB_LOGS_BUFFER_BLOCKS;
    }
    
    // 获取剩余空间
    size_t get_free_blocks() const {
        return USB_LOGS_BUFFER_BLOCKS - used_blocks;
    }
    
    // 获取已使用空间
    size_t get_used_blocks() const {
        return used_blocks;
    }
    
    // 清空缓冲区
    void clear() {
        write_index = 0;
        read_index = 0;
        used_blocks = 0;
        for (size_t i = 0; i < USB_LOGS_BUFFER_BLOCKS; i++) {
            blocks[i].valid = false;
        }
    }
};

// 回调函数类型
using USB_LogCallback = std::function<void(const USB_LogEntry&)>;
using USB_ErrorCallback = std::function<void(const std::string&)>;

// USB串口日志类
class USB_SerialLogs {
public:
    explicit USB_SerialLogs(HAL_USB* usb_hal);
    ~USB_SerialLogs();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 配置管理
    bool set_config(const USB_SerialLogs_Config config);
    bool get_config(USB_SerialLogs_Config& config);
    
    // 日志输出方法
    void log(USB_LogLevel level, const std::string& message, const std::string& tag = "");
    void debug(const std::string& message, const std::string& tag = "");
    void info(const std::string& message, const std::string& tag = "");
    void warning(const std::string& message, const std::string& tag = "");
    void error(const std::string& message, const std::string& tag = "");
    void critical(const std::string& message, const std::string& tag = "");
    
    // 格式化日志输出
    void logf(USB_LogLevel level, const char* format, ...);
    void debugf(const char* format, ...);
    void infof(const char* format, ...);
    void warningf(const char* format, ...);
    void errorf(const char* format, ...);
    void criticalf(const char* format, ...);
    
    // 原始数据输出
    bool write_raw(const uint8_t* data, size_t length);
    bool write_string(const std::string& str);
    bool write_line(const std::string& line);
    
    // 缓冲区管理
    void flush();
    inline void clear_buffer() { log_buffer_.clear(); }
    inline size_t get_buffer_size() const { return log_buffer_.get_used_blocks(); }
    inline bool is_buffer_full() const { return log_buffer_.is_full(); }
    
    // 统计信息
    struct Statistics {
        uint32_t total_logs;
        uint32_t debug_count;
        uint32_t info_count;
        uint32_t warning_count;
        uint32_t error_count;
        uint32_t critical_count;
        uint32_t dropped_logs;
        uint32_t bytes_sent;
        
        Statistics() : total_logs(0), debug_count(0), info_count(0), 
                      warning_count(0), error_count(0), critical_count(0),
                      dropped_logs(0), bytes_sent(0) {}
    };
    
    bool get_statistics(Statistics& stats);
    void reset_statistics();
    
    // 回调设置
    inline void set_log_callback(USB_LogCallback callback) { log_callback_ = callback; }
    inline void set_error_callback(USB_ErrorCallback callback) { error_callback_ = callback; }
    
    // 任务处理
    void task();
    
    // 静态方法 - 全局日志实例
    static inline void set_global_instance(USB_SerialLogs* instance) { global_instance_ = instance; }
    static inline USB_SerialLogs* get_global_instance() { return global_instance_; }
    static void global_log(USB_LogLevel level, const std::string& message, const std::string& tag = "");
    
private:
    HAL_USB* usb_hal_;
    bool initialized_;
    USB_SerialLogs_Config config_;
    Statistics stats_;
    
    // 队列管理
    USB_LogRingBuffer log_buffer_;  // 环形缓冲区
    uint32_t last_flush_time_;
    
    // 回调
    USB_LogCallback log_callback_;
    USB_ErrorCallback error_callback_;
    
    // 静态全局实例
    static USB_SerialLogs* global_instance_;
    
    // 内部方法
    inline bool should_log(USB_LogLevel level) const {
        return static_cast<uint8_t>(level) >= static_cast<uint8_t>(config_.min_level);
    }
    std::string format_log_entry(const USB_LogEntry& entry);
    std::string get_level_string(USB_LogLevel level) const;
    std::string get_level_color(USB_LogLevel level) const;
    std::string get_timestamp_string(uint32_t timestamp) const;
    
    void add_to_queue(const USB_LogEntry& entry);
    
    void update_statistics(USB_LogLevel level);
    void handle_error(const std::string& error_msg);
    
    // 格式化辅助方法
    std::string format_string(const char* format, va_list args);
};

// 便利宏定义
#define USB_LOG_DEBUG(msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) logger->debugf(msg, ##__VA_ARGS__); \
} while(0)

#define USB_LOG_INFO(msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) logger->infof(msg, ##__VA_ARGS__); \
} while(0)

#define USB_LOG_WARNING(msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) logger->warningf(msg, ##__VA_ARGS__); \
} while(0)

#define USB_LOG_ERROR(msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) logger->errorf(msg, ##__VA_ARGS__); \
} while(0)

#define USB_LOG_CRITICAL(msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) logger->criticalf(msg, ##__VA_ARGS__); \
} while(0)

// 带标签的日志宏
#define USB_LOG_TAG_DEBUG(tag, msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) { \
        char formatted_msg[256]; \
        snprintf(formatted_msg, sizeof(formatted_msg), msg, ##__VA_ARGS__); \
        logger->debug(formatted_msg, tag); \
    } \
} while(0)

#define USB_LOG_TAG_INFO(tag, msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) { \
        char formatted_msg[256]; \
        snprintf(formatted_msg, sizeof(formatted_msg), msg, ##__VA_ARGS__); \
        logger->info(formatted_msg, tag); \
    } \
} while(0)

#define USB_LOG_TAG_WARNING(tag, msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) { \
        char formatted_msg[256]; \
        snprintf(formatted_msg, sizeof(formatted_msg), msg, ##__VA_ARGS__); \
        logger->warning(formatted_msg, tag); \
    } \
} while(0)

#define USB_LOG_TAG_ERROR(tag, msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) { \
        char formatted_msg[256]; \
        snprintf(formatted_msg, sizeof(formatted_msg), msg, ##__VA_ARGS__); \
        logger->error(formatted_msg, tag); \
    } \
} while(0)

#define USB_LOG_TAG_CRITICAL(tag, msg, ...) do { \
    auto* logger = USB_SerialLogs::get_global_instance(); \
    if (logger) { \
        char formatted_msg[256]; \
        snprintf(formatted_msg, sizeof(formatted_msg), msg, ##__VA_ARGS__); \
        logger->critical(formatted_msg, tag); \
    } \
} while(0)

#endif // USB_SERIAL_LOGS_H