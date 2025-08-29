#include "status_page.h"
#include "../engine/graphics_engine.h"
#include "../engine/font_system.h"

namespace ui {

StatusPage::StatusPage(GraphicsEngine* graphics_engine) : PageTemplate(graphics_engine) {
    // 构造函数实现
}

StatusPage::~StatusPage() {
    // 析构函数实现
}

bool StatusPage::init() {
    // 初始化页面
    return true;
}

void StatusPage::deinit() {
    // 清理页面资源
}

void StatusPage::draw(GraphicsEngine* graphics) {
    if (!graphics) return;
    
    // 绘制状态页面
    graphics->clear();
    graphics->draw_text("系统状态", 10, 10, 0xFFFF);
    
    // 绘制状态信息
    int y_pos = 35;
    
    // 系统运行时间
    char uptime_str[32];
    snprintf(uptime_str, sizeof(uptime_str), "运行时间: %lu秒", uptime_seconds_);
    graphics->draw_text(uptime_str, 10, y_pos, 0xFFFF);
    y_pos += 18;
    
    // 内存使用情况
    char memory_str[32];
    snprintf(memory_str, sizeof(memory_str), "内存使用: %d%%", memory_usage_);
    graphics->draw_text(memory_str, 10, y_pos, 0xFFFF);
    y_pos += 18;
    
    // 触摸设备状态
    const char* touch_status = touch_device_connected_ ? "已连接" : "未连接";
    uint16_t touch_color = touch_device_connected_ ? 0x07E0 : 0xF800; // 绿色/红色
    graphics->draw_text("触摸设备:", 10, y_pos, 0xFFFF);
    graphics->draw_text(touch_status, 80, y_pos, touch_color);
    y_pos += 18;
    
    // 灯光设备状态
    const char* light_status = light_device_connected_ ? "已连接" : "未连接";
    uint16_t light_color = light_device_connected_ ? 0x07E0 : 0xF800; // 绿色/红色
    graphics->draw_text("灯光设备:", 10, y_pos, 0xFFFF);
    graphics->draw_text(light_status, 80, y_pos, light_color);
    y_pos += 18;
    
    // 校准状态
    char calibration_str[32];
    snprintf(calibration_str, sizeof(calibration_str), "校准进度: %d%%", calibration_progress_);
    graphics->draw_text(calibration_str, 10, y_pos, 0xFFFF);
    y_pos += 18;
    
    // 错误计数
    char error_str[32];
    snprintf(error_str, sizeof(error_str), "错误计数: %lu", error_count_);
    uint16_t error_color = (error_count_ > 0) ? 0xF800 : 0x07E0; // 红色/绿色
    graphics->draw_text(error_str, 10, y_pos, error_color);
    y_pos += 25;
    
    // 返回按钮提示
    graphics->draw_text("按确认键返回", 10, y_pos, 0x07E0);
}

void StatusPage::update() {
    // 更新状态信息
    uptime_seconds_++;
    
    // 这里可以添加更多状态更新逻辑
    // 例如检查设备连接状态、内存使用等
}


void StatusPage::set_uptime(uint32_t seconds) {
    uptime_seconds_ = seconds;
}

void StatusPage::set_memory_usage(int percentage) {
    if (percentage >= 0 && percentage <= 100) {
        memory_usage_ = percentage;
    }
}

void StatusPage::set_touch_device_status(bool connected) {
    touch_device_connected_ = connected;
}

void StatusPage::set_light_device_status(bool connected) {
    light_device_connected_ = connected;
}

void StatusPage::set_calibration_progress(int progress) {
    if (progress >= 0 && progress <= 100) {
        calibration_progress_ = progress;
    }
}

void StatusPage::set_error_count(uint32_t count) {
    error_count_ = count;
}

} // namespace ui