#ifndef STATUS_PAGE_H
#define STATUS_PAGE_H

#include "page_template.h"
#include "page_types.h"
#include <cstdint>

class GraphicsEngine;
class UIManager;

namespace ui {

class StatusPage : public PageTemplate {
public:
    StatusPage(GraphicsEngine* graphics_engine);
    virtual ~StatusPage();
    
    // PageTemplate接口实现
    bool init();
    void deinit();
    void draw(GraphicsEngine* graphics);
    void update();
    
    // 状态页面特有方法
    void set_uptime(uint32_t seconds);
    void set_memory_usage(int percentage);
    void set_touch_device_status(bool connected);
    void set_light_device_status(bool connected);
    void set_calibration_progress(int progress);
    void set_error_count(uint32_t count);
    
private:
    uint32_t uptime_seconds_ = 0;           // 系统运行时间
    int memory_usage_ = 0;                  // 内存使用百分比
    bool touch_device_connected_ = false;   // 触摸设备连接状态
    bool light_device_connected_ = false;   // 灯光设备连接状态
    int calibration_progress_ = 0;          // 校准进度
    uint32_t error_count_ = 0;              // 错误计数
};

} // namespace ui

#endif // STATUS_PAGE_H