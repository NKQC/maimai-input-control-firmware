#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include <cstdint>

namespace ui {

/**
 * 通用设置页面构造器
 * 包含息屏超时和亮度设置
 */
class GeneralSettings : public PageConstructor {
public:
    GeneralSettings();
    virtual ~GeneralSettings() = default;
    
    /**
     * 渲染通用设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    // 息屏超时设置值（秒）
    static int32_t screen_timeout_seconds_;
    
    // 亮度设置值（0-255）
    static int32_t brightness_value_;
    
    // 亮度百分比数据（用于进度条显示）
    static uint8_t brightness_progress_data_;
    
    /**
     * 更新亮度进度条数据
     */
    void update_brightness_progress();
    
    /**
     * 息屏超时值变化回调
     * @param new_value 新的超时值（秒）
     */
    static void on_screen_timeout_changed(int32_t new_value);
    
    /**
     * 息屏超时值设置完成回调
     */
    static void on_screen_timeout_complete();
    
    /**
     * 亮度值变化回调
     * @param new_value 新的亮度值（0-255）
     */
    static void on_brightness_changed(int32_t new_value);
    
    /**
     * 亮度值设置完成回调
     */
    static void on_brightness_complete();
};

} // namespace ui