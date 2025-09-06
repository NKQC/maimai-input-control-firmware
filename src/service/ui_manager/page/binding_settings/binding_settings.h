#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 绑区设置页面构造器
 * 支持启动/终止交互式绑区，显示进度和确认保存
 */
class BindingSettings : public PageConstructor {
public:
    BindingSettings();
    virtual ~BindingSettings() = default;
    
    /**
     * 渲染绑区设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    /**
     * 绑区UI状态枚举
     */
    enum class BindingUIState {
        IDLE,              // 空闲状态
        BINDING_ACTIVE,    // 绑区进行中
        BINDING_COMPLETE,  // 绑区完成，等待确认
        BINDING_ERROR      // 绑区错误
    };
    
    /**
     * 获取当前绑区状态
     * @return 当前绑区状态
     */
    BindingUIState get_current_binding_state();
    
    /**
     * 获取绑区进度百分比
     * @return 进度百分比 (0-100)
     */
    uint8_t get_binding_progress();
    
    /**
     * 获取当前绑定的区域名称
     * @return 区域名称
     */
    std::string get_current_binding_area();
    
    /**
     * 启动交互式绑区
     * @return 是否成功启动
     */
    static bool start_serial_binding();
    
    /**
     * 终止当前绑区操作
     * @return 是否成功终止
     */
    static bool stop_binding();
    
    /**
     * 确认并保存绑区设置
     * @return 是否成功保存
     */
    static bool confirm_and_save_binding();
    
    /**
     * 回退绑区步骤
     * @return 是否成功回退
     */
    static bool step_back_binding();
    
    /**
     * 格式化绑区状态文本
     * @param state 绑区状态
     * @return 状态文本
     */
    std::string format_binding_state_text(BindingUIState state);
    
    /**
     * 获取Mai2区域名称
     * @param area_index 区域索引
     * @return 区域名称
     */
    std::string get_mai2_area_name(uint8_t area_index);
    
    // 静态成员用于保持UI状态
    static BindingUIState s_ui_state;
    static uint32_t s_last_update_time;
};

} // namespace ui