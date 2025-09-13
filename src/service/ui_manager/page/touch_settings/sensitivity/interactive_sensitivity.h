#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 交互式灵敏度调整页面构造器
 * 触摸检测后自动调整该区域灵敏度
 */
class InteractiveSensitivity : public PageConstructor {
public:
    InteractiveSensitivity();
    virtual ~InteractiveSensitivity() = default;
    
    /**
     * 渲染交互式灵敏度调整页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    /**
     * 交互式调整状态枚举
     */
    enum class InteractiveState {
        WAITING_TOUCH,    // 等待触摸
        TOUCH_DETECTED,   // 检测到触摸
        ADJUSTING,        // 正在调整
        COMPLETED,        // 调整完成
        ERROR             // 错误状态
    };
    
    /**
     * 获取当前交互状态
     * @return 当前状态
     */
    InteractiveState get_current_state();
    
    /**
     * 检测触摸区域
     * @param detected_device 输出检测到的设备名称
     * @param detected_channel 输出检测到的通道
     * @return 是否检测到触摸
     */
    bool detect_touch_area(std::string& detected_device, uint8_t& detected_channel);
    
    /**
     * 执行灵敏度调整
     * @param device_name 设备名称
     * @param channel 通道号
     * @param new_sensitivity 新的灵敏度值
     * @return 是否调整成功
     */
    bool adjust_sensitivity(const std::string& device_name, uint8_t channel, uint8_t new_sensitivity);
    
    /**
     * 格式化状态显示文本
     * @param state 当前状态
     * @return 状态文本
     */
    std::string format_state_text(InteractiveState state);
    
    /**
     * 获取建议的灵敏度值
     * @param device_name 设备名称
     * @param channel 通道号
     * @return 建议的灵敏度值
     */
    uint8_t get_suggested_sensitivity(const std::string& device_name, uint8_t channel);
    
    /**
     * 按钮回调函数
     */
    static void on_jump_to_channel_settings();
    static void on_retry_detect();
    static void on_continue_adjust();
    static void on_retry_adjust();
    
    // 静态成员用于保持状态
    static InteractiveState s_current_state;
    static std::string s_detected_device;
    static uint8_t s_detected_channel;
    static uint8_t s_suggested_sensitivity;
    static uint32_t s_last_update_time;
};

} // namespace ui