#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include <cstdint>

namespace ui {

/**
 * 触摸设置主页面构造器
 * 包含触摸IC状态和灵敏度调整菜单项
 */
class TouchSettingsMain : public PageConstructor {
public:
    TouchSettingsMain();
    virtual ~TouchSettingsMain() = default;
    
    /**
     * 渲染触摸设置主页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    static int32_t delay_value;

    /**
     * 格式化触摸IC地址为字符串
     * @param device_id_mask 触摸IC设备ID掩码
     * @return 格式化后的地址字符串
     */
    static std::string format_device_address(uint32_t device_id_mask);

    /**
     * 格式化触摸通道位图为字符串
     * @param touch_mask 触摸通道掩码
     * @param max_channels 最大通道数
     * @param enabled_channels_mask 已启用通道掩码
     * @return 格式化后的通道位图字符串
     */
    static std::string format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask);
};

} // namespace ui