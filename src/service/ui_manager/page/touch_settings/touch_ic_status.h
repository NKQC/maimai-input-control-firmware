#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 触摸IC状态页面构造器
 * 显示所有注册的IC设备名称、地址和实时触摸bitmap
 */
class TouchICStatus : public PageConstructor {
public:
    TouchICStatus();
    virtual ~TouchICStatus() = default;
    
    /**
     * 渲染触摸IC状态页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 接收跳转时传递的字符串参数（设备名称）
     * 如果提供设备名称，则只显示该设备的状态
     * @param str 设备名称，为空则显示所有设备
     */
    virtual void jump_str(const std::string& str) override;
    
private:
    std::string target_device_name_;  // 通过jump_str接收的目标设备名称
    
    /**
     * 格式化设备地址显示
     * @param device_id_mask 32位设备ID掩码
     * @return 格式化的地址字符串
     */
    std::string format_device_address(uint32_t device_id_mask);
    
    /**
     * 格式化触摸bitmap显示
     * @param touch_mask 触摸状态掩码
     * @param max_channels 最大通道数
     * @param enabled_channels_mask 启用通道掩码
     * @return 格式化的bitmap字符串
     */
    std::string format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask);
};

} // namespace ui