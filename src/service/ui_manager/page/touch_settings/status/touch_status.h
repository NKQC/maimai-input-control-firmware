#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 触摸状态显示页面构造器
 * 显示所有触摸设备的状态信息，每行一个设备
 */
class TouchStatus : public PageConstructor {
public:
    TouchStatus();
    virtual ~TouchStatus() = default;
    
    /**
     * 渲染触摸状态页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:

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