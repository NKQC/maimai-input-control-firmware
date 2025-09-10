#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 已绑区信息显示页面构造器
 * 显示A1-E8所有区域的绑定情况和通道ID
 */
class BindingInfo : public PageConstructor {
public:
    BindingInfo();
    virtual ~BindingInfo() = default;
    
    /**
     * 渲染已绑区信息页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    /**
     * 获取Mai2区域名称
     * @param area Mai2触摸区域枚举
     * @return 区域名称字符串
     */
    std::string get_area_name(Mai2_TouchArea area);
    
    /**
     * 格式化通道ID为HEX字符串
     * @param channel_id 32位通道ID
     * @return HEX格式字符串
     */
    std::string format_channel_hex(uint32_t channel_id);
    
    /**
     * 检查区域是否已绑定
     * @param area Mai2触摸区域枚举
     * @return 是否已绑定
     */
    bool is_area_bound(Mai2_TouchArea area);
    
    /**
     * 获取区域绑定的通道ID
     * @param area Mai2触摸区域枚举
     * @return 通道ID，未绑定返回0xFFFFFFFF
     */
    uint32_t get_area_channel_id(Mai2_TouchArea area);
};

} // namespace ui