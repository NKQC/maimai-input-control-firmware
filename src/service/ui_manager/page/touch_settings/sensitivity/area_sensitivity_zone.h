#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 区域组灵敏度设置页面构造器
 * 显示特定区域组(A/B/C/D/E)中的各个区域
 */
class AreaSensitivityZone : public PageConstructor {
public:
    AreaSensitivityZone();
    virtual ~AreaSensitivityZone() = default;
    
    /**
     * 渲染区域组灵敏度设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 处理字符串参数传递
     */
    void jump_str(const std::string& str) override;

private:
    // 当前区域组索引
    uint8_t current_zone_index_;
    
    /**
     * 获取区域名称
     * @param area_index 区域索引 (1-34)
     * @return 区域名称字符串
     */
    std::string getAreaName(uint8_t area_index);
    
    /**
     * 获取区域组名称
     * @param zone_index 区域组索引 (0-4)
     * @return 区域组名称字符串
     */
    std::string getZoneName(uint8_t zone_index);
};

} // namespace ui