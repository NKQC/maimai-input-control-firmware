#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>
#include <vector>

namespace ui {

class AreaSensitivity : public PageConstructor {
public:
    AreaSensitivity();
    virtual ~AreaSensitivity() = default;

    virtual void render(PageTemplate& page_template) override;

private:
    static std::vector<int32_t> s_area_values_;     // 每个区域的灵敏度缓存 (0-99)
    static std::vector<bool> s_area_bound_;         // 每个区域是否已绑定
    static bool s_initialized_;

    static void init_area_values();                 // 初始化区域灵敏度缓存
    static void on_sensitivity_complete();          // 完成回调：写回所有区域设置

    static std::string get_area_name(uint8_t index);// 生成区域名称（A1-E8）
    static uint32_t get_area_channel_id(uint8_t index); // 获取区域绑定的物理通道ID（未绑定返回0xFFFFFFFF）
};

} // namespace ui