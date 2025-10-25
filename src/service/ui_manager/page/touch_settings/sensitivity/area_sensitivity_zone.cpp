#include "area_sensitivity_zone.h"
#include "area_sensitivity.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdio>

namespace ui {

AreaSensitivityZone::AreaSensitivityZone() : current_zone_index_(0) {
}

std::string AreaSensitivityZone::getAreaName(uint8_t area_index) {
    // 区域名称映射 (1-34对应A1-E8)
    if (area_index < 1 || area_index > 34) {
        return "未知";
    }
    
    uint8_t zone_idx = (area_index - 1) / 8;  // 0-4对应A-E
    uint8_t area_in_zone = (area_index - 1) % 8 + 1;  // 1-8
    
    const char* zone_names[] = {"A", "B", "C", "D", "E"};
    
    static char area_name[8];
    snprintf(area_name, sizeof(area_name), "%s%d", zone_names[zone_idx], area_in_zone);
    return area_name;
}

std::string AreaSensitivityZone::getZoneName(uint8_t zone_index) {
    const char* zone_names[] = {"A", "B", "C", "D", "E"};
    if (zone_index < 5) {
        return zone_names[zone_index];
    }
    return "未知";
}

void AreaSensitivityZone::jump_str(const std::string& str) {
    current_zone_index_ = static_cast<uint8_t>(std::stoi(str));
}

void AreaSensitivityZone::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
    
    // 获取区域数据
    auto* zone_infos = AreaSensitivity::getZoneInfos();
    if (current_zone_index_ >= 5) {
        return;
    }
    
    const auto& zone_info = zone_infos[current_zone_index_];
    
    PAGE_START()
    
    // 设置标题
    static char title[32];
    snprintf(title, sizeof(title), "%s区灵敏度设置", getZoneName(current_zone_index_).c_str());
    SET_TITLE(title, COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    if (!zone_info.has_any_bindings) {
        ADD_TEXT("该区域组无绑定", COLOR_WHITE, LineAlign::CENTER)
    } else {
        ADD_TEXT("选择要调整的区域", COLOR_WHITE, LineAlign::CENTER)
        
        // 收集已绑定的区域并按实际区域索引排序
        struct BoundArea {
            uint8_t area_index;
            const AreaSensitivity::AreaInfo* area_info;
        };
        
        BoundArea bound_areas[8];
        uint8_t bound_count = 0;
        
        // 收集已绑定的区域
        for (uint8_t area_idx = 0; area_idx < 8; area_idx++) {
            const auto& area_info = zone_info.areas[area_idx];
            if (area_info.is_bound) {
                bound_areas[bound_count].area_index = area_info.area_index;
                bound_areas[bound_count].area_info = &area_info;
                bound_count++;
            }
        }
        
        // 按实际区域索引排序（冒泡排序，简单有效）
        for (uint8_t i = 0; i < bound_count - 1; i++) {
            for (uint8_t j = 0; j < bound_count - 1 - i; j++) {
                if (bound_areas[j].area_index > bound_areas[j + 1].area_index) {
                    BoundArea temp = bound_areas[j];
                    bound_areas[j] = bound_areas[j + 1];
                    bound_areas[j + 1] = temp;
                }
            }
        }
        
        // 按排序后的顺序显示区域
        for (uint8_t i = 0; i < bound_count; i++) {
            const auto* area_info = bound_areas[i].area_info;
            
            // 构建显示文本
            static char area_text[64];
            snprintf(area_text, sizeof(area_text), "%s - 当前: %ld", 
                    area_info->name.c_str(), area_info->current_value);
            
            // 使用绿色表示已修改的区域
            uint32_t text_color = area_info->has_modified ? COLOR_TEXT_GREEN : COLOR_TEXT_WHITE;
            
            // 传递区域索引作为参数
            static char area_param[8];
            snprintf(area_param, sizeof(area_param), "%d", area_info->area_index);
            
            ADD_MENU_WITH_STR(area_text, "area_sensitivity_detail", area_param, text_color);
        }
    }
    
    PAGE_END()
}

} // namespace ui