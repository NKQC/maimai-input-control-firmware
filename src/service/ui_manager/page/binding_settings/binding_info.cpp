#include "binding_info.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"
#include <cstdio>
#include <cstring>

namespace ui {

BindingInfo::BindingInfo() {
    // 构造函数无需特殊初始化
}

void BindingInfo::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("已绑区信息", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 检查当前工作模式
    auto work_mode = input_manager->getWorkMode();
    if (work_mode != InputWorkMode::SERIAL_MODE) {
        PAGE_START()
        SET_TITLE("已绑区信息", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("绑区功能仅在Serial模式下可用", COLOR_YELLOW, LineAlign::CENTER)
        ADD_TEXT("当前模式: HID模式", COLOR_TEXT_WHITE, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    PAGE_START()
    SET_TITLE("已绑区信息", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 使用分屏模式显示绑区信息，一行显示2个区域
    std::vector<LineConfig> left_lines;
    std::vector<LineConfig> right_lines;
    
    // 遍历所有34个区域 (A1-E8)
    for (int i = 0; i < 34; i++) {
        Mai2_TouchArea area = static_cast<Mai2_TouchArea>(i + 1); // Mai2_TouchArea从1开始
        std::string area_name = get_area_name(area);
        uint32_t channel_id = get_area_channel_id(area);
        
        std::string display_text;
        Color text_color;
        
        if (channel_id != 0xFFFFFFFF) {
            // 已绑定，显示通道ID
            char hex_str[16];
            snprintf(hex_str, sizeof(hex_str), "0x%08lX", channel_id);
            display_text = area_name + ": " + hex_str;
            text_color = COLOR_TEXT_GREEN;
        } else {
            // 未绑定
            display_text = area_name + ": 未绑定";
            text_color = COLOR_ERROR;
        }
        ADD_TEXT(display_text, text_color, LineAlign::LEFT)
    }
    
    PAGE_END()
}

std::string BindingInfo::get_area_name(Mai2_TouchArea area) {
    // Mai2区域名称映射 (A1-A8, B1-B8, C1-C2, D1-D8, E1-E8)
    static const char* area_names[] = {
        "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8",  // 1-8
        "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8",  // 9-16
        "C1", "C2",                                        // 17-18
        "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",  // 19-26
        "E1", "E2", "E3", "E4", "E5", "E6", "E7", "E8"   // 27-34
    };
    
    int area_index = static_cast<int>(area) - 1; // 转换为0-33索引
    if (area_index >= 0 && area_index < 34) {
        return std::string(area_names[area_index]);
    }
    return "未知";
}

std::string BindingInfo::format_channel_hex(uint32_t channel_id) {
    char hex_str[16];
    snprintf(hex_str, sizeof(hex_str), "0x%08lX", channel_id);
    return std::string(hex_str);
}

bool BindingInfo::is_area_bound(Mai2_TouchArea area) {
    return get_area_channel_id(area) != 0xFFFFFFFF;
}

uint32_t BindingInfo::get_area_channel_id(Mai2_TouchArea area) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return 0xFFFFFFFF;
    }
    
    // 获取配置
    auto config = input_manager->getConfig();
    
    // 检查区域映射
    int area_index = static_cast<int>(area) - 1; // 转换为0-33索引
    if (area_index >= 0 && area_index < 34) {
        uint32_t channel = config.area_channel_mappings.serial_mappings[area_index].channel;
        return channel;
    }
    
    return 0xFFFFFFFF;
}

} // namespace ui