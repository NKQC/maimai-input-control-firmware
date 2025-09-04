#include "touch_ic_status.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include <cstdio>
#include <algorithm>

namespace ui {

TouchICStatus::TouchICStatus() : target_device_name_("") {
    // 构造函数无需特殊初始化
}

void TouchICStatus::render(PageTemplate& page_template) {
    PAGE_START()
    SET_TITLE("触摸IC状态", COLOR_WHITE)
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count = 0;
    input_manager->get_all_device_status(device_status, device_count);
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    if (device_count == 0) {
        ADD_TEXT("未检测到触摸IC设备", COLOR_YELLOW, LineAlign::CENTER)
    } else {
        // 如果指定了目标设备，只显示该设备
        if (!target_device_name_.empty()) {
            bool found = false;
            for (int i = 0; i < device_count; i++) {
                const auto& device = device_status[i];
                if (device.device_name == target_device_name_) {
                    // 设备名称和地址行
                    std::string device_info = device.device_name + ": " + format_device_address(device.touch_device.device_id_mask);
                    Color device_color = device.is_connected ? COLOR_TEXT_WHITE : COLOR_RED;
                    ADD_TEXT(device_info, device_color, LineAlign::LEFT)
                    
                    // 触摸bitmap行
                    std::string bitmap_line = format_touch_bitmap(device.touch_states_32bit, device.touch_device.max_channels, device.touch_device.enabled_channels_mask);
                    ADD_TEXT(bitmap_line, COLOR_TEXT_WHITE, LineAlign::LEFT)
                    
                    found = true;
                    break;
                }
            }
            if (!found) {
                ADD_TEXT("指定设备未找到: " + target_device_name_, COLOR_RED, LineAlign::CENTER)
            }
        } else {
            ADD_TEXT("未指定设备", COLOR_RED, LineAlign::CENTER)
        }
    }
    
    PAGE_END()
}

std::string TouchICStatus::format_device_address(uint32_t device_id_mask) {
    // 提取IC ID (高8位)
    uint8_t ic_id = (device_id_mask >> 24) & 0xFF;
    
    // 格式化为十六进制地址
    char addr_str[16];
    snprintf(addr_str, sizeof(addr_str), "0x%02X", ic_id);
    
    return std::string(addr_str);
}

std::string TouchICStatus::format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask) {
    std::string bitmap;
    bitmap.reserve(max_channels + 1);
    
    // 限制最大通道数为24
    uint8_t channels = std::min(max_channels, (uint8_t)24);
    
    for (uint8_t i = 0; i < channels; i++) {
        bool channel_enabled = (enabled_channels_mask & (1UL << i)) != 0;
        if (!channel_enabled) {
            // 跳过的通道显示为'-'
            bitmap += '-';
        } else {
            // 启用的通道显示触摸状态
            bool touched = (touch_mask & (1UL << i)) != 0;
            bitmap += touched ? '1' : '0';
        }
    }
    
    return bitmap;
}

void TouchICStatus::jump_str(const std::string& str) {
    // 接收通过ADD_MENU_WITH_STR传递的设备名称
    target_device_name_ = str;
}

} // namespace ui