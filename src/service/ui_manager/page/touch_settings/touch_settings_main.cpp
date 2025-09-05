#include "touch_settings_main.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"

namespace ui {

int32_t TouchSettingsMain::delay_value = 0;

TouchSettingsMain::TouchSettingsMain() {
    // 构造函数无需特殊初始化
}

void TouchSettingsMain::render(PageTemplate& page_template) {
    // 触摸设置主页面 - 使用页面构造宏
    PAGE_START()
    SET_TITLE("触摸设置", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    // 灵敏度调整菜单项
    ADD_MENU("灵敏度调整", "sensitivity_main", COLOR_TEXT_WHITE)

    // 获取InputManager实例并显示各设备状态
    InputManager* input_manager = InputManager::getInstance();
    
    // Serial模式延迟设置（仅在Serial模式下显示）
    if (input_manager) {
        InputManager_PrivateConfig config = input_manager->getConfig();
        if (config.work_mode == InputWorkMode::SERIAL_MODE) {
            if (!delay_value) {
                delay_value = static_cast<int32_t>(input_manager->getTouchResponseDelay()); // 更新当前值
            }
            ADD_INT_SETTING(&delay_value, 0, 100, 
                          "延迟设置: " + std::to_string(delay_value) + "ms", 
                          "触摸响应延迟",
                          nullptr,
                          []() {
                              // 完成回调
                              auto* input_mgr = InputManager::getInstance();
                              if (input_mgr) {
                                  input_mgr->setTouchResponseDelay(static_cast<uint8_t>(delay_value));
                            }},
                          COLOR_TEXT_WHITE);
        }
    }

    if (input_manager) {
        int device_count = input_manager->get_device_count();
        InputManager::TouchDeviceStatus device_status[device_count];
        input_manager->get_all_device_status(device_status);
        
        if (device_count > 0) {
            std::string device_info = "检测到 " + std::to_string(device_count) + " 个触摸设备:";
            ADD_TEXT(device_info, COLOR_TEXT_WHITE, LineAlign::LEFT)
            for (int i = 0; i < device_count; i++) {
                const auto& device = device_status[i];
                // 设备名称和地址行
                device_info = device.device_name + ": " + format_device_address(device.touch_device.device_id_mask);
                Color device_color = device.is_connected ? COLOR_TEXT_WHITE : COLOR_RED;
                ADD_TEXT(device_info, device_color, LineAlign::LEFT)
                
                // 触摸bitmap行
                std::string bitmap_line = format_touch_bitmap(device.touch_states_32bit, device.touch_device.max_channels, device.touch_device.enabled_channels_mask);
                ADD_TEXT(bitmap_line, COLOR_TEXT_WHITE, LineAlign::LEFT)
            }
        } else {
            ADD_TEXT("未检测到触摸IC设备", COLOR_YELLOW, LineAlign::CENTER)
        }
    }
    PAGE_END()
}

std::string TouchSettingsMain::format_device_address(uint32_t device_id_mask) {
    // 提取IC ID (高8位)
    uint8_t ic_id = (device_id_mask >> 24) & 0xFF;
    
    // 格式化为十六进制地址
    char addr_str[16];
    snprintf(addr_str, sizeof(addr_str), "0x%02X", ic_id);
    
    return std::string(addr_str);
}

std::string TouchSettingsMain::format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask) {
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

} // namespace ui