#include "touch_settings_main.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"

namespace ui {

int32_t TouchSettingsMain::delay_value = 0;

// 校准相关静态变量初始化
uint8_t TouchSettingsMain::progress = 0;
uint8_t TouchSettingsMain::sensitivity_target = 2;  // 默认灵敏度
bool TouchSettingsMain::calibration_in_progress_ = false;
bool TouchSettingsMain::calibration_completed_ = false;

TouchSettingsMain::TouchSettingsMain() {
    // 构造函数无需特殊初始化
}

void TouchSettingsMain::render(PageTemplate& page_template) {
    // 获取InputManager实例并显示各设备状态
    InputManager* input_manager = InputManager::getInstance();

    PAGE_START()
    SET_TITLE("触摸设置", COLOR_WHITE)

    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    if (input_manager) {
        int device_count = input_manager->get_device_count();
        InputManager::TouchDeviceStatus device_status[device_count];
        input_manager->get_all_device_status(device_status);
        
        if (device_count > 0) {
            std::string device_info = "共 " + std::to_string(device_count) + " 个触摸设备:";
            ADD_TEXT(device_info, COLOR_TEXT_WHITE, LineAlign::CENTER)
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
    
    if (calibration_in_progress_) {
        // 校准进行中，显示进度条
        InputManager* input_manager = InputManager::getInstance();
        progress = input_manager->getCalibrationProgress();
        if (progress == 255) {
            // 校准完成
            calibration_in_progress_ = false;
            calibration_completed_ = true;
        } else {
            // 显示进度条
            ADD_TEXT("校准进度", COLOR_YELLOW, LineAlign::CENTER)
            ADD_PROGRESS(&progress, COLOR_YELLOW)
        }
    } else if (calibration_completed_) {
        // 校准完成
        ADD_TEXT("校准完成", COLOR_GREEN, LineAlign::CENTER)
    } else {
        // 校准灵敏度目标选择器
        static char sensitivity_text[32];
        const char* sensitivity_options[] = {"低敏", "默认", "高敏", "超敏"};
        snprintf(sensitivity_text, sizeof(sensitivity_text), "校准灵敏度: %s", sensitivity_options[TouchSettingsMain::sensitivity_target - 1]);
        ADD_SIMPLE_SELECTOR(sensitivity_text, [](JoystickState state) {
            if (state == JoystickState::UP && TouchSettingsMain::sensitivity_target < 4) {
                TouchSettingsMain::sensitivity_target++;
            } else if (state == JoystickState::DOWN && TouchSettingsMain::sensitivity_target > 1) {
                TouchSettingsMain::sensitivity_target--;
            }
        }, COLOR_TEXT_WHITE)
        
        // 未开始校准，显示校准按钮
        ADD_BUTTON("校准全部传感器", onCalibrateButtonPressed, COLOR_TEXT_WHITE, LineAlign::CENTER)
    }

    // 灵敏度调整菜单项
    ADD_MENU("按模块调整灵敏度", "sensitivity_main", COLOR_TEXT_WHITE)

    PAGE_END()
}

std::string TouchSettingsMain::format_device_address(uint8_t device_id_mask) {
    // 格式化为十六进制地址
    char addr_str[16];
    snprintf(addr_str, sizeof(addr_str), "0x%02X", device_id_mask);
    
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

void TouchSettingsMain::onCalibrateButtonPressed() {
    // 启动校准
    InputManager* input_manager = InputManager::getInstance();
    if (input_manager) {
        input_manager->calibrateAllSensorsWithTarget(sensitivity_target);
        calibration_in_progress_ = true;
        calibration_completed_ = false;
        // 清空状态
    }
}


} // namespace ui