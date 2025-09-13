#include "touch_settings_main.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"

namespace ui {

// 灵敏度选项实现
const char* getSensitivityOptionText(SensitivityOption option, bool include_unchanged) {
    switch (option) {
        case SensitivityOption::UNCHANGED: return include_unchanged ? "不变" : "未知";
        case SensitivityOption::LOW: return "低敏";
        case SensitivityOption::DEFAULT: return "默认";
        case SensitivityOption::HIGH: return "高敏";
        case SensitivityOption::ULTRA: return "超敏";
        default: return "未知";
    }
}

const char* const* getSensitivityOptions() {
    static const char* options[] = {"低敏", "默认", "高敏", "超敏"};
    return options;
}

size_t getSensitivityOptionsCount() {
    return 4;
}

int32_t TouchSettingsMain::delay_value = 0;

// 校准相关静态变量初始化
uint8_t TouchSettingsMain::progress = 0;
uint8_t TouchSettingsMain::sensitivity_target = 2;  // 默认灵敏度
bool TouchSettingsMain::calibration_in_progress_ = false;

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

    // 触摸状态查看菜单项
    ADD_MENU("查看触摸状态", "touch_status", COLOR_TEXT_WHITE)
    calibration_in_progress_ = input_manager->isCalibrationInProgress();
    if (calibration_in_progress_) {
        // 校准进行中，显示进度条
        InputManager* input_manager = InputManager::getInstance();
        progress = input_manager->getCalibrationProgress();
        if (progress == 255) {
            // 校准完成
            ADD_TEXT("校准完成", COLOR_GREEN, LineAlign::CENTER)
        } else {
            // 显示进度条
            ADD_TEXT("校准进度", COLOR_YELLOW, LineAlign::CENTER)
            ADD_PROGRESS(&progress, COLOR_YELLOW)
        }
    } else {
        // 校准灵敏度目标选择器
        static char sensitivity_text[32];
        SensitivityOption current_option = static_cast<SensitivityOption>(TouchSettingsMain::sensitivity_target);
        snprintf(sensitivity_text, sizeof(sensitivity_text), "校准灵敏度: %s", getSensitivityOptionText(current_option));
        ADD_SIMPLE_SELECTOR(sensitivity_text, [](JoystickState state) {
            if (state == JoystickState::UP && TouchSettingsMain::sensitivity_target < static_cast<uint8_t>(SensitivityOption::ULTRA)) {
                TouchSettingsMain::sensitivity_target++;
            } else if (state == JoystickState::DOWN && TouchSettingsMain::sensitivity_target > static_cast<uint8_t>(SensitivityOption::LOW)) {
                TouchSettingsMain::sensitivity_target--;
            }
        }, COLOR_TEXT_WHITE)
        
        // 未开始校准，显示校准按钮
        ADD_BUTTON("校准全部传感器", onCalibrateButtonPressed, COLOR_TEXT_WHITE, LineAlign::CENTER)
    }

    // 灵敏度调整菜单项
    ADD_MENU("按模块调整灵敏度", "sensitivity_main", COLOR_TEXT_WHITE)
    
    // 按分区设置灵敏度选项（仅Serial模式）
    if (input_manager->getWorkMode() == InputWorkMode::SERIAL_MODE) {
        ADD_MENU("按分区设置灵敏度", "zone_sensitivity", COLOR_TEXT_WHITE)
    }

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
        // 清空状态
    }
}


} // namespace ui