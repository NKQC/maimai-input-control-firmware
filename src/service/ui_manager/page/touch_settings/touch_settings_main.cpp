#include "touch_settings_main.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"
#include <vector>
#include <cstdio>

namespace ui {

// 内联数字到字符串转换函数
inline void int8_to_string(int8_t value, char* buffer) {
    char* ptr = buffer;
    
    if (value > 0) {
        *ptr++ = '+';
    } else if (value < 0) {
        *ptr++ = '-';
        value = -value;
    }
    
    if (value >= 10) {
        *ptr++ = '0' + (value / 10);
    }
    *ptr++ = '0' + (value % 10);
    *ptr = '\0';
}

// 灵敏度选项实现
const char* getSensitivityOptionText(SensitivityOption option, bool include_unchanged) {
    static char buffer[8]; // 静态缓冲区用于存储格式化的数值字符串
    
    int8_t value = static_cast<int8_t>(option);
    
    // 确保值在有效范围内
    if (value < SENSITIVITY_MIN || value > SENSITIVITY_MAX) {
        return "未知";
    }
    
    // 使用内联转换函数
    int8_to_string(value, buffer);
    
    return buffer;
}

// 获取所有可用的灵敏度选项（-10到+10，不包括UNCHANGED）
const int8_t* getSensitivityValues() {
    static int8_t values[SENSITIVITY_MAX - SENSITIVITY_MIN + 1];
    static bool initialized = false;
    
    if (!initialized) {
        for (int8_t i = SENSITIVITY_MIN; i <= SENSITIVITY_MAX; ++i) {
            values[i - SENSITIVITY_MIN] = i;
        }
        initialized = true;
    }
    
    return values;
}

size_t getSensitivityOptionsCount() {
    return SENSITIVITY_MAX - SENSITIVITY_MIN + 1; // -10到+10共21个选项
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
    
    // 检查是否存在可校准的传感器
    bool has_calibratable_sensors = input_manager->hasCalibratableSensors();
    
    if (has_calibratable_sensors) {
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
                if (state == JoystickState::UP && TouchSettingsMain::sensitivity_target < SENSITIVITY_MAX) {
                    TouchSettingsMain::sensitivity_target++;
                } else if (state == JoystickState::DOWN && TouchSettingsMain::sensitivity_target > SENSITIVITY_MIN) {
                    TouchSettingsMain::sensitivity_target--;
                }
            }, COLOR_TEXT_WHITE)
            
            // 未开始校准，显示校准按钮
            ADD_BUTTON("校准全部传感器", onCalibrateButtonPressed, COLOR_TEXT_WHITE, LineAlign::CENTER)
            ADD_MENU("按分区校准灵敏度", "zone_sensitivity", COLOR_TEXT_WHITE)
        }
    }

    // 按分区设置灵敏度选项（仅Serial模式）
    if (input_manager->getWorkMode() == InputWorkMode::SERIAL_MODE) {
        ADD_MENU("按区域调整灵敏度", "area_sensitivity", COLOR_TEXT_WHITE)
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
        // 清空状态
    }
}


} // namespace ui