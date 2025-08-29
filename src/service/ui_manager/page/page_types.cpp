#include "page_types.h"

namespace ui {

std::string page_id_to_string(UIPage page_id) {
    switch (page_id) {
        case UIPage::MAIN:
            return "main_menu";
        case UIPage::STATUS:
            return "status";
        case UIPage::SETTINGS:
            return "settings";
        case UIPage::CALIBRATION:
            return "calibration";
        case UIPage::DIAGNOSTICS:
            return "diagnostics";
        case UIPage::SENSITIVITY:
            return "sensitivity";
        case UIPage::TOUCH_MAPPING:
            return "touch_mapping";
        case UIPage::KEY_MAPPING:
            return "key_mapping";
        case UIPage::GUIDED_BINDING:
            return "guided_binding";
        case UIPage::LIGHT_MAPPING:
            return "light_mapping";
        case UIPage::UART_SETTINGS:
            return "uart_settings";
        case UIPage::ERROR:
            return "error";
        case UIPage::ABOUT:
            return "about";
        default:
            return "unknown";
    }
}

std::string error_type_to_string(ErrorType type) {
    switch (type) {
        case ErrorType::NONE:
            return "无故障";
        case ErrorType::HARDWARE_INIT:
            return "硬件初始化失败";
        case ErrorType::DISPLAY_ERROR:
            return "显示设备故障";
        case ErrorType::INPUT_ERROR:
            return "输入设备故障";
        case ErrorType::LIGHT_ERROR:
            return "灯光设备故障";
        case ErrorType::CONFIG_ERROR:
            return "配置错误";
        case ErrorType::COMMUNICATION_ERROR:
            return "通信错误";
        case ErrorType::MEMORY_ERROR:
            return "内存错误";
        case ErrorType::SENSOR_ERROR:
            return "传感器错误";
        case ErrorType::CALIBRATION_ERROR:
            return "校准错误";
        case ErrorType::UNKNOWN_ERROR:
            return "未知错误";
        default:
            return "未定义错误";
    }
}

} // namespace ui