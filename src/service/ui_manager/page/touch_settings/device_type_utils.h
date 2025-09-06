#pragma once

#include "../../../input_manager/input_manager.h"
#include <string>
#include <cstdio>

namespace ui {

/**
 * 设备类型判断和页面跳转工具类
 * 提供统一的设备类型判断逻辑，避免重复代码
 */
class DeviceTypeUtils {
public:
    /**
     * 根据设备类型获取对应的页面名称和跳转字符串
     * @param device 设备状态信息
     * @param page_name 输出的页面名称
     * @param jump_str 输出的跳转字符串
     * @param menu_text 输出的菜单文本
     */
    static inline void getDevicePageInfo(const InputManager::TouchDeviceStatus& device, 
                                        std::string& page_name, 
                                        std::string& jump_str, 
                                        std::string& menu_text) {
        switch (device.device_type) {
            case TouchSensorType::AD7147:
                if (device.is_connected) {
                    page_name = "ad7147_custom_settings";
                    jump_str = device.device_name;
                    char custom_menu_text[64];
                    snprintf(custom_menu_text, sizeof(custom_menu_text), "%s AD7147设置", device.device_name.c_str());
                    menu_text = custom_menu_text;
                } else {
                    page_name = "sensitivity_device";
                    jump_str = device.device_name;
                    menu_text = device.device_name;
                }
                break;
                
            default:
                page_name = "sensitivity_device";
                jump_str = device.device_name;
                menu_text = device.device_name;
                break;
        }
    }
    
    /**
     * 根据设备类型获取对应的页面名称和跳转字符串（简化版本）
     * @param device 设备状态信息
     * @param page_name 输出的页面名称
     * @param jump_str 输出的跳转字符串
     */
    static inline void getDevicePageInfo(const InputManager::TouchDeviceStatus& device, 
                                        std::string& page_name, 
                                        std::string& jump_str) {
        std::string menu_text; // 临时变量，不使用
        getDevicePageInfo(device, page_name, jump_str, menu_text);
    }
    
    /**
     * 根据设备类型和通道获取跳转到具体通道设置的页面信息
     * @param device 设备状态信息
     * @param channel 通道号
     * @param page_name 输出的页面名称
     * @param jump_str 输出的跳转字符串
     */
    static inline void getChannelPageInfo(const InputManager::TouchDeviceStatus& device, 
                                         uint8_t channel,
                                         std::string& page_name, 
                                         std::string& jump_str) {
        switch (device.device_type) {
            case TouchSensorType::AD7147:
                if (device.is_connected) {
                    page_name = "ad7147_custom_settings";
                    jump_str = device.device_name;
                } else {
                    page_name = "sensitivity_device";
                    jump_str = device.device_name;
                }
                break;
                
            default:
                page_name = "sensitivity_device";
                jump_str = device.device_name;
                break;
        }
    }
};

} // namespace ui