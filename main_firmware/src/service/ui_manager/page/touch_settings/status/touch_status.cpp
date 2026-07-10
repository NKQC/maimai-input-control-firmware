#include "touch_status.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../input_manager/input_manager.h"

namespace ui {

TouchStatus::TouchStatus() {
    // 构造函数无需特殊初始化
}

void TouchStatus::render(PageTemplate& page_template) {
    // 获取InputManager实例并显示各设备状态
    InputManager* input_manager = InputManager::getInstance();

    PAGE_START()
    SET_TITLE("触摸状态", COLOR_WHITE)

    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    if (input_manager) {
        int device_count = input_manager->get_device_count();
        InputManager::TouchDeviceStatus device_status[device_count];
        input_manager->get_all_device_status(device_status);
        
        if (device_count > 0) {
            for (int i = 0; i < device_count; i++) {
                const auto& device = device_status[i];
                
                // 直接从设备实例获取enabled_channels_mask
                uint32_t enabled_channels_mask = 0;
                const auto& devices = input_manager->getTouchSensorDevices();
                for (TouchSensor* sensor : devices) {
                    if (sensor && sensor->getModuleMask() == device.touch_device.device_id_mask) {
                        enabled_channels_mask = sensor->getEnabledChannelMask();
                        break;
                    }
                }
                
                // 设备名称和bitmap在同一行
                std::string bitmap = format_touch_bitmap(device.touch_states_32bit, device.touch_device.max_channels, enabled_channels_mask);
                std::string device_line = device.device_name + " " + bitmap;
                Color device_color = device.is_connected ? COLOR_TEXT_WHITE : COLOR_RED;
                ADD_TEXT(device_line, device_color, LineAlign::LEFT)
            }
        } else {
            ADD_TEXT("未检测到触摸IC设备", COLOR_YELLOW, LineAlign::CENTER)
        }
    } else {
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
    }

    PAGE_END()
}



std::string TouchStatus::format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask) {
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