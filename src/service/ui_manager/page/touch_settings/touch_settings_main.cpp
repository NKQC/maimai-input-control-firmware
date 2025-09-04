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

    // 获取InputManager实例并显示各设备状态
    InputManager* input_manager = InputManager::getInstance();
    if (input_manager) {
        InputManager::TouchDeviceStatus device_status[8];
        int device_count = 0;
        input_manager->get_all_device_status(device_status, device_count);
        
        if (device_count > 0) {
            ADD_TEXT("各设备状态:", COLOR_TEXT_WHITE, LineAlign::LEFT)
            for (int i = 0; i < device_count; i++) {
                const auto& device = device_status[i];
                std::string menu_text = device.device_name + " 状态";
                ADD_MENU_WITH_STR(menu_text, "touch_ic_status", device.device_name, COLOR_TEXT_WHITE)
            }
        }
    }
    
    // 灵敏度调整菜单项
    ADD_MENU("灵敏度调整", "sensitivity_main", COLOR_TEXT_WHITE)
    
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
    
    PAGE_END()
}

} // namespace ui