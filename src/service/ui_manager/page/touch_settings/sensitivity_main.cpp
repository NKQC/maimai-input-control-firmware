#include "sensitivity_main.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include <cstdio>

namespace ui {

SensitivityMain::SensitivityMain() {
    // 构造函数无需特殊初始化
}

void SensitivityMain::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("灵敏度调整", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count = 0;
    input_manager->get_all_device_status(device_status, device_count);
    
    PAGE_START()
    SET_TITLE("灵敏度调整", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 交互式灵敏度调整选项（第一行）
    ADD_MENU("交互式调整", "interactive_sensitivity", COLOR_TEXT_YELLOW)
    
    if (device_count == 0) {
        ADD_TEXT("未检测到触摸IC设备", COLOR_YELLOW, LineAlign::CENTER)
    } else {
        // 显示每个设备的菜单项
        for (int i = 0; i < device_count; i++) {
            const auto& device = device_status[i];
            
            // 设备颜色：连接状态决定
            Color device_color = device.is_connected ? COLOR_TEXT_WHITE : COLOR_RED;
            
            // 使用新的ADD_MENU_WITH_STR宏传递设备名称
            ADD_MENU_WITH_STR(device.device_name, "sensitivity_device", device.device_name, device_color)
        }
    }
    
    PAGE_END()
}



} // namespace ui