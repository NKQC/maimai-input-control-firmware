#include "interactive_sensitivity.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include <cstdio>
#include <cstring>
#include "pico/time.h"

namespace ui {

// 静态成员初始化
InteractiveSensitivity::InteractiveState InteractiveSensitivity::s_current_state = InteractiveState::WAITING_TOUCH;
std::string InteractiveSensitivity::s_detected_device = "";
uint8_t InteractiveSensitivity::s_detected_channel = 0;
uint8_t InteractiveSensitivity::s_suggested_sensitivity = 0;
uint32_t InteractiveSensitivity::s_last_update_time = 0;

InteractiveSensitivity::InteractiveSensitivity() {
    // 构造函数无需特殊初始化
}

void InteractiveSensitivity::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("交互式灵敏度调整", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 更新当前状态
    InteractiveState current_state = get_current_state();
    
    PAGE_START()
    SET_TITLE("交互式灵敏度调整", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 根据状态显示不同内容
    switch (current_state) {
        case InteractiveState::WAITING_TOUCH:
            ADD_TEXT("请触摸需要调整的区域", COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_TEXT("系统将自动检测触摸位置", COLOR_TEXT_WHITE, LineAlign::CENTER)
            break;
            
        case InteractiveState::TOUCH_DETECTED: {
            char device_info[128];
            snprintf(device_info, sizeof(device_info), "检测到: %s CH%d", 
                    s_detected_device.c_str(), s_detected_channel);
            ADD_TEXT(device_info, COLOR_TEXT_GREEN, LineAlign::CENTER)
            
            char sensitivity_info[64];
            snprintf(sensitivity_info, sizeof(sensitivity_info), "建议灵敏度: %d", s_suggested_sensitivity);
            ADD_TEXT(sensitivity_info, COLOR_TEXT_WHITE, LineAlign::CENTER)
            
            ADD_BUTTON("确认调整", []() { InteractiveSensitivity::on_confirm_adjust(); }, COLOR_TEXT_GREEN, LineAlign::CENTER)
            ADD_BUTTON("重新检测", []() { InteractiveSensitivity::on_retry_detect(); }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            break;
        }
        
        case InteractiveState::ADJUSTING:
            ADD_TEXT("正在调整灵敏度...", COLOR_TEXT_YELLOW, LineAlign::CENTER)
            // 创建进度数据变量
            static uint8_t adjust_progress = 50;
            ADD_PROGRESS(&adjust_progress, COLOR_TEXT_WHITE)
            break;
            
        case InteractiveState::COMPLETED: {
            ADD_TEXT("灵敏度调整完成!", COLOR_TEXT_GREEN, LineAlign::CENTER)
            
            char result_info[128];
            snprintf(result_info, sizeof(result_info), "%s CH%d -> %d", 
                    s_detected_device.c_str(), s_detected_channel, s_suggested_sensitivity);
            ADD_TEXT(result_info, COLOR_TEXT_WHITE, LineAlign::CENTER)
            
            ADD_BUTTON("继续调整", []() { InteractiveSensitivity::on_continue_adjust(); }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            break;
        }
        
        case InteractiveState::ERROR:
            ADD_TEXT("调整失败", COLOR_RED, LineAlign::CENTER)
            ADD_TEXT("请检查设备连接状态", COLOR_TEXT_WHITE, LineAlign::CENTER)
            ADD_BUTTON("重试", []() { InteractiveSensitivity::on_retry_adjust(); }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            break;
    }
    
    PAGE_END()
}

InteractiveSensitivity::InteractiveState InteractiveSensitivity::get_current_state() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return InteractiveState::ERROR;
    }
    
    // 根据当前状态和输入情况更新状态
    switch (s_current_state) {
        case InteractiveState::WAITING_TOUCH: {
            std::string detected_device;
            uint8_t detected_channel;
            if (detect_touch_area(detected_device, detected_channel)) {
                s_detected_device = detected_device;
                s_detected_channel = detected_channel;
                s_suggested_sensitivity = get_suggested_sensitivity(detected_device, detected_channel);
                s_current_state = InteractiveState::TOUCH_DETECTED;
            }
            break;
        }
        
        case InteractiveState::TOUCH_DETECTED:
            // 等待用户确认或重新检测
            break;
            
        case InteractiveState::ADJUSTING:
            // 模拟调整过程，实际应该检查调整是否完成
            if (s_last_update_time == 0) {
                s_last_update_time = to_ms_since_boot(get_absolute_time());
            } else {
                uint32_t current_time = to_ms_since_boot(get_absolute_time());
                if (current_time - s_last_update_time > 1000) { // 1秒后完成
                    s_current_state = InteractiveState::COMPLETED;
                    s_last_update_time = 0;
                }
            }
            break;
            
        case InteractiveState::COMPLETED:
        case InteractiveState::ERROR:
            // 保持状态直到用户操作
            break;
    }
    
    return s_current_state;
}

bool InteractiveSensitivity::detect_touch_area(std::string& detected_device, uint8_t& detected_channel) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count = 0;
    input_manager->get_all_device_status(device_status);
    
    // 检查每个设备的触摸状态
    for (int i = 0; i < device_count; i++) {
        const auto& device = device_status[i];
        if (!device.is_connected) {
            continue;
        }
        
        // 检查每个通道的触摸状态
        for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
            bool channel_enabled = (device.touch_device.enabled_channels_mask & (1UL << ch)) != 0;
            bool channel_touched = (device.touch_states_32bit & (1UL << ch)) != 0;
            
            if (channel_enabled && channel_touched) {
                detected_device = device.device_name;
                detected_channel = ch;
                return true;
            }
        }
    }
    
    return false;
}

bool InteractiveSensitivity::adjust_sensitivity(const std::string& device_name, uint8_t channel, uint8_t new_sensitivity) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 调用InputManager的灵敏度设置接口
    return input_manager->setSensitivityByDeviceName(device_name, channel, new_sensitivity);
}

std::string InteractiveSensitivity::format_state_text(InteractiveState state) {
    switch (state) {
        case InteractiveState::WAITING_TOUCH:
            return "等待触摸";
        case InteractiveState::TOUCH_DETECTED:
            return "检测到触摸";
        case InteractiveState::ADJUSTING:
            return "正在调整";
        case InteractiveState::COMPLETED:
            return "调整完成";
        case InteractiveState::ERROR:
            return "错误";
        default:
            return "未知状态";
    }
}

uint8_t InteractiveSensitivity::get_suggested_sensitivity(const std::string& device_name, uint8_t channel) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return 15; // 默认值
    }
    
    // 获取所有设备状态
    int device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    // 查找指定设备的device_id_mask
    uint8_t device_id_mask = 0;
    for (int i = 0; i < device_count; i++) {
        if (device_status[i].device_name == device_name && device_status[i].is_connected) {
            device_id_mask = device_status[i].touch_device.device_id_mask;
            break;
        }
    }
    
    if (device_id_mask == 0) {
        return 15; // 设备未找到，返回默认值
    }
    
    // 获取当前灵敏度
    uint8_t current_sensitivity = input_manager->getSensitivity(device_id_mask, channel);
    
    // 简单的建议算法：如果当前值较低，建议提高；如果较高，建议降低
    if (current_sensitivity < 10) {
        return current_sensitivity + 5;
    } else if (current_sensitivity > 20) {
        return current_sensitivity - 5;
    } else {
        return current_sensitivity; // 保持当前值
    }
}

// 按钮回调函数实现
void InteractiveSensitivity::on_confirm_adjust() {
    // 确认调整：开始执行灵敏度调整
    s_current_state = InteractiveState::ADJUSTING;
    s_last_update_time = to_ms_since_boot(get_absolute_time());
    
    // 执行实际的灵敏度调整
    InputManager* input_manager = InputManager::getInstance();
    if (input_manager && !s_detected_device.empty()) {
        // 这里应该调用实际的灵敏度调整函数
        // 暂时模拟调整过程
        s_current_state = InteractiveState::COMPLETED;
    } else {
        s_current_state = InteractiveState::ERROR;
    }
}

void InteractiveSensitivity::on_retry_detect() {
    // 重新检测：重置状态到等待触摸
    s_current_state = InteractiveState::WAITING_TOUCH;
    s_detected_device = "";
    s_detected_channel = 0;
    s_suggested_sensitivity = 0;
    s_last_update_time = to_ms_since_boot(get_absolute_time());
}

void InteractiveSensitivity::on_continue_adjust() {
    // 继续调整：重置状态到等待触摸，准备调整下一个区域
    s_current_state = InteractiveState::WAITING_TOUCH;
    s_detected_device = "";
    s_detected_channel = 0;
    s_suggested_sensitivity = 0;
    s_last_update_time = to_ms_since_boot(get_absolute_time());
}

void InteractiveSensitivity::on_retry_adjust() {
    // 重试调整：重置状态到等待触摸
    s_current_state = InteractiveState::WAITING_TOUCH;
    s_detected_device = "";
    s_detected_channel = 0;
    s_suggested_sensitivity = 0;
    s_last_update_time = to_ms_since_boot(get_absolute_time());
}

} // namespace ui