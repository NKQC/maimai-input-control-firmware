#include "binding_settings.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../input_manager/input_manager.h"
#include <cstdio>
#include <cstring>

namespace ui {

// 静态成员初始化
BindingSettings::BindingUIState BindingSettings::s_ui_state = BindingUIState::IDLE;
uint32_t BindingSettings::s_last_update_time = 0;

BindingSettings::BindingSettings() {
    // 构造函数无需特殊初始化
}

void BindingSettings::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("绑区设置", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 检查当前工作模式
    auto work_mode = input_manager->getWorkMode();
    if (work_mode != InputWorkMode::SERIAL_MODE) {
        PAGE_START()
        SET_TITLE("绑区设置", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("绑区功能仅在Serial模式下可用", COLOR_YELLOW, LineAlign::CENTER)
        ADD_TEXT("当前模式: HID模式", COLOR_TEXT_WHITE, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 更新当前状态
    BindingUIState current_state = get_current_binding_state();
    
    PAGE_START()
    SET_TITLE("绑区设置 (Serial模式)", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 根据状态显示不同内容
    switch (current_state) {
        case BindingUIState::IDLE: {
            ADD_TEXT("当前状态: 空闲", COLOR_TEXT_WHITE, LineAlign::CENTER)
            ADD_TEXT("准备开始交互式绑区", COLOR_TEXT_YELLOW, LineAlign::CENTER)
            
            // 检查是否已有绑区设置
            // 检查是否有现有的Serial映射
            bool has_existing_mapping = false;
            InputManager::TouchDeviceStatus device_status[8];
            int device_count;
            input_manager->get_all_device_status(device_status, device_count);
            
            for (int i = 0; i < device_count; i++) {
                for (int ch = 0; ch < device_status[i].touch_device.max_channels; ch++) {
                    if (device_status[i].touch_device.serial_mappings[ch].channel != 0) {
                        has_existing_mapping = true;
                        break;
                    }
                }
                if (has_existing_mapping) break;
            }
            
            if (has_existing_mapping) {
                ADD_TEXT("检测到现有绑区设置", COLOR_TEXT_GREEN, LineAlign::CENTER)
                ADD_TEXT("新的绑区将覆盖现有设置", COLOR_YELLOW, LineAlign::CENTER)
            }
            
            ADD_BUTTON("开始绑区", [this]() {
                this->start_serial_binding();
            }, COLOR_TEXT_GREEN, LineAlign::CENTER)
            break;
        }
        
        case BindingUIState::BINDING_ACTIVE: {
            uint8_t progress = get_binding_progress();
            std::string current_area = get_current_binding_area();
            
            ADD_TEXT("绑区进行中...", COLOR_TEXT_YELLOW, LineAlign::CENTER)
            
            char progress_text[64];
            snprintf(progress_text, sizeof(progress_text), "进度: %d%%", progress);
            ADD_TEXT(progress_text, COLOR_TEXT_WHITE, LineAlign::CENTER)
            
            static uint8_t progress_data = 0;
            progress_data = progress;
            ADD_PROGRESS(&progress_data, COLOR_TEXT_YELLOW)
            
            if (!current_area.empty()) {
                char area_text[128];
                snprintf(area_text, sizeof(area_text), "当前绑定: %s", current_area.c_str());
                ADD_TEXT(area_text, COLOR_TEXT_GREEN, LineAlign::CENTER)
            }
            
            ADD_BUTTON("回退一步", [this]() {
                this->step_back_binding();
            }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_BUTTON("终止绑区", [this]() {
                this->stop_binding();
            }, COLOR_RED, LineAlign::CENTER)
            break;
        }
        
        case BindingUIState::BINDING_COMPLETE: {
            ADD_TEXT("绑区完成!", COLOR_TEXT_GREEN, LineAlign::CENTER)
            ADD_TEXT("请确认保存设置", COLOR_TEXT_YELLOW, LineAlign::CENTER)
            
            // TODO: Implement getTotalBoundAreas
            // uint8_t total_areas = input_manager->getTotalBoundAreas();
            uint8_t total_areas = 0;
            char summary_text[64];
            snprintf(summary_text, sizeof(summary_text), "已绑定区域: %d个", total_areas);
            ADD_TEXT(summary_text, COLOR_TEXT_WHITE, LineAlign::CENTER)
            
            ADD_BUTTON("确认保存", [this]() {
                this->confirm_and_save_binding();
            }, COLOR_TEXT_GREEN, LineAlign::CENTER)
            ADD_BUTTON("重新绑区", [this]() {
                this->start_serial_binding();
            }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_BUTTON("取消", [this]() {
                this->stop_binding();
            }, COLOR_RED, LineAlign::CENTER)
            break;
        }
        
        case BindingUIState::BINDING_ERROR: {
            ADD_TEXT("绑区出现错误", COLOR_RED, LineAlign::CENTER)
            
            // TODO: Implement getLastBindingError
            // std::string error_msg = input_manager->getLastBindingError();
            std::string error_msg = "绑定错误";
            if (!error_msg.empty()) {
                ADD_TEXT(error_msg, COLOR_TEXT_WHITE, LineAlign::CENTER);
            }
            
            ADD_BUTTON("重试", [this]() {
                this->start_serial_binding();
            }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_BUTTON("取消", [this]() {
                this->stop_binding();
            }, COLOR_RED, LineAlign::CENTER)
            break;
        }
    }
    
    PAGE_END()
}

BindingSettings::BindingUIState BindingSettings::get_current_binding_state() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return BindingUIState::BINDING_ERROR;
    }
    
    // 获取InputManager的绑区状态
    BindingState binding_state = input_manager->getBindingState();
    
    switch (binding_state) {
        case BindingState::IDLE:
            s_ui_state = BindingUIState::IDLE;
            break;
            
        case BindingState::SERIAL_BINDING_WAIT_TOUCH:
        case BindingState::SERIAL_BINDING_PROCESSING:
            s_ui_state = BindingUIState::BINDING_ACTIVE;
            break;
            
        case BindingState::SERIAL_BINDING_COMPLETE:
            s_ui_state = BindingUIState::BINDING_COMPLETE;
            break;
            
        case BindingState::HID_BINDING_INIT:
        case BindingState::HID_BINDING_WAIT_TOUCH:
            s_ui_state = BindingUIState::BINDING_ERROR;
            break;
            
        default:
            s_ui_state = BindingUIState::IDLE;
            break;
    }
    
    return s_ui_state;
}

uint8_t BindingSettings::get_binding_progress() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return 0;
    }
    
    // TODO: Implement getBindingProgress
    // return input_manager->getBindingProgress();
    return 0;
}

std::string BindingSettings::get_current_binding_area() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return "";
    }
    
    // TODO: Implement getCurrentBindingAreaIndex
    // uint8_t current_area_index = input_manager->getCurrentBindingAreaIndex();
    uint8_t current_area_index = 0;
    return get_mai2_area_name(current_area_index);
}

bool BindingSettings::start_serial_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // TODO: Implement startSerialBinding with callback
    // input_manager->startSerialBinding();
    return true;
}

bool BindingSettings::stop_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // TODO: Implement stopBinding
    // return input_manager->stopBinding();
    return true;
}

bool BindingSettings::confirm_and_save_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 确认并保存绑定设置
    // 这里应该调用相应的确认方法，暂时返回true
    return true;
}

bool BindingSettings::step_back_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 回退绑定步骤
    // 这里应该调用相应的回退方法，暂时返回true
    return true;
}

std::string BindingSettings::format_binding_state_text(BindingUIState state) {
    switch (state) {
        case BindingUIState::IDLE:
            return "空闲";
        case BindingUIState::BINDING_ACTIVE:
            return "绑区中";
        case BindingUIState::BINDING_COMPLETE:
            return "完成";
        case BindingUIState::BINDING_ERROR:
            return "错误";
        default:
            return "未知";
    }
}

std::string BindingSettings::get_mai2_area_name(uint8_t area_index) {
    // Mai2标准区域名称映射
    static const char* area_names[] = {
        "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8",
        "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8",
        "C1", "C2", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
        "E1", "E2", "E3", "E4", "E5", "E6", "E7", "E8"
    };
    
    if (area_index < sizeof(area_names) / sizeof(area_names[0])) {
        return std::string(area_names[area_index]);
    }
    
    char unknown_area[16];
    snprintf(unknown_area, sizeof(unknown_area), "区域%d", area_index);
    return std::string(unknown_area);
}

} // namespace ui