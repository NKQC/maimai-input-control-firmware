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
            
            // 检查是否已有绑区设置
            // 使用InputManager接口检查是否存在映射
            bool has_existing_mapping = input_manager->hasAvailableSerialMapping();
            
            if (has_existing_mapping) {
                ADD_TEXT("已有绑区 继续将覆盖", COLOR_TEXT_GREEN, LineAlign::CENTER)
            }
            
            ADD_BUTTON("开始绑区", []() {
                BindingSettings::start_serial_binding();
            }, COLOR_TEXT_GREEN, LineAlign::CENTER)

            ADD_MENU("绑区信息", "binding_info", COLOR_BLUE)
            break;
        }
        
        case BindingUIState::BINDING_ACTIVE: {
            uint8_t progress = get_binding_progress();
            std::string current_area = get_current_binding_area();

            static uint8_t progress_data = 0;
            progress_data = progress;
            ADD_PROGRESS(&progress_data, COLOR_TEXT_YELLOW)
            
            if (!current_area.empty()) {
                char area_text[128];
                snprintf(area_text, sizeof(area_text), "当前绑定: %s", current_area.c_str());
                ADD_TEXT(area_text, COLOR_TEXT_WHITE, LineAlign::CENTER)
            }
            ADD_BUTTON("终止绑区", []() {
                BindingSettings::stop_binding();
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
            
            ADD_BUTTON("确认保存", []() {
                BindingSettings::confirm_and_save_binding();
            }, COLOR_TEXT_GREEN, LineAlign::CENTER)
            ADD_BUTTON("重新绑区", []() {
                BindingSettings::start_serial_binding();
            }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_BUTTON("取消", []() {
                BindingSettings::stop_binding();
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
            
            ADD_BUTTON("重试", []() {
                BindingSettings::start_serial_binding();
            }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
            ADD_BUTTON("取消", []() {
                BindingSettings::stop_binding();
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
            
        case BindingState::WAIT_TOUCH:
        case BindingState::PROCESSING:
            s_ui_state = BindingUIState::BINDING_ACTIVE;
            break;
            
        // 注意：COMPLETE状态已被移除，绑定完成后直接回到IDLE状态
            

            
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
    
    // 根据绑定状态计算进度
    BindingState binding_state = input_manager->getBindingState();
    switch (binding_state) {
        case BindingState::WAIT_TOUCH:
        case BindingState::PROCESSING: {
            // 获取当前绑定索引，计算准确进度
            uint8_t current_index = input_manager->getCurrentBindingIndex();
            return (current_index * 100) / 34; // 总共34个区域
        }
        case BindingState::IDLE:
            return 100;
        default:
            return 0;
    }
}

std::string BindingSettings::get_current_binding_area() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return "";
    }
    
    // 根据绑定状态返回当前区域信息
    BindingState binding_state = input_manager->getBindingState();
    switch (binding_state) {
        case BindingState::WAIT_TOUCH:
        case BindingState::PROCESSING: {
            // 获取当前绑定索引并显示具体区域名称
            uint8_t current_index = input_manager->getCurrentBindingIndex();
            if (current_index < 34) {
                return get_mai2_area_name(current_index);
            }
            return "绑定完成";
        }
        case BindingState::IDLE:
            return "绑定完成";
        default:
            return "";
    }
}

bool BindingSettings::start_serial_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 启动Serial绑定，使用lambda作为回调
    input_manager->startSerialBinding([](bool success, const char* message) {
        // 绑定回调处理 - 可以在这里更新UI状态或显示消息
        // 暂时不做特殊处理
    });
    return true;
}

bool BindingSettings::stop_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 取消绑定
    input_manager->requestCancelBinding();
    return true;
}

bool BindingSettings::confirm_and_save_binding() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 确认自动绑区结果（如果是自动绑区完成状态）
    if (input_manager->isAutoSerialBindingComplete()) {
        input_manager->confirmAutoSerialBinding();
    }
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