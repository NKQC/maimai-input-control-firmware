#include "keyboard_settings.h"
#include "../../ui_manager.h"
#include "../../../input_manager/input_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/graphics_rendering/graphics_engine.h"

namespace ui {

// 可选择的按键列表
const HID_KeyCode KeyboardSettings::AVAILABLE_KEYS[] = {
    HID_KeyCode::KEY_NONE,
    HID_KeyCode::KEY_A, HID_KeyCode::KEY_B, HID_KeyCode::KEY_C, HID_KeyCode::KEY_D,
    HID_KeyCode::KEY_E, HID_KeyCode::KEY_F, HID_KeyCode::KEY_G, HID_KeyCode::KEY_H,
    HID_KeyCode::KEY_I, HID_KeyCode::KEY_J, HID_KeyCode::KEY_K, HID_KeyCode::KEY_L,
    HID_KeyCode::KEY_M, HID_KeyCode::KEY_N, HID_KeyCode::KEY_O, HID_KeyCode::KEY_P,
    HID_KeyCode::KEY_Q, HID_KeyCode::KEY_R, HID_KeyCode::KEY_S, HID_KeyCode::KEY_T,
    HID_KeyCode::KEY_U, HID_KeyCode::KEY_V, HID_KeyCode::KEY_W, HID_KeyCode::KEY_X,
    HID_KeyCode::KEY_Y, HID_KeyCode::KEY_Z,
    HID_KeyCode::KEY_1, HID_KeyCode::KEY_2, HID_KeyCode::KEY_3, HID_KeyCode::KEY_4,
    HID_KeyCode::KEY_5, HID_KeyCode::KEY_6, HID_KeyCode::KEY_7, HID_KeyCode::KEY_8,
    HID_KeyCode::KEY_9, HID_KeyCode::KEY_0,
    HID_KeyCode::KEY_ENTER, HID_KeyCode::KEY_ESCAPE, HID_KeyCode::KEY_BACKSPACE,
    HID_KeyCode::KEY_TAB, HID_KeyCode::KEY_SPACE,
    HID_KeyCode::KEY_F1, HID_KeyCode::KEY_F2, HID_KeyCode::KEY_F3, HID_KeyCode::KEY_F4,
    HID_KeyCode::KEY_F5, HID_KeyCode::KEY_F6, HID_KeyCode::KEY_F7, HID_KeyCode::KEY_F8,
    HID_KeyCode::KEY_F9, HID_KeyCode::KEY_F10, HID_KeyCode::KEY_F11, HID_KeyCode::KEY_F12,
    HID_KeyCode::KEY_LEFT_ARROW, HID_KeyCode::KEY_DOWN_ARROW, HID_KeyCode::KEY_UP_ARROW, HID_KeyCode::KEY_RIGHT_ARROW
};

const char* const KeyboardSettings::KEY_NAMES[] = {
    "无", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P",
    "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
    "回车", "ESC", "退格", "Tab", "空格",
    "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
    "左", "下", "上", "右"
};

const size_t KeyboardSettings::AVAILABLE_KEYS_COUNT = sizeof(AVAILABLE_KEYS) / sizeof(AVAILABLE_KEYS[0]);

// 静态成员变量定义
std::vector<KeyboardSettings::KeyMappingInfo> KeyboardSettings::key_mappings_;
bool KeyboardSettings::mappings_loaded_ = false;

KeyboardSettings::KeyboardSettings() {
    if (!mappings_loaded_) {
        loadKeyMappings();
        mappings_loaded_ = true;
    }
}

void KeyboardSettings::render(PageTemplate& page_template) {
    PAGE_WITH_TITLE("键盘设置", COLOR_PRIMARY)
    
    // 返回项
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    if (key_mappings_.empty()) {
        ADD_TEXT("未找到可配置的按键映射", COLOR_TEXT_WHITE, LineAlign::LEFT)
        ADD_TEXT("请检查InputManager配置", COLOR_TEXT_WHITE, LineAlign::LEFT)
    } else {
        // 显示每个按键映射
        for (size_t i = 0; i < key_mappings_.size(); i++) {
            const auto& mapping = key_mappings_[i];
            
            static char mapping_text[128];
            snprintf(mapping_text, sizeof(mapping_text), "%s: %s", 
                     mapping.gpio_name.c_str(), mapping.key_name.c_str());
            
            // 创建lambda捕获索引
            auto callback = [i](JoystickState state) {
                onKeyMappingChange(i, state);
            };
            
            ADD_SIMPLE_SELECTOR(mapping_text, callback, COLOR_TEXT_WHITE)
        }
    }
    
    PAGE_END()
}

void KeyboardSettings::onKeyMappingChange(size_t mapping_index, JoystickState state) {
    if (mapping_index >= key_mappings_.size()) {
        return;
    }
    
    auto& mapping = key_mappings_[mapping_index];
    size_t current_key_index = findKeyIndex(mapping.current_key);
    
    if (state == JoystickState::UP) {
        if (current_key_index < AVAILABLE_KEYS_COUNT - 1) {
            current_key_index++;
        } else {
            current_key_index = 0; // 循环到第一个
        }
    } else if (state == JoystickState::DOWN) {
        if (current_key_index > 0) {
            current_key_index--;
        } else {
            current_key_index = AVAILABLE_KEYS_COUNT - 1; // 循环到最后一个
        }
    }
    
    // 更新映射
    mapping.current_key = AVAILABLE_KEYS[current_key_index];
    mapping.key_name = getKeyName(mapping.current_key);
    
    // 保存更改
    saveKeyMappings();
}

void KeyboardSettings::loadKeyMappings() {
    key_mappings_.clear();
    
    InputManager* input_mgr = InputManager::getInstance();
    if (!input_mgr) {
        return;
    }
    
    // 获取物理键盘映射
    const auto& physical_keyboards = input_mgr->getPhysicalKeyboards();
    for (const auto& keyboard : physical_keyboards) {
        // 使用default_key作为当前按键
        HID_KeyCode current_key = keyboard.default_key;
        
        std::string gpio_name = getGPIOName(keyboard.gpio);
        std::string key_name = getKeyName(current_key);
        
        key_mappings_.emplace_back(keyboard.gpio, current_key, gpio_name, key_name);
    }
}

void KeyboardSettings::saveKeyMappings() {
    InputManager* input_mgr = InputManager::getInstance();
    if (!input_mgr) {
        return;
    }
    
    // 清除现有的逻辑按键映射
    input_mgr->clearAllLogicalKeyMappings();
    
    // 添加新的逻辑按键映射
    for (const auto& mapping : key_mappings_) {
        if (mapping.current_key != HID_KeyCode::KEY_NONE) {
            input_mgr->addLogicalKeyMapping(mapping.gpio_id, mapping.current_key);
        }
    }
}

std::string KeyboardSettings::getKeyName(HID_KeyCode key) {
    size_t index = findKeyIndex(key);
    if (index < AVAILABLE_KEYS_COUNT) {
        return KEY_NAMES[index];
    }
    return "未知";
}

size_t KeyboardSettings::findKeyIndex(HID_KeyCode key) {
    for (size_t i = 0; i < AVAILABLE_KEYS_COUNT; i++) {
        if (AVAILABLE_KEYS[i] == key) {
            return i;
        }
    }
    return 0; // 默认返回"无"的索引
}

std::string KeyboardSettings::getGPIOName(uint8_t gpio_id) {
    // 根据GPIO ID生成名称
    if (gpio_id < 32) {
        return "MCU_GPIO" + std::to_string(gpio_id);
    } else {
        return "MCP_GPIO" + std::to_string(gpio_id - 32);
    }
}

} // namespace ui