#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include "../../../../protocol/hid/hid.h"
#include <string>
#include <vector>
#include <functional>
#include <cstdint>

namespace ui {

/**
 * 键盘设置页面构造器
 * 显示和修改按键绑定的键盘映射关系
 */
class KeyboardSettings : public PageConstructor {
public:
    KeyboardSettings();
    virtual ~KeyboardSettings() = default;
    
    /**
     * 渲染键盘设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    // 按键映射信息结构
    struct KeyMappingInfo {
        uint8_t gpio_id;
        HID_KeyCode current_key;
        std::string gpio_name;
        std::string key_name;
        
        KeyMappingInfo(uint8_t id, HID_KeyCode key, const std::string& gpio, const std::string& keyname)
            : gpio_id(id), current_key(key), gpio_name(gpio), key_name(keyname) {}
    };
    
    // 可选择的按键列表
    static const HID_KeyCode AVAILABLE_KEYS[];
    static const char* const KEY_NAMES[];
    static const size_t AVAILABLE_KEYS_COUNT;
    
    // 当前状态
    static std::vector<KeyMappingInfo> key_mappings_;
    static bool mappings_loaded_;
    
    // 静态回调函数
    static void onKeyMappingChange(size_t mapping_index, JoystickState state);
    
    // 辅助函数
    static void loadKeyMappings();
    static void saveKeyMappings();
    static std::string getKeyName(HID_KeyCode key);
    static size_t findKeyIndex(HID_KeyCode key);
    static std::string getGPIOName(uint8_t gpio_id);
};

} // namespace ui