#include "sensitivity_device.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include <cstdio>
#include <cstring>

namespace ui {

std::string SensitivityDevice::device_name_ = "";
std::vector<int32_t> SensitivityDevice::cached_sensitivity_values_ = {};
TouchDeviceMapping SensitivityDevice::cached_mapping_ = {};

SensitivityDevice::SensitivityDevice() : mapping_cached_(false) {
    // 构造函数无需特殊初始化
}

void SensitivityDevice::render(PageTemplate& page_template) {
    // 使用通过jump_str传递的设备名称
    if (device_name_.empty()) {
        PAGE_START()
        SET_TITLE("灵敏度设置", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("无效的设备参数", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE(device_name_.c_str(), COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 初始化缓存的灵敏度值
    init_cached_values();
    
    if (!mapping_cached_) {
        PAGE_START()
        SET_TITLE(device_name_, COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("设备未找到或未连接", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    PAGE_START()
    SET_TITLE(device_name_, COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    ADD_TEXT("MAX:99 MIN:0", COLOR_WHITE, LineAlign::CENTER)

    // 显示每个通道的灵敏度设置
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        // 检查通道是否启用
        bool channel_enabled = (cached_mapping_.enabled_channels_mask & (1UL << ch)) != 0;
        if (!channel_enabled) {
            continue; // 跳过未启用的通道
        }
        
        // 生成通道标签
        char channel_label[24];
        snprintf(channel_label, sizeof(channel_label), "CH%d", ch);
        
        // 生成设置ID
        std::string setting_id = generate_channel_setting_id(device_name_, ch);
        
        // 使用缓存的灵敏度值
        int32_t* sensitivity_value_ptr = &cached_sensitivity_values_[ch];
        
        // 添加整数设置行，使用缓存的值指针和回调函数
        ADD_INT_SETTING(sensitivity_value_ptr, 0, 99, channel_label, setting_id, nullptr, on_sensitivity_complete, COLOR_TEXT_WHITE)
    }
    
    // 如果没有启用的通道
    if (cached_mapping_.enabled_channels_mask == 0) {
        ADD_TEXT("该设备无启用通道", COLOR_YELLOW, LineAlign::CENTER)
    }
    
    PAGE_END()
}

void SensitivityDevice::jump_str(const std::string& str) {
    // 接收通过ADD_MENU_WITH_STR传递的设备名称
    device_name_ = str;
    // 重置缓存状态
    mapping_cached_ = false;
    cached_sensitivity_values_.clear();
}

bool SensitivityDevice::get_device_mapping(const std::string& device_name, TouchDeviceMapping& mapping) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 获取所有设备状态
    uint8_t device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    
    input_manager->get_all_device_status(device_status);
    
    // 查找指定设备
    for (int i = 0; i < device_count; i++) {
        if (device_status[i].device_name == device_name && device_status[i].is_connected) {
            mapping = device_status[i].touch_device;
            return true;
        }
    }
    
    return false;
}

std::string SensitivityDevice::generate_channel_setting_id(const std::string& device_name, uint8_t channel) {
    char setting_id[64];
    snprintf(setting_id, sizeof(setting_id), "CH: %d", channel);
    return std::string(setting_id);
}

void SensitivityDevice::init_cached_values() {
    if (mapping_cached_) {
        return; // 已经初始化过了
    }
    
    // 获取设备映射信息并缓存
    if (!get_device_mapping(device_name_, cached_mapping_)) {
        return;
    }
    
    // 初始化缓存的灵敏度值数组
    cached_sensitivity_values_.clear();
    cached_sensitivity_values_.resize(cached_mapping_.max_channels);
    
    // 复制当前的灵敏度值到缓存
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        cached_sensitivity_values_[ch] = static_cast<int32_t>(cached_mapping_.sensitivity[ch]);
    }
    
    mapping_cached_ = true;
}

void SensitivityDevice::on_sensitivity_complete() {
    // 将缓存的灵敏度值写回到InputManager配置
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        input_manager->setSensitivity(cached_mapping_.device_id_mask, ch, cached_sensitivity_values_[ch]);
    }
    UIManager::log_debug_static("on_sensitivity_complete");
}

} // namespace ui