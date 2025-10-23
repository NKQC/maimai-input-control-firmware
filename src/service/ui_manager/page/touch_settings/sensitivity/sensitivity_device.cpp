#include "sensitivity_device.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
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
    // 检查InputManager是否初始化
    auto* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("设备灵敏度", COLOR_WHITE)
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
    
    // 获取设备以判断灵敏度模式
    TouchSensor* device = nullptr;
    const auto& devices = input_manager->getTouchSensorDevices();
    for (TouchSensor* sensor : devices) {
        if (sensor && sensor->getModuleMask() == cached_mapping_.device_id_mask) {
            device = sensor;
            break;
        }
    }
    bool is_relative_mode = device && device->isSensitivityRelativeMode();
    
    if (is_relative_mode) {
        ADD_TEXT("相对模式 范围: -127到127", COLOR_WHITE, LineAlign::CENTER)
    } else {
        ADD_TEXT("绝对模式 范围: 0到99", COLOR_WHITE, LineAlign::CENTER)
    }

    // 显示每个通道的灵敏度设置
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        // 直接从设备获取通道启用状态
        bool channel_enabled = device && device->getChannelEnabled(ch);
        if (!channel_enabled) {
            continue; // 跳过未启用的通道
        }
        
        // 生成通道标签
        char channel_label[32];
        if (is_relative_mode) {
            snprintf(channel_label, sizeof(channel_label), "CH%d (相对)", ch);
        } else {
            snprintf(channel_label, sizeof(channel_label), "CH%d (绝对)", ch);
        }
        
        // 生成设置ID
        std::string setting_id = generate_channel_setting_id(device_name_, ch);
        
        // 使用缓存的灵敏度值
        int32_t* sensitivity_value_ptr = &cached_sensitivity_values_[ch];
        
        // 根据模式设置不同的范围
        if (is_relative_mode) {
            ADD_INT_SETTING(sensitivity_value_ptr, -127, 127, channel_label, setting_id, nullptr, on_sensitivity_complete, COLOR_TEXT_WHITE)
        } else {
            ADD_INT_SETTING(sensitivity_value_ptr, 0, 99, channel_label, setting_id, nullptr, on_sensitivity_complete, COLOR_TEXT_WHITE)
        }
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
    
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        mapping_cached_ = true;
        return;
    }
    
    // 复制当前的灵敏度值到缓存
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        // 获取对应的TouchSensor设备以判断灵敏度模式
        TouchSensor* device = nullptr;
        const auto& devices = input_manager->getTouchSensorDevices();
        for (TouchSensor* sensor : devices) {
            if (sensor && sensor->getModuleMask() == cached_mapping_.device_id_mask) {
                device = sensor;
                break;
            }
        }
        if (!device) {
            cached_sensitivity_values_[ch] = 0;
            continue;
        }

        // 读取当前通道灵敏度
        uint8_t sens = input_manager->getDeviceChannelSensitivity(cached_mapping_.device_id_mask, ch);
        
        // 根据设备的灵敏度模式设置初始值
        if (device->isSensitivityRelativeMode()) {
            // 相对模式：直接使用-127到127的原生范围
            int8_t relative_sensitivity = static_cast<int8_t>(sens);
            cached_sensitivity_values_[ch] = static_cast<int32_t>(relative_sensitivity);
        } else {
            // 绝对模式：直接使用0-99的绝对值
            int sens_clamped = (sens > 99) ? 99 : sens;
            cached_sensitivity_values_[ch] = static_cast<int32_t>(sens_clamped);
        }
    }
    
    mapping_cached_ = true;
}

void SensitivityDevice::on_sensitivity_complete() {
    auto* input_manager = InputManager::getInstance();
    if (!input_manager) return;

    // 获取设备以判断灵敏度模式
    TouchSensor* device = nullptr;
    const auto& devices = input_manager->getTouchSensorDevices();
    for (TouchSensor* sensor : devices) {
        if (sensor && sensor->getModuleMask() == cached_mapping_.device_id_mask) {
            device = sensor;
            break;
        }
    }
    if (!device) return;

    // 将缓存的相对灵敏度值写回到每个启用的通道
    for (uint8_t ch = 0; ch < cached_mapping_.max_channels; ch++) {
        // 直接从设备获取通道启用状态
        if (!device->getChannelEnabled(ch)) {
            continue; // 跳过未启用的通道
        }

        // 根据设备模式转换灵敏度值
        uint8_t final_sensitivity;
        if (device->isSensitivityRelativeMode()) {
            // 相对模式：直接使用-127到127的原生范围
            int clamped_value = (cached_sensitivity_values_[ch] < -127) ? -127 : 
                               ((cached_sensitivity_values_[ch] > 127) ? 127 : cached_sensitivity_values_[ch]);
            final_sensitivity = static_cast<uint8_t>(clamped_value);
        } else {
            // 绝对模式：直接使用0-99的绝对值
            int clamped_value = (cached_sensitivity_values_[ch] < 0) ? 0 : 
                               ((cached_sensitivity_values_[ch] > 99) ? 99 : cached_sensitivity_values_[ch]);
            final_sensitivity = static_cast<uint8_t>(clamped_value);
        }

        // 设置通道灵敏度
        input_manager->setDeviceChannelSensitivity(cached_mapping_.device_id_mask, ch, final_sensitivity);
    }
}

} // namespace ui