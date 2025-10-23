#include "area_sensitivity_detail.h"
#include "area_sensitivity.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdio>

namespace ui {

// 静态成员初始化
uint8_t AreaSensitivityDetail::current_area_index_static_ = 1;
int32_t AreaSensitivityDetail::current_sensitivity_value_ = 200;

AreaSensitivityDetail::AreaSensitivityDetail() : current_area_index_(1) {
}

std::string AreaSensitivityDetail::getAreaName(uint8_t area_index) {
    // 区域名称映射 (1-34对应A1-E8)
    if (area_index < 1 || area_index > 34) {
        return "未知";
    }
    
    uint8_t zone_idx = (area_index - 1) / 8;  // 0-4对应A-E
    uint8_t area_in_zone = (area_index - 1) % 8 + 1;  // 1-8
    
    const char* zone_names[] = {"A", "B", "C", "D", "E"};
    
    static char area_name[8];
    snprintf(area_name, sizeof(area_name), "%s%d", zone_names[zone_idx], area_in_zone);
    return area_name;
}

void AreaSensitivityDetail::jump_str(const std::string& str) {
    // 解析传入的区域索引字符串
    current_area_index_ = static_cast<uint8_t>(std::stoi(str));
    // 同步到静态成员
    current_area_index_static_ = current_area_index_;
}

int32_t AreaSensitivityDetail::getCurrentSensitivity() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return 0;
    }
    
    // 检查是否为Serial模式
    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        return 0;
    }
    
    // 通过区域索引获取对应的灵敏度值
    // 遍历所有设备映射，查找绑定到当前区域的通道
    uint8_t device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
        const auto& device = device_status[dev_idx];
        if (!device.is_connected) continue;
        
        uint8_t device_id_mask = device.touch_device.device_id_mask;
        
        for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
            Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
            
            if (mapped_area == static_cast<Mai2_TouchArea>(current_area_index_)) {
                // 找到了映射到当前区域的通道，获取其灵敏度
                uint8_t raw_sensitivity = input_manager->getDeviceChannelSensitivity(device_id_mask, ch);
                
                // 检查设备是否为相对模式
                TouchSensor* sensor = input_manager->getTouchSensorByDeviceName(device.device_name);
                if (sensor && sensor->isSensitivityRelativeMode()) {
                    // 相对模式：直接使用-127到127的原生范围
                    int8_t relative_sensitivity = static_cast<int8_t>(raw_sensitivity);
                    return static_cast<int32_t>(relative_sensitivity);
                } else {
                    // 绝对模式：直接返回0-99范围的值
                    return static_cast<int32_t>(raw_sensitivity);
                }
            }
        }
    }
    
    // 如果没有找到映射，返回默认值
    // 对于相对模式，默认值应该是0；对于绝对模式，默认值是50
    return 0;
}

void AreaSensitivityDetail::setSensitivity(int32_t value) {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
    
    // 检查是否为Serial模式
    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        return;
    }
    
    // 查找映射到当前区域的设备和通道，检查是否为相对模式
    uint8_t device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
        const auto& device = device_status[dev_idx];
        if (!device.is_connected) continue;
        
        uint8_t device_id_mask = device.touch_device.device_id_mask;
        
        for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
            Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
            
            if (mapped_area == static_cast<Mai2_TouchArea>(current_area_index_)) {
                // 找到了映射到当前区域的通道
                TouchSensor* sensor = input_manager->getTouchSensorByDeviceName(device.device_name);
                if (sensor && sensor->isSensitivityRelativeMode()) {
                    // 相对模式：直接使用-127到127范围的值
                    input_manager->setSerialAreaSensitivity(static_cast<Mai2_TouchArea>(current_area_index_), static_cast<int8_t>(value));
                } else {
                    // 绝对模式：使用0-99范围的值
                    input_manager->setSerialAreaSensitivity(static_cast<Mai2_TouchArea>(current_area_index_), static_cast<int8_t>(value));
                }
                return;
            }
        }
    }
    
    // 如果没有找到映射，仍然尝试设置
    input_manager->setSerialAreaSensitivity(static_cast<Mai2_TouchArea>(current_area_index_), static_cast<int8_t>(value));
    
    // 更新区域信息中的修改标记
    auto* zone_infos = AreaSensitivity::getZoneInfos();
    uint8_t zone_idx = (current_area_index_ - 1) / 8;
    uint8_t area_in_zone = (current_area_index_ - 1) % 8;
    
    if (zone_idx < 5) {
        zone_infos[zone_idx].areas[area_in_zone].current_value = value;
        zone_infos[zone_idx].areas[area_in_zone].has_modified = true;
    }
}

bool AreaSensitivityDetail::supportsSensitivity() {
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return false;
    }
    
    // 检查是否为Serial模式
    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        return false;
    }
    
    // 检查当前区域是否有绑定的通道
    uint8_t device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
        const auto& device = device_status[dev_idx];
        if (!device.is_connected) continue;
        
        uint8_t device_id_mask = device.touch_device.device_id_mask;
        
        for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
            Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
            
            if (mapped_area == static_cast<Mai2_TouchArea>(current_area_index_)) {
                // 找到了映射到当前区域的通道，检查设备是否支持灵敏度调整
                // 通过设备名称获取TouchSensor实例
                TouchSensor* sensor = input_manager->getTouchSensorByDeviceName(device.device_name);
                if (sensor && sensor->supportsGeneralSensitivity()) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

void AreaSensitivityDetail::render(PageTemplate& page_template) {
    PAGE_START()
    
    // 设置标题
    static char title[32];
    snprintf(title, sizeof(title), "%s区灵敏度", getAreaName(current_area_index_).c_str());
    SET_TITLE(title, COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    if (!supportsSensitivity()) {
        ADD_TEXT("该区域不支持灵敏度调整", COLOR_RED, LineAlign::CENTER)
        ADD_TEXT("可能原因:", COLOR_WHITE, LineAlign::LEFT)
        ADD_TEXT("- 区域未绑定设备", COLOR_WHITE, LineAlign::LEFT)
        ADD_TEXT("- 设备不支持灵敏度", COLOR_WHITE, LineAlign::LEFT)
        ADD_TEXT("- 设备处于相对模式", COLOR_WHITE, LineAlign::LEFT)
    } else {
        int32_t current_value = getCurrentSensitivity();
        
        // 检查当前区域是否为相对模式
        InputManager* input_manager = InputManager::getInstance();
        bool is_relative_mode = false;
        if (input_manager) {
            uint8_t device_count = input_manager->get_device_count();
            InputManager::TouchDeviceStatus device_status[device_count];
            input_manager->get_all_device_status(device_status);
            
            for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
                const auto& device = device_status[dev_idx];
                if (!device.is_connected) continue;
                
                uint8_t device_id_mask = device.touch_device.device_id_mask;
                
                for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
                    Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
                    
                    if (mapped_area == static_cast<Mai2_TouchArea>(current_area_index_)) {
                        TouchSensor* sensor = input_manager->getTouchSensorByDeviceName(device.device_name);
                        if (sensor && sensor->isSensitivityRelativeMode()) {
                            is_relative_mode = true;
                        }
                        break;
                    }
                }
                if (is_relative_mode) break;
            }
        }
        
        // 显示当前值和模式信息
        static char current_text[64];
        if (is_relative_mode) {
            snprintf(current_text, sizeof(current_text), "当前灵敏度: %ld (相对模式)", current_value);
            ADD_TEXT(current_text, COLOR_WHITE, LineAlign::CENTER)
            ADD_TEXT("范围: -127 到 127", COLOR_WHITE, LineAlign::CENTER)
        } else {
            snprintf(current_text, sizeof(current_text), "当前灵敏度: %ld (绝对模式)", current_value);
            ADD_TEXT(current_text, COLOR_WHITE, LineAlign::CENTER)
            ADD_TEXT("范围: 0 到 99", COLOR_WHITE, LineAlign::CENTER)
        }
        
        // 使用INT_SETTING来实现灵敏度调整
        static int32_t sensitivity_value = current_value;
        sensitivity_value = current_value; // 同步当前值
        current_sensitivity_value_ = current_value; // 同步到静态成员
        
        // 根据模式设置不同的范围
        if (is_relative_mode) {
            ADD_INT_SETTING(&sensitivity_value, -127, 127, "灵敏度", "area_sens", 
                [](int32_t value) { 
                    // 值变化时同步到静态成员变量
                    current_sensitivity_value_ = value; 
                },  // 使用lambda同步值到静态成员
                on_sensitivity_complete,  // 使用静态完成回调
                COLOR_TEXT_WHITE);
        } else {
            ADD_INT_SETTING(&sensitivity_value, 0, 99, "灵敏度", "area_sens", 
                [](int32_t value) { 
                    // 值变化时同步到静态成员变量
                    current_sensitivity_value_ = value; 
                },  // 使用lambda同步值到静态成员
                on_sensitivity_complete,  // 使用静态完成回调
                COLOR_TEXT_WHITE);
        }
    }
    
    PAGE_END()
}

void AreaSensitivityDetail::on_sensitivity_complete() {
    // 使用静态成员变量获取当前区域信息
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) return;
    
    // 检查是否为Serial模式
    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        return;
    }
    
    // 将uint8_t转换为Mai2_TouchArea枚举
    Mai2_TouchArea area = static_cast<Mai2_TouchArea>(current_area_index_static_);
    
    // 从静态成员变量获取最新的灵敏度值，并转换为int8_t
    int8_t sensitivity = static_cast<int8_t>(current_sensitivity_value_);
    
    // 设置区域灵敏度
    input_manager->setSerialAreaSensitivity(area, sensitivity);
}

} // namespace ui