#include "zone_sensitivity.h"
#include "../touch_settings_main.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../../protocol/mai2serial/mai2serial.h"
#include <cstdio>
#include <cstring>

namespace ui {

// 静态成员变量定义
std::vector<ZoneSensitivity::ZoneBindingInfo> ZoneSensitivity::s_zone_info_;


ZoneSensitivity::ZoneSensitivity() {
    // 构造函数无需特殊初始化
}

void ZoneSensitivity::render(PageTemplate& page_template) {
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        PAGE_START()
        SET_TITLE("按分区设置灵敏度", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 检查是否为Serial模式
    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        PAGE_START()
        SET_TITLE("按分区设置灵敏度", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("此功能仅在Serial模式下可用", COLOR_YELLOW, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    if (s_zone_info_.empty()) {
        s_zone_info_ = getZoneBindingInfo();
    }
    
    // 检查绑区是否完整（所有5个分区都有绑定）
    bool binding_complete = true;
    for (const auto& zone : s_zone_info_) {
        if (!zone.has_bindings) {
            binding_complete = false;
            break;
        }
    }
    
    PAGE_START()
    SET_TITLE("按分区设置灵敏度", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    static char s_sensitivity_text_buffer[64];

    if (!binding_complete) {
        // 绑区不完整，拒绝设置
        ADD_TEXT("绑区不完整", COLOR_WHITE, LineAlign::CENTER)
        ADD_TEXT("完成所有分区绑定后再设置", COLOR_WHITE, LineAlign::CENTER)
    } else {
        // 绑区完整，显示分区目标灵敏度选择器
        ADD_TEXT("设置各分区目标灵敏度", COLOR_WHITE, LineAlign::CENTER)
        
        // 显示各分区的目标灵敏度选择器
        for (size_t i = 0; i < s_zone_info_.size(); ++i) {
            snprintf(s_sensitivity_text_buffer, sizeof(s_sensitivity_text_buffer), "%s区目标灵敏度: %s", 
                        s_zone_info_[i].zone_name.c_str(), getSensitivityOptionText(static_cast<SensitivityOption>(s_zone_info_[i].target_sensitivity_target), true));
            ADD_SIMPLE_SELECTOR(s_sensitivity_text_buffer, [i](JoystickState state) {
                onZoneSensitivityChange(state, i);
            }, COLOR_WHITE);
        }
        
        // 添加发起特殊校准按钮
        ADD_BUTTON("发起按区校准", []() {
             onStartSpecialCalibration();
        }, COLOR_WHITE, LineAlign::CENTER);
    }
    
    PAGE_END()
}

std::vector<ZoneSensitivity::ZoneBindingInfo> ZoneSensitivity::getZoneBindingInfo() {
    std::vector<ZoneBindingInfo> zones(5); // A, B, C, D, E五个分区
    
    // 初始化分区名称
    for (uint8_t i = 0; i < 5; i++) {
        zones[i].zone_name = getZoneName(i);
    }
    
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return zones;
    }
    
    // 获取所有设备状态
    uint8_t device_count = input_manager->get_device_count();
    if (device_count == 0) {
        return zones;
    }
    
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    // 遍历所有Mai2区域，检查绑定情况
    for (uint8_t area_idx = 1; area_idx <= 34; area_idx++) {
        Mai2_TouchArea area = static_cast<Mai2_TouchArea>(area_idx);
        uint8_t zone_idx = getZoneIndex(area);
        
        if (zone_idx < 0 || zone_idx >= 5) continue;
        
        // 检查每个设备的每个通道是否绑定到此区域
        for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
            const auto& device = device_status[dev_idx];
            if (!device.is_connected) continue;
            
            uint8_t device_id_mask = device.touch_device.device_id_mask;
            
            for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
                Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
                
                if (mapped_area == area) {
                    // 找到绑定，创建bitmap
                    uint32_t bitmap = (static_cast<uint32_t>(device_id_mask) << 24) | (1 << ch);
                    
                    // 检查是否已存在相同设备的bitmap
                    bool found_device = false;
                    for (auto& existing_bitmap : zones[zone_idx].bitmaps) {
                        if ((existing_bitmap >> 24) == device_id_mask) {
                            // 合并到现有bitmap
                            existing_bitmap |= (1 << ch);
                            found_device = true;
                            break;
                        }
                    }
                    
                    if (!found_device) {
                        // 添加新的设备bitmap
                        zones[zone_idx].bitmaps.push_back(bitmap);
                    }
                    
                    zones[zone_idx].has_bindings = true;
                }
            }
        }
    }
    
    return zones;
}

int ZoneSensitivity::getZoneIndex(Mai2_TouchArea area) {
    if (area >= MAI2_AREA_A1 && area <= MAI2_AREA_A8) {
        return 0; // A区
    } else if (area >= MAI2_AREA_B1 && area <= MAI2_AREA_B8) {
        return 1; // B区
    } else if (area >= MAI2_AREA_C1 && area <= MAI2_AREA_C2) {
        return 2; // C区
    } else if (area >= MAI2_AREA_D1 && area <= MAI2_AREA_D8) {
        return 3; // D区
    } else if (area >= MAI2_AREA_E1 && area <= MAI2_AREA_E8) {
        return 4; // E区
    }
    return -1; // 无效区域
}

std::string ZoneSensitivity::getZoneName(uint8_t zone_index) {
    switch (zone_index) {
        case 0: return "A";
        case 1: return "B";
        case 2: return "C";
        case 3: return "D";
        case 4: return "E";
        default: return "未知";
    }
}

void ZoneSensitivity::setZoneTargetSensitivity(uint8_t zone_index, uint8_t target_sensitivity) {
    if (zone_index >= s_zone_info_.size()) {
        return;
    }
    
    s_zone_info_[zone_index].target_sensitivity_target = target_sensitivity;
    
    // 如果选择了非UNCHANGED选项，立即更新InputManager
    if (target_sensitivity != static_cast<uint8_t>(SensitivityOption::UNCHANGED)) {
        // 立即调用InputManager接口更新校准参数
        InputManager* input_manager = InputManager::getInstance();
        if (input_manager) {
            const auto& zone = s_zone_info_[zone_index];
            
            // 对该分区的所有设备通道设置特殊校准目标灵敏度
            for (const auto& bitmap : zone.bitmaps) {
                input_manager->setCalibrationTargetByBitmap(bitmap, target_sensitivity);
            }
        }
    }
}

// 静态回调函数实现
void ZoneSensitivity::onZoneTargetSensitivityChange(uint8_t zone_index, SensitivityOption option) {
    uint8_t target_sensitivity_value = static_cast<uint8_t>(option);
    
    ZoneSensitivity instance;
    instance.setZoneTargetSensitivity(zone_index, target_sensitivity_value);
}

void ZoneSensitivity::onSubmitSpecialCalibration() {
    InputManager* input_manager = InputManager::getInstance();
    if (input_manager) {
        input_manager->calibrateSelectedChannels();
    }
}

void ZoneSensitivity::onStartSpecialCalibration() {
    InputManager* input_manager = InputManager::getInstance();
    if (input_manager) {
        input_manager->calibrateSelectedChannels();
    }
}

// 通用的分区灵敏度调整回调函数实现
void ZoneSensitivity::onZoneSensitivityChange(JoystickState state, uint8_t zone_index) {
    if (zone_index < 0 || zone_index >= s_zone_info_.size()) {
        return;
    }
    
    switch (state) {
    case JoystickState::UP: {
        s_zone_info_[zone_index].target_sensitivity_target++;
        if (s_zone_info_[zone_index].target_sensitivity_target > int8_t(SensitivityOption::ULTRA)) {
            s_zone_info_[zone_index].target_sensitivity_target = int8_t(SensitivityOption::ULTRA);
        }
        onZoneTargetSensitivityChange(zone_index, static_cast<SensitivityOption>(s_zone_info_[zone_index].target_sensitivity_target));
        break;
    }
    case JoystickState::DOWN: {
        s_zone_info_[zone_index].target_sensitivity_target--;
        if (s_zone_info_[zone_index].target_sensitivity_target < int8_t(SensitivityOption::LOW)) {
            s_zone_info_[zone_index].target_sensitivity_target = int8_t(SensitivityOption::LOW);
        }
        onZoneTargetSensitivityChange(zone_index, static_cast<SensitivityOption>(s_zone_info_[zone_index].target_sensitivity_target));
        break;
    }
    default:
        return;
    }
}
} // namespace ui