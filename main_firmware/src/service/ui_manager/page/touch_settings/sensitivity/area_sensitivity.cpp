#include "area_sensitivity.h"
#include "../../../ui_manager.h"
#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../input_manager/input_manager.h"
#include "../../binding_settings/binding_info.h"
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace ui {

// 静态成员定义
AreaSensitivity::ZoneInfo AreaSensitivity::s_zone_infos_[5];  // 固定5个区域组 A-E
bool AreaSensitivity::s_initialized_ = false;
uint8_t AreaSensitivity::s_current_zone_index_ = 0;
int32_t AreaSensitivity::s_current_area_index_ = -1;

// 当前编辑区域的静态数据存储
int32_t AreaSensitivity::s_current_sensitivity_value_ = 0;
std::string AreaSensitivity::s_current_area_name_ = "";
uint8_t AreaSensitivity::s_current_device_mask_ = 0;
uint8_t AreaSensitivity::s_current_channel_ = 0;

AreaSensitivity::AreaSensitivity() {
    // 构造函数实现
}

void AreaSensitivity::init_zone_infos() {
    // 初始化固定大小的数组
    for (uint8_t i = 0; i < 5; i++) {
        s_zone_infos_[i].zone_name = getZoneName(i);
        s_zone_infos_[i].has_any_bindings = false;
    }
    
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
}

uint8_t AreaSensitivity::getZoneIndex(uint8_t area_index) {
    // 根据Mai2_TouchArea枚举映射到区域索引
    if (area_index >= 1 && area_index <= 8) return 0;   // A1-A8 -> A区 (0)
    if (area_index >= 9 && area_index <= 16) return 1;  // B1-B8 -> B区 (1)
    if (area_index >= 17 && area_index <= 18) return 2; // C1-C2 -> C区 (2)
    if (area_index >= 19 && area_index <= 26) return 3; // D1-D8 -> D区 (3)
    if (area_index >= 27 && area_index <= 34) return 4; // E1-E8 -> E区 (4)
    return 0; // 默认A区
}

std::string AreaSensitivity::getAreaName(uint8_t area_index) {
    if (area_index >= 1 && area_index <= 34) {
        return mai2_area_names[area_index];  // 直接使用area_index作为索引，因为mai2_area_names[0]是"NONE"
    }
    return "未知区域";
}

uint32_t AreaSensitivity::getAreaChannelId(uint8_t area_index) {
    // 由于BindingInfo::get_area_channel_id是私有方法，我们直接通过InputManager获取映射信息
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return 0xFFFFFFFF;
    }
    
    // 遍历所有设备和通道，查找映射到指定区域的通道
    uint8_t device_count = input_manager->get_device_count();
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
        const auto& device = device_status[dev_idx];
        if (!device.is_connected) continue;
        
        uint8_t device_id_mask = device.touch_device.device_id_mask;
        
        for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
            Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
            
            if (mapped_area == static_cast<Mai2_TouchArea>(area_index)) {
                // 构造通道ID：高8位为设备掩码，低8位为通道号
                return (static_cast<uint32_t>(device_id_mask) << 24) | ch;
            }
        }
    }
    
    return 0xFFFFFFFF; // 未找到绑定
}

std::string AreaSensitivity::getZoneName(uint8_t zone_index) {
    const char* zone_names[] = {"A", "B", "C", "D", "E"};
    if (zone_index < 5) {
        return zone_names[zone_index];
    }
    return "未知";
}


void AreaSensitivity::jump_str(const std::string& str) {
    current_area_param_ = str;
}

void AreaSensitivity::render(PageTemplate& page_template) {
    // 加载区域数据
    loadAreaData();
    
    PAGE_START()
    SET_TITLE("按区域设置灵敏度", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 检查是否有任何绑定的区域
    bool has_any_bindings = false;
    for (uint8_t i = 0; i < 5; i++) {
        if (s_zone_infos_[i].has_any_bindings) {
            has_any_bindings = true;
            break;
        }
    }
    
    if (!has_any_bindings) {
        ADD_TEXT("未检测到绑定的区域", COLOR_WHITE, LineAlign::CENTER)
        ADD_TEXT("请先完成区域绑定", COLOR_WHITE, LineAlign::CENTER)
    } else {
        ADD_TEXT("选择要调整的区域组", COLOR_WHITE, LineAlign::CENTER)
        
        // 显示各区域组选项
        for (uint8_t zone_idx = 0; zone_idx < 5; zone_idx++) {
            if (s_zone_infos_[zone_idx].has_any_bindings) {
                // 构建显示文本
                static char zone_text[64];
                std::string zone_name = getZoneName(zone_idx);
                
                // 统计该zone中绑定的区域数量
                uint8_t bound_count = 0;
                for (uint8_t area_idx = 0; area_idx < 8; area_idx++) {
                    if (s_zone_infos_[zone_idx].areas[area_idx].is_bound) {
                        bound_count++;
                    }
                }
                
                snprintf(zone_text, sizeof(zone_text), "%s区 (%d个绑定)", 
                        zone_name.c_str(), bound_count);
                
                // 传递区域组索引作为参数
                static char zone_param[8];
                snprintf(zone_param, sizeof(zone_param), "%d", zone_idx);
                
                ADD_MENU_WITH_STR(zone_text, "area_sensitivity_zone", zone_param, COLOR_TEXT_WHITE);
            }
        }
    }
    
    PAGE_END()
}

void AreaSensitivity::getAreaBindingInfo(AreaInfo* areas, uint8_t max_areas, uint8_t* area_count) {
    *area_count = 0;
    
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
    
    // 获取所有设备状态
    uint8_t device_count = input_manager->get_device_count();
    if (device_count == 0) {
        return;
    }
    
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    // 遍历所有Mai2区域，检查绑定情况
    for (uint8_t area_idx = 1; area_idx <= 34 && *area_count < max_areas; area_idx++) {
        Mai2_TouchArea area = static_cast<Mai2_TouchArea>(area_idx);
        
        // 检查每个设备的每个通道是否绑定到此区域
        for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
            const auto& device = device_status[dev_idx];
            if (!device.is_connected) continue;
            
            uint8_t device_id_mask = device.touch_device.device_id_mask;
            
            for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
                Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
                
                if (mapped_area == area) {
                    // 找到绑定的区域
                    AreaInfo& area_info = areas[*area_count];
                    area_info.area_index = area_idx;
                    area_info.area_name = mai2_area_names[area_idx - 1]; // 数组索引从0开始
                    area_info.device_mask = device_id_mask;
                    area_info.channel = ch;
                    area_info.current_value = input_manager->getSensitivity(device_id_mask, ch);
                    area_info.has_modified = false; // 初始化为未修改
                    
                    (*area_count)++;
                    break;
                }
            }
            if (*area_count >= max_areas) break;
        }
    }
}

void AreaSensitivity::renderAreaDetail(PageTemplate& page_template) {
    // 获取跳转参数（区域名称）
    std::string area_name = current_area_param_;
    
    // 根据区域名称查找对应的区域信息
    static AreaInfo area_infos[34];  // 最多34个区域
    uint8_t area_count = 0;
    getAreaBindingInfo(area_infos, 34, &area_count);
    
    AreaInfo* current_area = nullptr;
    
    for (uint8_t i = 0; i < area_count; i++) {
        if (area_infos[i].area_name == area_name) {
            current_area = &area_infos[i];
            break;
        }
    }
    
    PAGE_START()
    
    if (!current_area) {
        SET_TITLE("区域未找到", COLOR_WHITE)
        ADD_TEXT("指定的区域未找到", COLOR_WHITE, LineAlign::CENTER)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    } else {
        // 只在区域切换时更新静态数据，避免每帧重复设置
        if (s_current_area_name_ != current_area->area_name) {
            s_current_area_name_ = current_area->area_name;
            s_current_sensitivity_value_ = current_area->current_value;
            s_current_device_mask_ = current_area->device_mask;
            s_current_channel_ = current_area->channel;
        }
        
        static char title_text[64];
        snprintf(title_text, sizeof(title_text), "%s 灵敏度设置", s_current_area_name_.c_str());
        SET_TITLE(title_text, COLOR_WHITE)
        
        // 显示当前灵敏度值
        static char current_text[64];
        snprintf(current_text, sizeof(current_text), "当前灵敏度: %ld", s_current_sensitivity_value_);
        ADD_TEXT(current_text, COLOR_WHITE, LineAlign::CENTER)
        
        // 灵敏度调整控件 - 使用静态变量和完成回调
        ADD_INT_SETTING(&s_current_sensitivity_value_, 0, 99, "灵敏度", "调整灵敏度", 
                       [](int32_t value) { 
                           // 值变化时同步到静态成员变量
                           s_current_sensitivity_value_ = value; 
                       }, 
                       on_sensitivity_complete,  // 使用完成回调
                       COLOR_TEXT_WHITE);
        
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    }
    
    PAGE_END()
}

void AreaSensitivity::loadAreaData() {
    // 初始化所有zone信息
    for (uint8_t i = 0; i < 5; i++) {
        s_zone_infos_[i].zone_index = i;
        s_zone_infos_[i].has_any_bindings = false;
        
        // 初始化每个zone的8个区域
        for (uint8_t j = 0; j < 8; j++) {
            s_zone_infos_[i].areas[j] = AreaInfo();
        }
    }
    
    // 获取InputManager实例
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) {
        return;
    }
    
    // 获取所有设备状态
    uint8_t device_count = input_manager->get_device_count();
    if (device_count == 0) {
        return;
    }
    
    InputManager::TouchDeviceStatus device_status[device_count];
    input_manager->get_all_device_status(device_status);
    
    // 遍历所有区域 (1-34对应Mai2_TouchArea枚举)
    for (uint8_t area_idx = 1; area_idx <= 34; area_idx++) {
        uint8_t zone_idx = getZoneIndex(area_idx);  // 使用正确的zone映射函数
        uint8_t area_in_zone;
        
        // 根据Mai2_TouchArea枚举计算区域在zone内的索引
        if (area_idx >= 1 && area_idx <= 8) {
            area_in_zone = area_idx - 1;  // A1-A8 -> 0-7
        } else if (area_idx >= 9 && area_idx <= 16) {
            area_in_zone = area_idx - 9;  // B1-B8 -> 0-7
        } else if (area_idx >= 17 && area_idx <= 18) {
            area_in_zone = area_idx - 17; // C1-C2 -> 0-1
        } else if (area_idx >= 19 && area_idx <= 26) {
            area_in_zone = area_idx - 19; // D1-D8 -> 0-7
        } else if (area_idx >= 27 && area_idx <= 34) {
            area_in_zone = area_idx - 27; // E1-E8 -> 0-7
        } else {
            continue; // 跳过无效区域
        }
        
        AreaInfo& area_info = s_zone_infos_[zone_idx].areas[area_in_zone];
        area_info.area_index = area_idx;
        area_info.name = getAreaName(area_idx);
        
        // 检查是否有绑定到此区域的设备通道
        bool found_binding = false;
        for (uint8_t dev_idx = 0; dev_idx < device_count; dev_idx++) {
            const auto& device = device_status[dev_idx];
            if (!device.is_connected) continue;
            
            uint8_t device_id_mask = device.touch_device.device_id_mask;
            
            for (uint8_t ch = 0; ch < device.touch_device.max_channels; ch++) {
                Mai2_TouchArea mapped_area = input_manager->getSerialMapping(device_id_mask, ch);
                
                if (mapped_area == static_cast<Mai2_TouchArea>(area_idx)) {
                    // 找到绑定的区域
                    area_info.is_bound = true;
                    area_info.device_mask = device_id_mask;
                    area_info.channel = ch;
                    area_info.supports_sensitivity = true;
                    area_info.is_relative_mode = false;
                    area_info.current_value = input_manager->getSensitivity(device_id_mask, ch);
                    s_zone_infos_[zone_idx].has_any_bindings = true;
                    found_binding = true;
                    break;
                }
            }
            if (found_binding) break;
        }
        
        if (!found_binding) {
            area_info.is_bound = false;
            area_info.supports_sensitivity = false;
        }
    }
}
 
void AreaSensitivity::on_sensitivity_change(int32_t new_value) {
    // 实时更新灵敏度值
    if (s_current_area_index_ >= 0) {
        // 更新缓存的当前值
        // 这里可以添加实时预览逻辑
    }
}

void AreaSensitivity::on_sensitivity_complete() {
    // 使用静态成员变量获取当前区域信息
    InputManager* input_manager = InputManager::getInstance();
    if (!input_manager) return;
    
    // 检查是否有有效的区域数据
    if (s_current_area_name_.empty() || s_current_device_mask_ == 0) {
        return;
    }
    
    // 将灵敏度值转换为int8_t
    int8_t sensitivity = static_cast<int8_t>(s_current_sensitivity_value_);
    
    // 设置设备通道灵敏度
    input_manager->setSensitivity(s_current_device_mask_, s_current_channel_, sensitivity);
}

void AreaSensitivity::on_zone_select(uint8_t zone_index) {
    s_current_zone_index_ = zone_index;
}

} // namespace ui