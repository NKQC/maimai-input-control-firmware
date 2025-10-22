#include "area_sensitivity.h"

#include "../../../engine/page_construction/page_macros.h"
#include "../../../engine/page_construction/page_template.h"
#include "../../../../input_manager/input_manager.h"
#include "../../../../../protocol/mai2serial/mai2serial.h"

#include <algorithm>

namespace ui {

std::vector<int32_t> AreaSensitivity::s_area_values_;
std::vector<bool> AreaSensitivity::s_area_bound_;
bool AreaSensitivity::s_initialized_ = false;

AreaSensitivity::AreaSensitivity() {
}

std::string AreaSensitivity::get_area_name(uint8_t index) {
    // 根据 index 生成 A1-E8 名称。A:0-7, B:8-15, C:16-17(仅2个), D:18-25, E:26-33
    const char* row_names[] = {"A", "B", "C", "D", "E"};
    uint8_t row = 0;
    uint8_t col = 0;
    if (index <= 7) { row = 0; col = index + 1; }
    else if (index <= 15) { row = 1; col = (index - 8) + 1; }
    else if (index <= 17) { row = 2; col = (index - 16) + 1; }
    else if (index <= 25) { row = 3; col = (index - 18) + 1; }
    else { row = 4; col = (index - 26) + 1; }
    return std::string(row_names[row]) + std::to_string(col);
}

uint32_t AreaSensitivity::get_area_channel_id(uint8_t index) {
    auto* input_manager = InputManager::getInstance();
    if (!input_manager) return 0xFFFFFFFF;
    const auto& cfg = input_manager->getConfig();
    if (index >= sizeof(cfg.area_channel_mappings.serial_mappings) / sizeof(cfg.area_channel_mappings.serial_mappings[0])) {
        return 0xFFFFFFFF;
    }
    return cfg.area_channel_mappings.serial_mappings[index].channel;
}

void AreaSensitivity::init_area_values() {
    auto* input_manager = InputManager::getInstance();
    if (!input_manager) return;

    s_area_values_.assign(34, 0);
    s_area_bound_.assign(34, false);

    for (uint8_t i = 0; i < 34; ++i) {
        uint32_t channel_id = get_area_channel_id(i);
        if (channel_id == 0xFFFFFFFF) {
            s_area_bound_[i] = false;
            s_area_values_[i] = 0;
            continue;
        }
        s_area_bound_[i] = true;
        // 从32位物理地址解码设备掩码和通道号（避免调用可能为private的接口）
        uint8_t device_mask = static_cast<uint8_t>((channel_id >> 24) & 0xFF);
        uint32_t channel_bitmap = channel_id & 0x00FFFFFF;
        uint8_t ch = 0xFF;
        for (uint8_t bit = 0; bit < 24; ++bit) {
            if (channel_bitmap & (1UL << bit)) { ch = bit; break; }
        }
        if (ch == 0xFF) {
            s_area_values_[i] = 0;
            s_area_bound_[i] = false;
            continue;
        }
        // 读取当前通道灵敏度（0-99范围）
        uint8_t sens = input_manager->getDeviceChannelSensitivity(device_mask, ch);
        s_area_values_[i] = static_cast<int32_t>(std::clamp<int>(sens, 0, 99));
    }

    s_initialized_ = true;
}

void AreaSensitivity::on_sensitivity_complete() {
    auto* input_manager = InputManager::getInstance();
    if (!input_manager) return;

    // 将缓存值写回到每个已绑定区域
    for (uint8_t i = 0; i < 34; ++i) {
        if (!s_area_bound_[i]) continue;
        int32_t v = std::clamp<int32_t>(s_area_values_[i], 0, 99);
        input_manager->setSerialAreaSensitivity(static_cast<Mai2_TouchArea>(i + 1), static_cast<uint8_t>(v));
    }
}

void AreaSensitivity::render(PageTemplate& page_template) {
    auto* input_manager = InputManager::getInstance();

    PAGE_START()
    SET_TITLE("按区域调整灵敏度", COLOR_WHITE)
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    if (!input_manager) {
        ADD_TEXT("InputManager未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }

    if (input_manager->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        ADD_TEXT("此功能仅在Serial模式下可用", COLOR_YELLOW, LineAlign::CENTER)
        PAGE_END()
        return;
    }

    if (!s_initialized_) {
        init_area_values();
    }

    ADD_TEXT("范围: 0-99 (越大越灵敏)", COLOR_TEXT_WHITE, LineAlign::LEFT)

    // 遍历所有区域，显示对应的灵敏度或未绑定提示
    for (uint8_t i = 0; i < 34; ++i) {
        std::string name = get_area_name(i);
        uint32_t ch_id = get_area_channel_id(i);
        if (ch_id == 0xFFFFFFFF || !s_area_bound_[i]) {
            ADD_TEXT(name + "：未绑定", COLOR_RED, LineAlign::LEFT)
            continue;
        }
        // 对应区域的灵敏度设置，使用统一完成回调
        ADD_INT_SETTING(&s_area_values_[i], 0, 99, name.c_str(), ("AREA_" + name).c_str(), nullptr, AreaSensitivity::on_sensitivity_complete, COLOR_TEXT_WHITE)
    }

    PAGE_END()
}

} // namespace ui