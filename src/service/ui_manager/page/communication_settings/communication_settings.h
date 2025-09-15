#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include "../../../../protocol/mai2serial/mai2serial.h"
#include "../../../light_manager/light_manager.h"
#include "../../../../hal/uart/hal_uart.h"
#include <string>
#include <functional>
#include <cstdint>

namespace ui {

/**
 * 通信设置页面构造器
 * 提供Mai2Serial和LightManager波特率设置、串口延迟设置、键盘映射开关等功能
 */
class CommunicationSettings : public PageConstructor {
public:
    CommunicationSettings();
    virtual ~CommunicationSettings() = default;
    
    /**
     * 渲染通信设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    
    // 当前设置状态
    static uint32_t current_mai2serial_baud_;
    static uint32_t current_lightmanager_baud_;
    static uint8_t current_serial_delay_;
    static bool current_keyboard_mapping_enabled_;
    
    // Serial模式新功能设置状态
    static bool current_send_only_on_change_;
    static uint8_t current_data_aggregation_delay_;
    static uint8_t current_extra_send_count_;
    
    // 波特率选择器索引
    static size_t mai2serial_baud_index_;
    static size_t lightmanager_baud_index_;
    
    // 静态回调函数
    static void onMai2SerialBaudRateChange(JoystickState state);
    static void onLightManagerBaudRateChange(JoystickState state);
    static void onSerialDelayChange(JoystickState state);
    static void onKeyboardMappingToggle();
    
    // Serial模式新功能回调函数
    static void onSendOnlyOnChangeToggle();
    static void onDataAggregationDelayChange(JoystickState state);
    static void onExtraSendCountChange(JoystickState state);
    
    // 频率限制回调函数
    static void onRateLimitEnabledToggle();
    static void onRateLimitFrequencyChange(JoystickState state);
    
    // 辅助函数
    static void loadCurrentSettings();
    static void ApplySettings();
    static size_t findBaudRateIndex(uint32_t baud_rate);
    static std::string formatBaudRateText(uint32_t baud_rate);
    static std::string formatDelayText(uint8_t delay_ms);
    static std::string formatKeyboardMappingText(bool enabled);
    
    // Serial模式新功能辅助函数
    static std::string formatSendOnlyOnChangeText(bool enabled);
    static std::string formatDataAggregationDelayText(uint8_t delay_ms);
    static std::string formatExtraSendCountText(uint8_t count);
    static bool isSerialMode();
};

} // namespace ui