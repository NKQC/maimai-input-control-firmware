#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include <cstdint>

namespace ui {

/**
 * 灵敏度选项枚举
 * 范围从-10到+10，默认为+2
 */
enum class SensitivityOption : int8_t {
    UNCHANGED = 0,  // 不变（仅供zone_sensitivity使用，不在UI中显示）
    MIN_VALUE = -10, // 最小值
    MAX_VALUE = 10,  // 最大值
    DEFAULT_VALUE = 2 // 默认值
};

/**
 * 灵敏度数值范围常量
 */
static constexpr int8_t SENSITIVITY_MIN = -10;
static constexpr int8_t SENSITIVITY_MAX = 10;
static constexpr int8_t SENSITIVITY_DEFAULT = 2;

/**
 * 获取灵敏度选项的文字描述
 * @param option 灵敏度选项
 * @param include_unchanged 是否包含"不变"选项的描述
 * @return 文字描述
 */
const char* getSensitivityOptionText(SensitivityOption option, bool include_unchanged = false);

/**
 * 获取所有可用的灵敏度数值（-10到+10，不包括UNCHANGED）
 * @return 灵敏度数值静态数组指针
 */
const int8_t* getSensitivityValues();

/**
 * 获取灵敏度选项的数量
 * @return 选项数量
 */
size_t getSensitivityOptionsCount();

/**
 * 触摸设置主页面构造器
 * 包含触摸IC状态和灵敏度调整菜单项
 */
class TouchSettingsMain : public PageConstructor {
public:
    TouchSettingsMain();
    virtual ~TouchSettingsMain() = default;
    
    /**
     * 渲染触摸设置主页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    static int32_t delay_value;
    
    // 校准相关静态变量
    static uint8_t progress;
    static uint8_t sensitivity_target;        // 校准灵敏度目标 (1=高敏, 2=默认, 3=低敏)

    static bool calibration_in_progress_;     // 校准是否正在进行

    /**
     * 格式化触摸IC地址为字符串
     * @param device_id_mask 触摸IC设备ID掩码
     * @return 格式化后的地址字符串
     */
    static std::string format_device_address(uint8_t device_id_mask);

    /**
     * 格式化触摸通道位图为字符串
     * @param touch_mask 触摸通道掩码
     * @param max_channels 最大通道数
     * @param enabled_channels_mask 已启用通道掩码
     * @return 格式化后的通道位图字符串
     */
    static std::string format_touch_bitmap(uint32_t touch_mask, uint8_t max_channels, uint32_t enabled_channels_mask);
    
    /**
     * 校准按钮回调函数
     */
    static void onCalibrateButtonPressed();
};

} // namespace ui