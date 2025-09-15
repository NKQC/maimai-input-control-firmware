#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include "../touch_settings_main.h"
#include <cstdint>
#include <string>
#include <vector>

namespace ui {

// 前向声明
enum class SensitivityOption : int8_t;

/**
 * 按分区设置灵敏度页面构造器
 * 仅在Serial模式中出现，支持A-E五个分区的灵敏度选择
 */
class ZoneSensitivity : public PageConstructor {
public:
    ZoneSensitivity();
    virtual ~ZoneSensitivity() = default;
    
    /**
     * 渲染按分区设置灵敏度页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    /**
     * 分区绑定信息结构体
     */
    struct ZoneBindingInfo {
        std::string zone_name;           // 分区名称 (A, B, C, D, E)
        std::vector<uint32_t> bitmaps;   // 该分区的设备通道bitmap列表
        int8_t target_sensitivity_target;      // 目标灵敏度设置 (0=不变, 1=低敏, 2=中敏, 3=高敏, 4=超敏)
        bool has_bindings;               // 是否有绑定
        bool has_modified;               // 是否有修改
        
        ZoneBindingInfo() : target_sensitivity_target(SENSITIVITY_DEFAULT), has_bindings(false), has_modified(false) {}
    };
    
    /**
     * 获取所有分区的绑定信息
     * @return 分区绑定信息数组
     */
    std::vector<ZoneBindingInfo> getZoneBindingInfo();
    
    /**
     * 根据Mai2_TouchArea获取分区索引
     * @param area Mai2触摸区域
     * @return 分区索引 (0=A, 1=B, 2=C, 3=D, 4=E, -1=无效)
     */
    int getZoneIndex(Mai2_TouchArea area);
    
    /**
     * 获取分区名称
     * @param zone_index 分区索引
     * @return 分区名称
     */
    std::string getZoneName(uint8_t zone_index);
    
    /**
     * 设置分区目标灵敏度
     * @param zone_index 分区索引
     * @param target_sensitivity_target 目标灵敏度值 (0=不变, 1=低敏, 2=中敏, 3=高敏, 4=超敏)
     */
    void setZoneTargetSensitivity(uint8_t zone_index, uint8_t target_sensitivity);

    // 静态回调函数
    static void onZoneTargetSensitivityChange(uint8_t zone_index, SensitivityOption option);
    static void onSubmitSpecialCalibration();
    static void onStartSpecialCalibration();
    
    // 通用的分区灵敏度回调函数
    static void onZoneSensitivityChange(JoystickState state, uint8_t zone_index);
    
    // 静态状态变量
    static std::vector<ZoneBindingInfo> s_zone_info_;
};

} // namespace ui