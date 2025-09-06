#pragma once

#include "src/service/ui_manager/engine/page_construction/page_constructor.h"
#include "src/service/input_manager/input_manager.h"
#include "src/protocol/touch_sensor/ad7147/ad7147.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 位域配置辅助结构体
 * 用于静态页面构造和位域转换
 */
struct BitfieldHelper {
    // AFE偏移位域配置
    struct {
        int32_t neg_afe_offset = 0;      // 负AFE偏移 (0-63)
        int32_t neg_afe_swap = 0;        // 负AFE交换 (0-1)
        int32_t pos_afe_offset = 0;      // 正AFE偏移 (0-63)
        int32_t pos_afe_swap = 0;        // 正AFE交换 (0-1)
    } afe_offset;
    
    // 灵敏度位域配置
    struct {
        int32_t neg_threshold_sensitivity = 0;  // 负阈值灵敏度 (0-15)
        int32_t neg_peak_detect = 0;           // 负峰值检测 (0-5)
        int32_t pos_threshold_sensitivity = 0;  // 正阈值灵敏度 (0-15)
        int32_t pos_peak_detect = 0;           // 正峰值检测 (0-5)
    } sensitivity;
    BitfieldHelper() {};
    // 从StageConfig加载位域值
    void loadFromStageConfig(const StageConfig& config);
    
    // 写回到StageConfig
    void writeToStageConfig(StageConfig& config) const;
};

/**
 * AD7147设备自定义设置页面构造器
 * 允许设置指定stage的全部配置并实时生效
 * 提供当前通道CDC的读取值
 */
class AD7147CustomSettings : public PageConstructor {
public:
    AD7147CustomSettings();
    virtual ~AD7147CustomSettings() = default;
    
    /**
     * 渲染AD7147自定义设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 接收跳转时传递的字符串参数（设备名称）
     * @param str 设备名称
     */
    virtual void jump_str(const std::string& str) override;
    
private:
    static std::string device_name_;                    // 设备名称
    static int32_t current_stage_;                      // 当前选择的阶段
    static StageConfig current_config_;                 // 当前阶段配置
    static BitfieldHelper bitfield_helper_;            // 位域配置辅助结构体
    static uint16_t current_cdc_value_;                 // 当前CDC值
    static bool config_loaded_;                         // 配置是否已加载
    static bool channel_triggered_;                     // 当前通道触发状态
    
    // 一键拉偏移功能相关
    static bool auto_offset_active_;                   // 自动偏移校准是否激活
    static uint8_t auto_offset_progress_;              // 自动偏移校准进度 (0-100)
    
    /**
     * 获取AD7147设备实例
     * @return AD7147设备指针，失败返回nullptr
     */
    static AD7147* getAD7147Device();
    
    /**
     * 加载当前阶段配置和状态数据（合并函数，减少重复开销）
     */
    static void loadStageDataAndStatus();
    
    /**
     * 应用配置到硬件
     */
    static void applyConfig();
    
    /**
     * 重置当前阶段配置为默认值
     */
    static void resetToDefault();
    
    /**
     * 从AD7147设备读取当前阶段的默认配置并应用
     */
    static void resetCurrentStageFromDevice();
    
    /**
     * 启动一键拉偏移校准
     */
    static void startAutoOffsetCalibration();
    
    /**
     * 更新自动偏移校准状态和进度
     */
    static void updateAutoOffsetStatus();
    
    // 静态回调函数
    static void onStageChange();
    static void onConfigComplete();
    static void onAutoOffsetButtonClick();
};

} // namespace ui