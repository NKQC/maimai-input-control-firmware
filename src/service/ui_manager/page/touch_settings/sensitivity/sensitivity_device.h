#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 单个IC的灵敏度设置页面构造器
 * 显示每个通道的灵敏度值并支持调整
 */
class SensitivityDevice : public PageConstructor {
public:
    SensitivityDevice();
    virtual ~SensitivityDevice() = default;
    
    /**
     * 渲染单个IC的灵敏度设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 接收跳转时传递的字符串参数（设备名称）
     * @param str 设备名称
     */
    virtual void jump_str(const std::string& str) override;
    
private:
    static std::string device_name_;  // 通过jump_str接收的设备名称
    static std::vector<int32_t> cached_sensitivity_values_;  // 缓存的灵敏度值
    static TouchDeviceMapping cached_mapping_;  // 缓存的设备映射
    static bool mapping_cached_;  // 映射是否已缓存（改为静态变量）
    
    /**
     * 获取指定设备的映射信息
     * @param device_name 设备名称
     * @param mapping 输出的设备映射信息
     * @return 是否成功获取
     */
    bool get_device_mapping(const std::string& device_name, TouchDeviceMapping& mapping);
    
    /**
     * 生成通道设置的唯一标识符
     * @param device_name 设备名称
     * @param channel 通道号
     * @return 设置标识符
     */
    std::string generate_channel_setting_id(const std::string& device_name, uint8_t channel);
    
    /**
     * 灵敏度设置完成回调
     * @param channel 通道号
     */
    static void on_sensitivity_complete();
    
    /**
     * 初始化缓存的灵敏度值
     */
    void init_cached_values();
};

} // namespace ui