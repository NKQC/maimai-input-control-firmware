#pragma once

#include <stdint.h>
#include <string>

/**
 * TouchSensor基类 - 触摸传感器统一接口
 * 提供触摸传感器模块的统一抽象接口
 * 支持最大28个触摸通道，使用32位掩码存储状态
 * ID掩码占用最高4位（28-31位），通道掩码占用低28位（0-27位）
 */
class TouchSensor {
public:
    // 构造函数和析构函数
    TouchSensor(uint8_t max_channels) : max_channels_(max_channels), 
                                        enabled_module_mask_(0),
                                        supported_channel_count_(0),
                                        module_id_mask_(0) {}
    virtual ~TouchSensor() = default;

    // 纯虚函数接口 - 所有派生类必须实现
    
    /**
     * 获取已启用模块掩码
     * @return 32位掩码，高4位为ID掩码，低28位为通道启用状态
     */
    virtual uint32_t getEnabledModuleMask() const = 0;
    
    /**
     * 获取当前模块触摸状态
     * @return 32位掩码，高4位为ID掩码，低28位为触摸状态（1=触摸，0=未触摸）
     */
    virtual uint32_t getCurrentTouchState() const = 0;
    
    /**
     * 获取当前模块支持的通道数量
     * @return 支持的最大通道数（1-28）
     */
    virtual uint32_t getSupportedChannelCount() const = 0;
    
    /**
     * 获取当前模块ID掩码
     * @return 32位掩码，仅高4位有效（1表示可用，0表示关闭）
     */
    virtual uint32_t getModuleIdMask() const = 0;

    // 通用接口
    
    /**
     * 初始化触摸传感器
     * @return true=成功，false=失败
     */
    virtual bool init() = 0;
    
    /**
     * 反初始化触摸传感器
     */
    virtual void deinit() = 0;
    
    /**
     * 获取设备名称
     * @return 设备名称字符串
     */
    virtual std::string getDeviceName() const = 0;
    
    /**
     * 检查设备是否已初始化
     * @return true=已初始化，false=未初始化
     */
    virtual bool isInitialized() const = 0;

protected:
    uint8_t max_channels_;  // 该IC支持的最大通道数
    
    // 每个模块实例的状态变量（非实时）
    uint32_t enabled_module_mask_;     // 启用模块掩码
    uint32_t supported_channel_count_; // 支持的通道数量
    uint32_t module_id_mask_;          // 模块ID掩码
    
    /**
     * 生成ID掩码（占用最高4位）
     * @param id_bits 4位ID值（0-15）
     * @return 32位掩码，仅高4位有效
     */
    static uint32_t generateIdMask(uint8_t id_bits) {
        return (static_cast<uint32_t>(id_bits & 0x0F) << 28);
    }
    
    /**
     * 生成完整掩码（ID + 通道状态）
     * @param id_bits 4位ID值
     * @param channel_mask 28位通道掩码
     * @return 32位完整掩码
     */
    static uint32_t generateFullMask(uint8_t id_bits, uint32_t channel_mask) {
        return generateIdMask(id_bits) | (channel_mask & 0x0FFFFFFF);
    }
    
    /**
     * 从完整掩码中提取ID
     * @param full_mask 32位完整掩码
     * @return 4位ID值
     */
    static uint8_t extractIdFromMask(uint32_t full_mask) {
        return static_cast<uint8_t>((full_mask >> 28) & 0x0F);
    }
    
    /**
     * 从完整掩码中提取通道掩码
     * @param full_mask 32位完整掩码
     * @return 28位通道掩码
     */
    static uint32_t extractChannelMask(uint32_t full_mask) {
        return full_mask & 0x0FFFFFFF;
    }
};