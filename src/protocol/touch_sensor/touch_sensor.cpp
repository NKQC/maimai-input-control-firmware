#include "touch_sensor.h"

// TouchSensor基类实现文件
// 实现三个非实时状态函数，使用基类中定义的实例变量
// getCurrentTouchState作为纯虚函数由各派生类实现

/**
 * 获取已启用模块掩码
 * @return 32位掩码，高4位为ID掩码，低28位为通道启用状态
 */
uint32_t TouchSensor::getEnabledModuleMask() const {
    return enabled_module_mask_;
}

/**
 * 获取当前模块支持的通道数量
 * @return 支持的最大通道数（1-28）
 */
uint32_t TouchSensor::getSupportedChannelCount() const {
    return supported_channel_count_;
}

/**
 * 获取当前模块ID掩码
 * @return 32位掩码，仅高4位有效（1表示可用，0表示关闭）
 */
uint32_t TouchSensor::getModuleIdMask() const {
    return module_id_mask_;
}