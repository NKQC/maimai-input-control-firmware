#include "gtx312l.h"
#include <cstring>
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../../protocol/usb_serial_logs/usb_serial_logs.h"

// 静态成员变量定义
uint8_t GTX312L::_async_read_buffer[2];

// GTX312L构造函数
GTX312L::GTX312L(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : TouchSensor(GTX312L_MAX_CHANNELS), i2c_hal_(i2c_hal), i2c_bus_(i2c_bus), 
      device_addr_(device_addr), i2c_device_address_(device_addr), initialized_(false),
      i2c_bus_enum_(i2c_bus), enabled_channels_mask_(0) {
    // 生成模块掩码：bit7=I2C总线编号，bit6-0=I2C地址
    module_name = "GTX312L";
    module_mask_ = generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    
    // 设置GTX312L的功能标志：支持一般灵敏度设置(位0)
    sensor_flag_.supports_general_sensitivity = true;
    sensor_flag_.sensitivity_relative_mode = false;
    sensor_flag_.sensitivity_private_mode = false;
    sensor_flag_.supports_calibration = false;
    sensor_flag_.reserved = 0;
}

// GTX312L析构函数
GTX312L::~GTX312L() {
    deinit();
}

// 初始化触摸控制器
bool GTX312L::init() {
    if (!i2c_hal_ || initialized_) {
        return false;
    }
    
    // 简单的设备存在性检查 - 读取芯片ID寄存器
    uint8_t chip_id;
    if (!read_register(GTX312L_REG_CHIPADDR_VER, chip_id)) {
        USB_LOG_TAG_WARNING("GTX312L", "Chip Init failed %d", i2c_device_address_);
        return false;
    }
    USB_LOG_TAG_WARNING("GTX312L", "Chip Init Success %d", chip_id);
    uint8_t ret = 0;
    // 下面是默认设置
    ret |= !write_register(GTX312L_REG_MON_RST, 1);  // 自复位
    ret |= !write_register(GTX312L_REG_SLEEP, 0);  // 关闭睡眠模式
    ret |= !write_register(GTX312L_REG_I2C_PU_DIS, 1);  // I2C上拉
    ret |= !write_register(GTX312L_REG_WRITE_LOCK, 0x5A);  // I2C上拉
    
    ret |= !write_register(GTX312L_REG_INT_TOUCH_MODE, 0x01);  // 不关心中断 只启用多点触摸
    ret |= !write_register(GTX312L_REG_EXP_CONFIG, 0x00);  // 关闭触摸超时
    ret |= !write_register(GTX312L_REG_CAL_TIME, 0x00);  // 单周期校准 我们依靠外部电路的稳定确保采样正确
    // 关闭空闲时间
    ret |= !write_register(GTX312L_REG_SEN_IDLE_TIME, 0x00); 
    ret |= !write_register(GTX312L_REG_SEN_IDLE_SUFFIX, 0x00);
    ret |= !write_register(GTX312L_REG_BUSY_TO_IDLE, 0x00);

    ret |= !write_register(GTX312L_REG_I2B_MODE, 0x00);  // 手动进入BUSY模式
    ret |= !write_register(GTX312L_REG_SLIDE_MODE, 0x00);  // 禁用滑动模式
    
    if (ret) {
        return false;
    }
    
    // 读取当前启用的通道掩码
    uint8_t ch_enable_l, ch_enable_h;
    if (read_register(GTX312L_REG_CH_ENABLE_L, ch_enable_l) &&
        read_register(GTX312L_REG_CH_ENABLE_H, ch_enable_h)) {
        enabled_channels_mask_ = static_cast<uint32_t>(ch_enable_l) | 
                                (static_cast<uint32_t>(ch_enable_h & 0x0F) << 8);
    } else {
        enabled_channels_mask_ = 0x0FFF;  // 默认全部启用
    }
    
    initialized_ = true;
    return true;
}

// 清理触摸控制器
void GTX312L::deinit() {
    initialized_ = false;
}

// 读取设备信息
bool GTX312L::read_device_info(GTX312L_DeviceInfo& info) {
    if (!initialized_) {
        return false;
    }
    
    info.i2c_address = i2c_device_address_;
    info.is_valid = true;
    
    return true;
}

void GTX312L::sample(async_touchsampleresult callback) {
    if (!callback) return;
    
    // 异步读取两个寄存器的数据
    i2c_hal_->read_register_async(i2c_device_address_, GTX312L_REG_TOUCH_STATUS_L, _async_read_buffer, 2, [this, callback](bool success) {
        TouchSampleResult result = {0, 0};
        
        if (success) {
            GTX312L_SampleData bitmap{};
            bitmap.l = _async_read_buffer[0];
            bitmap.h = _async_read_buffer[1];
            result.channel_mask = (static_cast<uint32_t>(bitmap.value) & 0x0FFFu) & enabled_channels_mask_;
            result.module_mask = module_mask_;
        }
        
        result.timestamp_us = time_us_32();
        callback(result);
    });
}

bool GTX312L::write_register(uint8_t reg, uint8_t value) {
    return i2c_hal_->write_register(i2c_device_address_, reg, &value, 1);
}

bool GTX312L::read_register(uint8_t reg, uint8_t& value) {
    return i2c_hal_->read_register(i2c_device_address_, reg, &value, 1);
}

bool GTX312L::write_registers(uint8_t reg, const uint8_t* data, size_t length) {
    // 创建包含寄存器地址的数据包
    uint8_t* write_data = new uint8_t[length + 1];
    write_data[0] = reg;
    std::memcpy(write_data + 1, data, length);
    
    bool result = i2c_hal_->write(i2c_device_address_, write_data, length + 1);
    delete[] write_data;
    return result;
}

bool GTX312L::read_registers(uint8_t reg, uint8_t* data, size_t length) {
    // 先写寄存器地址
    if (!i2c_hal_->write(i2c_device_address_, &reg, 1)) {
        return false;
    }
    // 再读取数据
    return i2c_hal_->read(i2c_device_address_, data, length);
}

uint32_t GTX312L::getSupportedChannelCount() const {
    return static_cast<uint32_t>(max_channels_);
}

bool GTX312L::isInitialized() const {
    return initialized_;
}

// TouchSensor新接口实现
bool GTX312L::setChannelEnabled(uint8_t channel, bool enabled) {
    if (!initialized_ || channel >= GTX312L_MAX_CHANNELS) {
        return false;
    }
    
    if (enabled) {
        enabled_channels_mask_ |= (1UL << channel);
    } else {
        enabled_channels_mask_ &= ~(1UL << channel);
    }
    
    return true;
}

bool GTX312L::getChannelEnabled(uint8_t channel) const {
    if (channel >= GTX312L_MAX_CHANNELS) {
        return false;
    }
    return (enabled_channels_mask_ & (1 << channel)) != 0;
}

uint32_t GTX312L::getEnabledChannelMask() const {
    return enabled_channels_mask_;
}

bool GTX312L::setChannelSensitivity(uint8_t channel, int8_t sensitivity) {
    if (!initialized_ || channel >= GTX312L_MAX_CHANNELS || sensitivity > 99) {
        return false;
    }
    // 将0-99范围转换为GTX312L的0-63范围
    uint8_t gtx_sensitivity = (sensitivity * GTX312L_SENSITIVITY_MAX) / 99;
    // 直接写入灵敏度寄存器
    return write_register(GTX312L_REG_SENSITIVITY_1 + channel, gtx_sensitivity);
}

uint8_t GTX312L::getChannelSensitivity(uint8_t channel) const {
    if (channel >= GTX312L_MAX_CHANNELS || !initialized_) {
        return 50;  // 默认值
    }
    
    // 读取GTX312L的灵敏度寄存器
    uint8_t gtx_sensitivity;
    if (!const_cast<GTX312L*>(this)->read_register(GTX312L_REG_SENSITIVITY_1 + channel, gtx_sensitivity)) {
        return 50;  // 读取失败返回默认值
    }
    
    // 将GTX312L的0-255范围转换为0-99范围
    return (gtx_sensitivity * 99) / GTX312L_SENSITIVITY_MAX;
}
