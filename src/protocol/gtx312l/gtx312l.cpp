#include "gtx312l.h"
#include <cstring>
#include <pico/time.h>
#include <pico/stdlib.h>
#include "src/protocol/usb_serial_logs/usb_serial_logs.h"

// GTX312L构造函数
GTX312L::GTX312L(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : i2c_hal_(i2c_hal), i2c_bus_(i2c_bus), device_addr_(device_addr), 
      i2c_device_address_(device_addr), initialized_(false) {
    // 构造物理设备地址
    physical_device_address_.mask = 0x0000;
    physical_device_address_.i2c_port = static_cast<uint8_t>(i2c_bus);
    physical_device_address_.addr = device_addr;
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

// 获取设备名称
std::string GTX312L::get_device_name() const {
    char name[32];
    snprintf(name, sizeof(name), "GTX312L_I2C%d_0x%02X", 
             static_cast<int>(i2c_bus_), i2c_device_address_);
    return std::string(name);
}

// 获取16位物理设备地址（通道bitmap为0）
GTX312L_PhysicalAddr GTX312L::get_physical_device_address() const {
    return physical_device_address_;
}

// 高效采样接口 没有检查!
GTX312L_SampleResult GTX312L::sample_touch_data() {
    // 读取触摸状态寄存器
    GTX312L_SampleData bitmap;
    if (!read_register(GTX312L_REG_TOUCH_STATUS_L, bitmap.l) ||
        !read_register(GTX312L_REG_TOUCH_STATUS_H, bitmap.h)) {
        return GTX312L_SampleResult(physical_device_address_.mask, 0);
    }
    
    GTX312L_SampleResult result(physical_device_address_.mask & 0xF000, bitmap.value);
    return result;
}

// 设置单个通道使能状态
bool GTX312L::set_channel_enable(uint8_t channel, bool enabled) {
    if (channel >= GTX312L_MAX_CHANNELS) {
        return false;
    }
    
    // 读取当前使能寄存器
    uint8_t enable_low = 0, enable_high = 0;
    if (!read_register(GTX312L_REG_CH_ENABLE_L, enable_low) ||
        !read_register(GTX312L_REG_CH_ENABLE_H, enable_high)) {
        return false;
    }
    
    // 组合16位使能掩码
    uint16_t enable_mask = (static_cast<uint16_t>(enable_high) << 8) | enable_low;
    
    if (enabled) {
        enable_mask |= (1 << channel);
    } else {
        enable_mask &= ~(1 << channel);
    }
    
    // 写回使能寄存器
    enable_low = enable_mask & 0xFF;
    enable_high = (enable_mask >> 8) & 0x0F;  // 只有低4位有效
    
    return write_register(GTX312L_REG_CH_ENABLE_L, enable_low) &&
           write_register(GTX312L_REG_CH_ENABLE_H, enable_high);
}

// 设置通道灵敏度
bool GTX312L::set_sensitivity(uint8_t channel, uint8_t sensitivity) {
    if (channel >= GTX312L_MAX_CHANNELS || sensitivity > GTX312L_SENSITIVITY_MAX) {
        return false;
    }
    
    // 灵敏度寄存器从GTX312L_REG_SENSITIVITY_1开始，每个通道一个字节
    return write_register(GTX312L_REG_SENSITIVITY_1 + channel, sensitivity);
}

bool GTX312L::write_register(uint8_t reg, uint8_t value) {
    return i2c_hal_->write_register(i2c_device_address_, reg, value);
}

bool GTX312L::read_register(uint8_t reg, uint8_t& value) {
    return i2c_hal_->read_register(i2c_device_address_, reg, &value);
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
