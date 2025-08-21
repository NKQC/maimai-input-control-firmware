#include "gtx312l.h"
#include <cstring>
#include <algorithm>
#include <pico/time.h>
#include <pico/stdlib.h>

// GTX312L构造函数
GTX312L::GTX312L(HAL_I2C* i2c_hal, uint8_t device_address, const std::string& device_name) 
    : i2c_hal_(i2c_hal)
    , device_address_(device_address)
    , device_name_(device_name)
    , initialized_(false)
    , device_index_(0) {
    
    // 如果没有提供设备名称，使用默认格式
    if (device_name_.empty()) {
        device_name_ = "GTX312L_0x" + std::to_string(device_address);
    }
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
    
    // 检查设备是否存在
    if (!is_gtx312l_device(i2c_hal_, device_address_)) {
        return false;
    }
    
    // 复位设备
    if (!reset()) {
        return false;
    }
    
    // 设置默认配置
    if (!set_config(config_)) {
        return false;
    }
    
    // 执行初始校准
    if (!calibrate()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

// 释放资源
void GTX312L::deinit() {
    if (initialized_) {
        // 进入睡眠模式以节省功耗
        enter_sleep();
        
        initialized_ = false;
        touch_callback_ = nullptr;
    }
}

// 检查设备是否就绪
bool GTX312L::is_ready() const {
    return initialized_;
}

// 读取设备信息
bool GTX312L::read_device_info(GTX312L_DeviceInfo& info) {
    if (!i2c_hal_) {
        return false;
    }
    
    info.i2c_address = device_address_;
    
    // 读取芯片ID（根据datasheet，01h寄存器包含芯片ID信息）
    uint8_t chip_id_reg;
    if (!read_register(GTX312L_REG_FIRMWARE_VER, chip_id_reg)) {
        return false;
    }
    info.chip_id = chip_id_reg;
    
    // 读取固件版本
    if (!read_register(GTX312L_REG_FIRMWARE_VER, info.firmware_version)) {
        return false;
    }
    
    info.is_valid = (info.chip_id == GTX312L_CHIP_ID_VALUE);
    return true;
}

// 获取设备名称
std::string GTX312L::get_device_name() const {
    return device_name_;
}

// 获取设备地址
uint8_t GTX312L::get_device_address() const {
    return device_address_;
}

// 读取触摸数据
bool GTX312L::read_touch_data(GTX312L_TouchData& touch_data) {
    if (!is_ready()) {
        return false;
    }
    
    // 读取触摸状态寄存器（02h和03h）
    uint8_t touch_out_l, touch_out_h;
    if (!read_register(0x02, touch_out_l) ||
        !read_register(0x03, touch_out_h)) {
        return false;
    }
    
    // 组合触摸状态（通道1-8在低字节，通道9-12在高字节的低4位）
    touch_data.touch_status = touch_out_l | ((touch_out_h & 0x0F) << 8);
    
    touch_data.timestamp = time_us_32();
    touch_data.valid = true;
    
    return true;
}

// 设置配置
bool GTX312L::set_config(const GTX312L_Config& config) {
    if (!i2c_hal_) {
        return false;
    }
    
    config_ = config;
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 设置通道使能
    if (!write_register(GTX312L_REG_CH_ENABLE_L, config.channel_enable_mask_l)) {
        return false;
    }
    if (!write_register(GTX312L_REG_CH_ENABLE_H, config.channel_enable_mask_h)) {
        return false;
    }
    
    // 设置每个通道的灵敏度
    for (int i = 0; i < GTX312L_MAX_CHANNELS; i++) {
        uint8_t sensitivity_reg = GTX312L_REG_SENSITIVITY_1 + i;
        if (!write_register(sensitivity_reg, config.sensitivity[i])) {
            return false;
        }
    }
    
    // 设置校准时间
    if (!write_register(GTX312L_REG_CAL_TIME, config.cal_time)) {
        return false;
    }
    
    // 设置感应空闲时间
    if (!write_register(GTX312L_REG_SEN_IDLE_TIME, config.sen_idle_time)) {
        return false;
    }
    if (!write_register(GTX312L_REG_SEN_IDLE_SUFFIX, config.sen_idle_time_suffix)) {
        return false;
    }
    
    // 设置忙碌到空闲时间
    if (!write_register(GTX312L_REG_BUSY_TO_IDLE, config.busy_to_idle_time)) {
        return false;
    }
    
    // 设置中断模式和多点触摸模式
    uint8_t int_mode = 0;
    if (config.interrupt_enable) {
        int_mode |= GTX312L_INT_MODE_ENABLE;
    }
    if (config.multi_touch_enable) {
        int_mode |= GTX312L_MULTI_MODE_ENABLE;
    }
    if (!write_register(GTX312L_REG_INT_MODE, int_mode)) {
        return false;
    }
    
    // 设置I2C上拉配置
    uint8_t i2c_config = config.i2c_pullup_disable ? 0x01 : 0x00;
    if (!write_register(GTX312L_REG_I2C_PU_DIS, i2c_config)) {
        return false;
    }
    
    return true;
}

// 获取配置
bool GTX312L::get_config(GTX312L_Config& config) {
    config = config_;
    return true;
}

// 设置全局灵敏度（应用到所有通道）
bool GTX312L::set_global_sensitivity(uint8_t sensitivity) {
    if (!is_ready() || sensitivity > GTX312L_SENSITIVITY_MAX) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 设置所有通道的灵敏度
    for (int i = 0; i < GTX312L_MAX_CHANNELS; i++) {
        config_.sensitivity[i] = sensitivity;
        uint8_t sensitivity_reg = GTX312L_REG_SENSITIVITY_1 + i;
        if (!write_register(sensitivity_reg, sensitivity)) {
            return false;
        }
    }
    
    return true;
}

// 设置单个通道灵敏度
bool GTX312L::set_channel_sensitivity(uint8_t channel, uint8_t sensitivity) {
    if (!is_ready() || channel >= GTX312L_MAX_CHANNELS || sensitivity > GTX312L_SENSITIVITY_MAX) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    config_.sensitivity[channel] = sensitivity;
    uint8_t sensitivity_reg = GTX312L_REG_SENSITIVITY_1 + channel;
    return write_register(sensitivity_reg, sensitivity);
}

// 设置单个通道使能状态
bool GTX312L::set_channel_enable(uint8_t channel, bool enabled) {
    if (!is_ready() || channel >= GTX312L_MAX_CHANNELS) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    if (channel < 8) {
        // 通道1-8
        if (enabled) {
            config_.channel_enable_mask_l |= (1 << channel);
        } else {
            config_.channel_enable_mask_l &= ~(1 << channel);
        }
        return write_register(GTX312L_REG_CH_ENABLE_L, config_.channel_enable_mask_l);
    } else {
        // 通道9-12
        uint8_t bit_pos = channel - 8;
        if (enabled) {
            config_.channel_enable_mask_h |= (1 << bit_pos);
        } else {
            config_.channel_enable_mask_h &= ~(1 << bit_pos);
        }
        return write_register(GTX312L_REG_CH_ENABLE_H, config_.channel_enable_mask_h);
    }
}

// 设置所有通道使能状态
bool GTX312L::set_all_channels_enable(bool enabled) {
    if (!is_ready()) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    if (enabled) {
        config_.channel_enable_mask_l = GTX312L_CH_ENABLE_ALL_L;
        config_.channel_enable_mask_h = GTX312L_CH_ENABLE_ALL_H;
    } else {
        config_.channel_enable_mask_l = 0x00;
        config_.channel_enable_mask_h = 0x00;
    }
    
    if (!write_register(GTX312L_REG_CH_ENABLE_L, config_.channel_enable_mask_l)) {
        return false;
    }
    if (!write_register(GTX312L_REG_CH_ENABLE_H, config_.channel_enable_mask_h)) {
        return false;
    }
    
    return true;
}

// 设置多点触摸模式
bool GTX312L::set_multi_touch_mode(bool enabled) {
    if (!is_ready()) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    config_.multi_touch_enable = enabled;
    
    uint8_t int_mode = 0;
    if (config_.interrupt_enable) {
        int_mode |= GTX312L_INT_MODE_ENABLE;
    }
    if (enabled) {
        int_mode |= GTX312L_MULTI_MODE_ENABLE;
    }
    
    return write_register(GTX312L_REG_INT_MODE, int_mode);
}

// 设置中断模式
bool GTX312L::set_interrupt_mode(bool enabled) {
    if (!is_ready()) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    config_.interrupt_enable = enabled;
    
    uint8_t int_mode = 0;
    if (enabled) {
        int_mode |= GTX312L_INT_MODE_ENABLE;
    }
    if (config_.multi_touch_enable) {
        int_mode |= GTX312L_MULTI_MODE_ENABLE;
    }
    
    return write_register(GTX312L_REG_INT_MODE, int_mode);
}

// 获取全局灵敏度（从第一个通道读取）
uint8_t GTX312L::get_global_sensitivity() const {
    if (!is_ready()) {
        return GTX312L_SENSITIVITY_DEFAULT;
    }
    
    uint8_t sensitivity;
    if (!read_register(GTX312L_REG_SENSITIVITY_1, sensitivity)) {
        return GTX312L_SENSITIVITY_DEFAULT;
    }
    
    return sensitivity & GTX312L_SENSITIVITY_MAX;
}

// 获取单个通道灵敏度
uint8_t GTX312L::get_channel_sensitivity(uint8_t channel) const {
    if (!is_ready() || channel >= GTX312L_MAX_CHANNELS) {
        return GTX312L_SENSITIVITY_DEFAULT;
    }
    
    uint8_t sensitivity;
    uint8_t sensitivity_reg = GTX312L_REG_SENSITIVITY_1 + channel;
    if (!read_register(sensitivity_reg, sensitivity)) {
        return GTX312L_SENSITIVITY_DEFAULT;
    }
    
    return sensitivity & GTX312L_SENSITIVITY_MAX;
}

// 获取单个通道使能状态
bool GTX312L::get_channel_enable(uint8_t channel) const {
    if (!is_ready() || channel >= GTX312L_MAX_CHANNELS) {
        return false;
    }
    
    uint8_t enable_mask;
    if (channel < 8) {
        // 通道1-8
        if (!read_register(GTX312L_REG_CH_ENABLE_L, enable_mask)) {
            return false;
        }
        return (enable_mask & (1 << channel)) != 0;
    } else {
        // 通道9-12
        if (!read_register(GTX312L_REG_CH_ENABLE_H, enable_mask)) {
            return false;
        }
        uint8_t bit_pos = channel - 8;
        return (enable_mask & (1 << bit_pos)) != 0;
    }
}

// 手动校准
bool GTX312L::calibrate() {
    if (!i2c_hal_) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 设置监控复位位来触发校准（0Ah寄存器的bit0）
    if (!write_register(0x0A, 0x01)) {
        return false;
    }
    
    // 等待校准完成
    sleep_ms(500);
    
    // 清除监控复位位
    if (!write_register(0x0A, 0x00)) {
        return false;
    }
    
    return true;
}

// 复位设备
bool GTX312L::reset() {
    if (!i2c_hal_) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 设置软件复位位（0Bh寄存器的bit0）
    if (!write_register(0x0B, 0x01)) {
        return false;
    }
    
    // 等待复位完成
    sleep_ms(100);
    
    // 清除软件复位位
    if (!write_register(0x0B, 0x00)) {
        return false;
    }
    
    return true;
}

// 进入睡眠模式
bool GTX312L::enter_sleep() {
    if (!i2c_hal_) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 设置睡眠模式（0Bh寄存器的bit0设为1）
    return write_register(0x0B, 0x01);
}

// 唤醒设备
bool GTX312L::wakeup() {
    if (!i2c_hal_) {
        return false;
    }
    
    // 解锁寄存器写保护
    if (!write_register(GTX312L_REG_WRITE_LOCK, GTX312L_WRITE_LOCK_VALUE)) {
        return false;
    }
    
    // 清除睡眠模式（0Bh寄存器的bit0设为0）
    if (!write_register(0x0B, 0x00)) {
        return false;
    }
    
    // 等待唤醒完成
    sleep_ms(50);
    
    return true;
}

// 设置触摸回调
void GTX312L::set_touch_callback(GTX312L_TouchCallback callback, uint8_t device_index) {
    touch_callback_ = callback;
    device_index_ = device_index;
}

// 中断处理
void GTX312L::handle_interrupt() {
    if (!is_ready() || !touch_callback_) {
        return;
    }
    
    GTX312L_TouchData touch_data;
    if (read_touch_data(touch_data)) {
        touch_callback_(device_index_, touch_data);
    }
}

// 任务处理
void GTX312L::task() {
    if (!is_ready()) {
        return;
    }
    
    // 高效的触摸数据读取 - 直接读取而不检查状态以提高响应速度
    GTX312L_TouchData touch_data;
    if (read_touch_data(touch_data)) {
        // 只有在有实际触摸时才触发回调，减少不必要的处理
        if (touch_data.touch_status != 0 && touch_callback_) {
            touch_callback_(device_index_, touch_data);
        }
    }
}

// I2C总线扫描 - 优化版本，支持快速扫描和详细验证
std::vector<uint8_t> GTX312L::scan_i2c_bus(HAL_I2C* i2c_hal) {
    std::vector<uint8_t> found_addresses;
    
    if (!i2c_hal) {
        return found_addresses;
    }
    
    // 首先进行快速扫描，检查设备响应
    std::vector<uint8_t> responding_addresses;
    for (uint8_t addr = GTX312L_I2C_ADDR_MIN; addr <= GTX312L_I2C_ADDR_MAX; addr++) {
        uint8_t test_data = 0;
        if (i2c_hal->read(addr, &test_data, 1)) {
            responding_addresses.push_back(addr);
        }
    }
    
    // 对响应的地址进行详细验证
    for (uint8_t addr : responding_addresses) {
        if (is_gtx312l_device(i2c_hal, addr)) {
            found_addresses.push_back(addr);
        }
    }
    
    return found_addresses;
}

// 设备发现
std::vector<GTX312L*> GTX312L::discover_devices(HAL_I2C* i2c_hal, const std::string& name_prefix) {
    std::vector<GTX312L*> devices;
    
    if (!i2c_hal) {
        return devices;
    }
    
    // 扫描I2C总线
    std::vector<uint8_t> addresses = scan_i2c_bus(i2c_hal);
    
    // 为每个发现的地址创建GTX312L实例
    for (size_t i = 0; i < addresses.size(); i++) {
        std::string device_name = name_prefix + "_" + std::to_string(i);
        GTX312L* device = new GTX312L(i2c_hal, addresses[i], device_name);
        
        if (device->init()) {
            devices.push_back(device);
        } else {
            delete device;
        }
    }
    
    return devices;
}

// 清理设备
void GTX312L::cleanup_devices(std::vector<GTX312L*>& devices) {
    for (GTX312L* device : devices) {
        if (device) {
            device->deinit();
            delete device;
        }
    }
    devices.clear();
}

// 写寄存器
bool GTX312L::write_register(uint8_t reg_addr, uint8_t data) {
    if (!i2c_hal_) {
        return false;
    }
    
    uint8_t buffer[2] = {reg_addr, data};
    return i2c_hal_->write(device_address_, buffer, 2);
}

// 读寄存器
bool GTX312L::read_register(uint8_t reg_addr, uint8_t& data) {
    if (!i2c_hal_) {
        return false;
    }
    
    // 写寄存器地址
    if (!i2c_hal_->write(device_address_, &reg_addr, 1)) {
        return false;
    }
    
    return i2c_hal_->read(device_address_, &data, 1);
}

// 读多个寄存器
bool GTX312L::read_registers(uint8_t reg_addr, uint8_t* data, uint8_t length) {
    if (!i2c_hal_ || !data) {
        return false;
    }
    
    // 写寄存器地址
    if (!i2c_hal_->write(device_address_, &reg_addr, 1)) {
        return false;
    }
    
    return i2c_hal_->read(device_address_, data, length);
}

// 解析触摸数据
void GTX312L::parse_touch_data(const uint8_t* raw_data, GTX312L_TouchData& touch_data) {
    if (!raw_data) {
        return;
    }
    
    // 对于GTX312L，数据已经在read_touch_data中解析
    // 这个函数保留用于未来扩展
}

// 检测是否为GTX312L设备 - 增强版本，支持多重验证
bool GTX312L::is_gtx312l_device(HAL_I2C* i2c_hal, uint8_t address) {
    if (!i2c_hal) {
        return false;
    }
    
    // 第一步：尝试读取芯片ID
    uint8_t reg_addr = GTX312L_REG_CHIP_ID;
    if (!i2c_hal->write(address, &reg_addr, 1)) {
        return false;
    }
    
    uint8_t chip_id_data[2];
    if (!i2c_hal->read(address, chip_id_data, 2)) {
        return false;
    }
    
    uint16_t chip_id = (chip_id_data[1] << 8) | chip_id_data[0];
    
    // 第二步：检查芯片ID是否匹配GTX312L
    if (chip_id != GTX312L_CHIP_ID_VALUE) {
        return false;
    }
    
    // 第三步：尝试读取固件版本以进一步验证
    uint8_t fw_reg = GTX312L_REG_FIRMWARE_VER;
    if (!i2c_hal->write(address, &fw_reg, 1)) {
        return false;
    }
    
    uint8_t fw_version;
    if (!i2c_hal->read(address, &fw_version, 1)) {
        return false;
    }
    
    // 固件版本应该在合理范围内（0x01-0xFF）
    return (fw_version > 0x00 && fw_version <= 0xFF);
}