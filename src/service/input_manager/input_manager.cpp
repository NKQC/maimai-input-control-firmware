#include "input_manager.h"
#include "protocol/mai2serial/mai2serial.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../../hal/i2c/hal_i2c.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cmath>

// 静态实例变量定义
InputManager* InputManager::instance_ = nullptr;

// 静态配置变量
static InputManager_PrivateConfig static_config_;

// InputManager配置管理纯公开函数实现
InputManager_PrivateConfig* input_manager_get_config_holder() {
    return &static_config_;
}

bool input_manager_load_config_from_manager(InputManager* manager) {
    if (manager == nullptr) {
        return false;
    }
    // 这里可以从manager中加载配置到static_config_
    // 目前使用默认配置
    return true;
}

InputManager_PrivateConfig input_manager_get_config_copy() {
    return static_config_;
}

bool input_manager_write_config_to_manager(InputManager* manager, const InputManager_PrivateConfig& config) {
    if (manager == nullptr) {
        return false;
    }
    static_config_ = config;
    // 这里可以将配置写入到manager中
    return true;
}


// 单例模式实现
InputManager* InputManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new InputManager();
    }
    return instance_;
}

// 构造函数
InputManager::InputManager()
    : initialized_(false)
    , current_mode_(InputMode::SERIAL) // 默认Serial模式
    , mcp23s17_(nullptr)
    , mai2_serial_(nullptr)
    , hid_device_(nullptr)
    , calibrating_(false)
    , calibration_device_name_("")
    , calibration_progress_(0)
    , calibration_start_time_(0)
    , auto_sensitivity_running_(false)
    , auto_sensitivity_point_(0)
    , auto_sensitivity_result_(0)
    , auto_sensitivity_start_time_(0)
    , debug_enabled_(false) {
    
    // 初始化状态数组
    current_touch_states_.fill(false);
    current_key_states_.fill(false);
    
    // 初始化默认物理点位配置
    for (uint8_t i = 0; i < 72; i++) {
        physical_points_[i] = PhysicalPoint();
        physical_points_[i].name = "Physical_" + std::to_string(i);
        physical_points_[i].enabled = false; // 默认禁用，只在映射时启用
    }
}

// 析构函数
InputManager::~InputManager() {
    deinit();
}

// 初始化
bool InputManager::init(const InputManager_Config& config) {
    if (initialized_) {
        return true;
    }
    
    // 从配置中添加GTX312L设备
    for (uint8_t i = 0; i < config.gtx312l_count && i < 8; i++) {
        if (config.gtx312l_devices[i] != nullptr) {
            std::string device_name = "GTX312L_" + std::to_string(i);
            add_device(config.gtx312l_devices[i], device_name);
        }
    }
    
    // 设置HID设备
    hid_device_ = config.hid;
    
    // 设置Mai2Serial设备
    mai2_serial_ = config.mai2_serial;
    
    // 设置MCP23S17设备
    mcp23s17_ = config.mcp23s17;
    
    if (devices_.empty()) {
        log_debug("Warning: No GTX312L devices found");
    }
    
    // 初始化通道管理
    update_channel_bitmaps();
    apply_channel_bitmaps_to_devices();
    
    initialized_ = true;
    
    log_debug("InputManager initialized with " + std::to_string(devices_.size()) + " devices");
    
    return true;
}

// 释放资源
void InputManager::deinit() {
    if (initialized_) {
        // 停止校准
        stop_calibration();
        
        // 释放所有设备
        for (auto& pair : devices_) {
            if (pair.second.device) {
                pair.second.device->deinit();
                delete pair.second.device;
            }
        }
        devices_.clear();
        
        // 清理回调
        touch_event_callback_ = nullptr;
        input_mapping_callback_ = nullptr;
        device_status_callback_ = nullptr;
        calibration_callback_ = nullptr;
        
        initialized_ = false;
        
        log_debug("InputManager deinitialized");
    }
}

// 检查是否就绪
bool InputManager::is_ready() const {
    return initialized_ && !devices_.empty();
}

// 添加设备
bool InputManager::add_device(GTX312L* device, const std::string& device_name) {
    if (!device) {
        return false;
    }
    
    if (devices_.size() >= 8) { // 最多支持8个设备
        return false;
    }
    
    // 生成设备名称：I2C通道+地址
    std::string final_device_name = device_name.empty() ? generate_device_name(device) : device_name;
    
    // 检查设备名称是否已存在
    if (devices_.find(final_device_name) != devices_.end()) {
        log_debug("Device name already exists: " + final_device_name);
        return false;
    }
    
    DeviceInfo info;
    info.device = device;
    info.name = final_device_name;
    info.connected = device->is_ready();
    info.last_scan_time = 0;
    
    devices_[final_device_name] = info;
    
    log_debug("Added device: " + final_device_name);
    
    // 通知设备状态回调
    if (device_status_callback_) {
        device_status_callback_(final_device_name, info.connected);
    }
    
    return true;
}

// 移除设备
bool InputManager::remove_device(const std::string& device_name) {
    auto it = devices_.find(device_name);
    if (it == devices_.end()) {
        return false;
    }
    
    if (it->second.device) {
        it->second.device->deinit();
        delete it->second.device;
    }
    
    devices_.erase(it);
    
    log_debug("Removed device: " + device_name);
    
    // 通知设备状态回调
    if (device_status_callback_) {
        device_status_callback_(device_name, false);
    }
    
    return true;
}

// 获取设备数量
bool InputManager::get_device_count() const {
    return devices_.size();
}

// 获取设备信息
bool InputManager::get_device_info(const std::string& device_name, std::string& name, bool& connected) {
    auto it = devices_.find(device_name);
    if (it == devices_.end()) {
        return false;
    }
    
    name = it->second.name;
    connected = it->second.connected;
    return true;
}

// 设置HID设备
bool InputManager::set_hid_device(HID* hid_device) {
    hid_device_ = hid_device;
    log_debug("HID device " + std::string(hid_device ? "set" : "cleared"));
    return true;
}

// 获取HID设备
HID* InputManager::get_hid_device() const {
    return hid_device_;
}

// 设置工作模式
bool InputManager::set_input_mode(InputMode mode) {
    current_mode_ = mode;
    log_debug("Input mode set to " + std::string(mode == InputMode::SERIAL ? "SERIAL" : "HID"));
    return true;
}

// 获取工作模式
InputMode InputManager::get_input_mode() const {
    return current_mode_;
}

// 设置Mai2Serial设备
bool InputManager::set_mai2serial_device(Mai2Serial* mai2_serial) {
    mai2_serial_ = mai2_serial;
    log_debug("Mai2Serial device " + std::string(mai2_serial ? "set" : "cleared"));
    return true;
}

// 获取Mai2Serial设备
Mai2Serial* InputManager::get_mai2serial_device() const {
    return mai2_serial_;
}

// 配置管理已移至纯公开函数

// 设置点位配置
bool InputManager::set_point_config(uint8_t point_index, const InputPoint& point) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    points_[point_index] = point;
    log_debug("Point " + std::to_string(point_index) + " configuration updated");
    return true;
}

// 获取点位配置
bool InputManager::get_point_config(uint8_t point_index, InputPoint& point) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    point = points_[point_index];
    return true;
}

// 启用/禁用点位 - 直接设置到设备，无缓存
bool InputManager::enable_point(uint8_t point_index, bool enabled) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    // 直接应用到对应的设备，无缓存
    std::string device_name;
    uint8_t channel;
    point_index_to_device_channel(point_index, device_name, channel);
    
    auto device = find_device_by_name(device_name);
    if (device) {
        bool result = device->set_channel_enable(channel, enabled);
        if (result) {
            log_debug("Point " + std::to_string(point_index) + " " + (enabled ? "enabled" : "disabled"));
        }
        return result;
    }
    
    return false;
}

// 获取点位启用状态 - 实时从设备读取
bool InputManager::get_point_enabled(uint8_t point_index) const {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    // 直接从对应的设备读取，无缓存
    std::string device_name;
    uint8_t channel;
    point_index_to_device_channel(point_index, device_name, channel);
    
    auto device = find_device_by_name(device_name);
    if (device) {
        return device->get_channel_enable(channel);
    }
    
    return false;
}

// 设置点位灵敏度 - 直接设置到设备，无缓存
bool InputManager::set_point_sensitivity(uint8_t point_index, uint8_t sensitivity) {
    if (!is_point_valid(point_index) || sensitivity > 63) {
        return false;
    }
    
    // 直接应用到对应的设备，无缓存
    std::string device_name;
    uint8_t channel;
    point_index_to_device_channel(point_index, device_name, channel);
    
    auto device = find_device_by_name(device_name);
    if (device) {
        bool result = device->set_channel_sensitivity(channel, sensitivity);
        if (result) {
            log_debug("Point " + std::to_string(point_index) + " sensitivity set to " + std::to_string(sensitivity));
        }
        return result;
    }
    
    return false;
}

// 设置全局灵敏度（应用到所有设备的所有通道）- 直接设置到设备，无缓存
bool InputManager::set_global_sensitivity(uint8_t sensitivity) {
    if (sensitivity > 63) {
        return false;
    }
    
    bool all_success = true;
    
    // 直接应用到所有设备的所有通道，无缓存
    for (auto& device_pair : devices_) {
        if (device_pair.second.device) {
            if (!device_pair.second.device->set_global_sensitivity(sensitivity)) {
                all_success = false;
            }
        }
    }
    
    if (all_success) {
        log_debug("Global sensitivity set to " + std::to_string(sensitivity));
    }
    return all_success;
}

// 获取全局灵敏度 - 从第一个可用设备实时读取
uint8_t InputManager::get_global_sensitivity() const {
    // 从第一个可用设备读取灵敏度
    for (const auto& device_pair : devices_) {
        if (device_pair.second.device && device_pair.second.connected) {
            uint8_t sensitivity = 0;
            if (device_pair.second.device->get_global_sensitivity(sensitivity)) {
                return sensitivity;
            }
        }
    }
    
    // 如果没有可用设备，返回默认值
    return 32;
}

// 相对调整全局灵敏度 - 基于实时读取的值进行调整
bool InputManager::adjust_global_sensitivity(int8_t delta) {
    uint8_t current_sensitivity = get_global_sensitivity();
    int16_t new_sensitivity = static_cast<int16_t>(current_sensitivity) + delta;
    
    // 限制范围在0-63之间
    if (new_sensitivity < 0) {
        new_sensitivity = 0;
    } else if (new_sensitivity > 63) {
        new_sensitivity = 63;
    }
    
    return set_global_sensitivity(static_cast<uint8_t>(new_sensitivity));
}



// 设置点位阈值
bool InputManager::set_point_threshold(uint8_t point_index, uint8_t threshold) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    points_[point_index].threshold = threshold;
    return true;
}

// 添加区域
bool InputManager::add_region(const InputRegion& region) {
    // 检查区域名称是否已存在
    for (const auto& existing : regions_) {
        if (existing.name == region.name) {
            return false;
        }
    }
    
    regions_.push_back(region);
    log_debug("Added region: " + region.name);
    return true;
}

// 移除区域
bool InputManager::remove_region(const std::string& name) {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](const InputRegion& region) {
                              return region.name == name;
                          });
    
    if (it != regions_.end()) {
        regions_.erase(it);
        log_debug("Removed region: " + name);
        return true;
    }
    
    return false;
}

// 获取区域
bool InputManager::get_region(const std::string& name, InputRegion& region) {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](const InputRegion& r) {
                              return r.name == name;
                          });
    
    if (it != regions_.end()) {
        region = *it;
        return true;
    }
    
    return false;
}

// 获取区域名称列表
std::vector<std::string> InputManager::get_region_names() {
    std::vector<std::string> names;
    for (const auto& region : regions_) {
        names.push_back(region.name);
    }
    return names;
}

// 添加映射
bool InputManager::add_mapping(const InputMapping& mapping) {
    if (!is_point_valid(mapping.point_index)) {
        return false;
    }
    
    mappings_[mapping.point_index] = mapping;
    log_debug("Added mapping for point " + std::to_string(mapping.point_index));
    return true;
}

// 移除映射
bool InputManager::remove_mapping(uint8_t point_index) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    mappings_[point_index] = InputMapping();
    mappings_[point_index].point_index = point_index;
    mappings_[point_index].enabled = false;
    
    log_debug("Removed mapping for point " + std::to_string(point_index));
    return true;
}

// 获取映射
bool InputManager::get_mapping(uint8_t point_index, InputMapping& mapping) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    mapping = mappings_[point_index];
    return true;
}

// 启用/禁用映射
bool InputManager::enable_mapping(uint8_t point_index, bool enabled) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    mappings_[point_index].enabled = enabled;
    log_debug("Mapping for point " + std::to_string(point_index) + " " + (enabled ? "enabled" : "disabled"));
    return true;
}

// 开始校准
bool InputManager::start_calibration(const std::string& device_name) {
    if (calibrating_) {
        return false;
    }
    
    calibrating_ = true;
    calibration_device_name_ = device_name;
    calibration_progress_ = 0;
    calibration_start_time_ = time_us_32() / 1000;
    
    log_debug("Started calibration for device " + device_name);
    
    // 如果指定了特定设备
    if (!device_name.empty()) {
        auto it = devices_.find(device_name);
        if (it != devices_.end() && it->second.device) {
            it->second.device->calibrate();
        }
    } else {
        // 校准所有设备
        for (auto& pair : devices_) {
            if (pair.second.device) {
                pair.second.device->calibrate();
            }
        }
    }
    
    return true;
}

// 停止校准
bool InputManager::stop_calibration() {
    if (!calibrating_) {
        return false;
    }
    
    calibrating_ = false;
    log_debug("Stopped calibration");
    
    // 通知校准回调
    if (calibration_callback_) {
        calibration_callback_(calibration_device_name_, true);
    }
    
    return true;
}

// 检查是否正在校准
bool InputManager::is_calibrating() const {
    return calibrating_;
}

// 获取校准进度
bool InputManager::get_calibration_progress(uint8_t& progress) {
    if (!calibrating_) {
        return false;
    }
    
    progress = calibration_progress_;
    return true;
}

// 获取触摸状态
bool InputManager::get_touch_state(uint8_t point_index, bool& pressed, uint8_t& pressure) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    pressed = current_states_[point_index];
    pressure = 0;  // 移除压力处理，直接设为0
    return true;
}

// 获取所有触摸状态
bool InputManager::get_all_touch_states(std::array<bool, 34>& states) {
    states = current_states_;
    return true;
}

// 获取活跃触摸点数量
uint8_t InputManager::get_active_touch_count() {
    uint8_t count = 0;
    for (bool state : current_states_) {
        if (state) count++;
    }
    return count;
}

// 点位到坐标转换
bool InputManager::point_to_coordinates(uint8_t point_index, uint16_t& x, uint16_t& y) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    x = points_[point_index].x_coord;
    y = points_[point_index].y_coord;
    return true;
}

// 坐标到点位转换 - 优化版本，支持区域检测和精确匹配
bool InputManager::coordinates_to_point(uint16_t x, uint16_t y, uint8_t& point_index) {
    uint32_t min_distance = UINT32_MAX;
    uint8_t closest_point = 0;
    bool found_valid_point = false;
    
    // 首先检查是否在任何启用点位的有效范围内
    for (uint8_t i = 0; i < 34; i++) {
        if (!get_point_enabled(i)) continue;
        
        int32_t dx = static_cast<int32_t>(x) - static_cast<int32_t>(points_[i].x_coord);
        int32_t dy = static_cast<int32_t>(y) - static_cast<int32_t>(points_[i].y_coord);
        uint32_t distance = dx * dx + dy * dy;
        
        // 检查是否在点位的有效半径内（基于点位大小）
        uint32_t max_radius_sq = points_[i].radius * points_[i].radius;
        
        if (distance <= max_radius_sq) {
            // 在有效范围内，选择最近的点
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = i;
                found_valid_point = true;
            }
        } else if (!found_valid_point && distance < min_distance) {
            // 如果没有找到有效范围内的点，记录最近的点作为备选
            min_distance = distance;
            closest_point = i;
        }
    }
    
    point_index = closest_point;
    return found_valid_point || (min_distance < UINT32_MAX);
}

// 获取统计信息
bool InputManager::get_statistics(InputStatistics& stats) {
    stats = statistics_;
    return true;
}

// 重置统计信息
void InputManager::reset_statistics() {
    statistics_ = InputStatistics();
    statistics_.last_reset_time = time_us_32() / 1000;
    log_debug("Statistics reset");
}

// 设置回调
void InputManager::set_touch_event_callback(TouchEventCallback callback) {
    touch_event_callback_ = callback;
}

void InputManager::set_input_mapping_callback(InputMappingCallback callback) {
    input_mapping_callback_ = callback;
}

void InputManager::set_device_status_callback(DeviceStatusCallback callback) {
    device_status_callback_ = callback;
}

void InputManager::set_calibration_callback(CalibrationCallback callback) {
    calibration_callback_ = callback;
}

// CPU0任务处理 - 读取原始分区触发bitmap -> 映射到逻辑区 -> [Serial模式](传递给Mai2Serial) / [HID模式](将触摸映射Bitmap发送到FIFO跨核心传输)
void InputManager::loop0() {
    static uint32_t last_touch_bitmap = 0;
    static uint8_t maintenance_counter = 0;
    
    uint32_t current_touch_bitmap = 0;
    bool touch_state_changed = false;
    
    // 1. 高频扫描GTX312L设备 - 读取原始物理触摸数据
    for (auto& pair : devices_) {
        DeviceInfo& info = pair.second;
        if (!info.device || !info.connected) continue;
        
        GTX312L_TouchData data;
        if (info.device->read_touch_data(data)) {
            uint16_t touch_status = data.touch_status;
            uint16_t prev_status = info.last_data.touch_status;
            uint16_t changed = touch_status ^ prev_status;
            
            if (changed) {
                // 计算设备在物理点位数组中的偏移
                uint8_t device_index = 0;
                for (const auto& dev_pair : devices_) {
                    if (dev_pair.first == pair.first) break;
                    device_index++;
                }
                uint8_t device_offset = device_index * 12;
                
                // 处理变化的通道
                while (changed) {
                    uint8_t ch = __builtin_ctz(changed);
                    changed &= changed - 1;
                    
                    uint8_t physical_point = device_offset + ch;
                    if (physical_point < 72 && physical_points_[physical_point].enabled) {
                        bool pressed = (touch_status & (1 << ch)) != 0;
                        if (current_touch_states_[physical_point] != pressed) {
                            current_touch_states_[physical_point] = pressed;
                            touch_state_changed = true;
                            
                            if (pressed) {
                                current_touch_bitmap |= (1UL << physical_point);
                            }
                        }
                    }
                }
                info.last_data = data;
            }
        }
    }
    
    // 2. 只在触摸状态变化时进行映射和传输
    if (touch_state_changed || current_touch_bitmap != last_touch_bitmap) {
        last_touch_bitmap = current_touch_bitmap;
        
        if (current_mode_ == InputMode::SERIAL) {
            // Serial模式：物理点位 -> 逻辑区域 -> Mai2Serial
            process_serial_mode_mapping(current_touch_bitmap);
        } else {
            // HID模式：物理点位 -> 逻辑点位 -> FIFO传输到CPU1
            process_hid_mode_mapping(current_touch_bitmap);
        }
    }
    
    // 3. 低频维护任务 - 每256次循环执行一次
    if (++maintenance_counter == 0) {
        // 处理校准状态
        if (calibrating_) {
            uint32_t elapsed = (time_us_32() / 1000) - calibration_start_time_;
            calibration_progress_ = (elapsed > 10000) ? 100 : elapsed / 100;
            if (calibration_progress_ >= 100) calibrating_ = false;
        }
        
        // 处理自动灵敏度调整
        if (auto_sensitivity_running_) {
            process_auto_sensitivity_adjustment();
        }
        
        // Serial模式下处理Mai2Serial命令
        if (current_mode_ == InputMode::SERIAL && mai2_serial_) {
            mai2_serial_->process_commands();
        }
    }
}

// CPU1任务处理 - 键盘数据处理和HID模式
void InputManager::loop1() {
    static MCP23S17_GPIO_State last_gpio = {0};
    
    // 高速MCP23S17扫描 - 按键输入处理
    if (mcp23s17_) {
        MCP23S17_GPIO_State gpio_state;
        if (mcp23s17_->read_all_gpio(gpio_state)) {
            // 检查变化的GPIO位
            uint16_t changed = gpio_state.port_a ^ last_gpio.port_a;
            changed |= (gpio_state.port_b ^ last_gpio.port_b) << 8;
            
            if (changed) {
                // 使用位操作快速处理GPIO变化
                while (changed) {
                    uint8_t i = __builtin_ctz(changed); // 找到最低位的1
                    changed &= changed - 1; // 清除最低位的1
                    
                    bool pressed = (i < 8) ? 
                        (gpio_state.port_a & (1 << i)) == 0 : // 假设低电平有效
                        (gpio_state.port_b & (1 << (i - 8))) == 0;
                    
                    // 根据模式处理按键映射
                    if (current_mode_ == InputMode::SERIAL) {
                        // Serial模式：通过Mai2Serial发送按键数据
                        if (mai2_serial_) {
                            Mai2Serial_ButtonData button_data;
                            button_data.button_id = i;
                            button_data.pressed = pressed;
                            mai2_serial_->send_button_data(button_data);
                        }
                    } else {
                        // HID模式：处理按键到HID键盘映射
                        process_key_input(i, pressed);
                    }
                }
                last_gpio = gpio_state;
            }
        }
    }
    
    // HID模式专用处理
    if (current_mode_ == InputMode::HID && hid_device_) {
        // 从FIFO接收触摸数据（来自CPU0）
        uint32_t touch_bitmap;
        if (multicore_fifo_pop_timeout_us(0, &touch_bitmap)) {
            // 转换触摸位图为HID触摸报告并发送
            process_hid_touch_mapping();
        }
        
        // 处理HID键盘映射
        process_hid_keyboard_mapping();
        
        // 处理HID设备状态更新
        hid_device_->update();
    }
}

// Serial模式映射处理 - 物理点位 -> 逻辑区域 -> Mai2Serial
void InputManager::process_serial_mode_mapping(uint32_t touch_bitmap) {
    if (!mai2_serial_) return;
    
    // 根据Serial模式映射配置，将物理点位映射到逻辑区域
    // Mai2 使用两个32位状态字来表示34个区域的触摸状态
    uint32_t state1 = 0;  // 区域1-32 (bit 0-31)
    uint32_t state2 = 0;  // 区域33-34 (bit 0-1)
    
    for (uint8_t area_idx = 0; area_idx < 34; area_idx++) {
        if (!serial_mapping_.logical_areas[area_idx].enabled) continue;
        
        // 检查该逻辑区域的所有物理点位是否有触摸
        bool area_touched = false;
        for (uint8_t point_idx = 0; point_idx < 8; point_idx++) {
            uint8_t physical_point = serial_mapping_.logical_areas[area_idx].physical_points[point_idx];
            if (physical_point < 72 && (touch_bitmap & (1UL << physical_point))) {
                area_touched = true;
                break;
            }
        }
        
        if (area_touched) {
            // 区域编号从1开始，所以area_idx+1对应实际区域号
            if (area_idx < 32) {
                state1 |= (1UL << area_idx);  // 区域1-32映射到state1的bit 0-31
            } else {
                state2 |= (1UL << (area_idx - 32));  // 区域33-34映射到state2的bit 0-1
            }
        }
    }
    
    // 使用二进制格式发送触摸状态到Mai2Serial
    mai2_serial_->send_touch_state(state1, state2);
}

// HID模式映射处理 - 物理点位 -> 逻辑点位 -> FIFO传输到CPU1
void InputManager::process_hid_mode_mapping(uint32_t touch_bitmap) {
    // 根据HID模式映射配置，将物理点位映射到逻辑点位
    uint32_t logical_point_bitmap = 0;
    
    for (uint8_t point_idx = 0; point_idx < 34; point_idx++) {
        if (!hid_mapping_.logical_points[point_idx].enabled) continue;
        
        // 检查该逻辑点位的所有物理点位是否有触摸
        bool point_touched = false;
        for (uint8_t phys_idx = 0; phys_idx < 8; phys_idx++) {
            uint8_t physical_point = hid_mapping_.logical_points[point_idx].physical_points[phys_idx];
            if (physical_point < 72 && (touch_bitmap & (1UL << physical_point))) {
                point_touched = true;
                break;
            }
        }
        
        if (point_touched) {
            logical_point_bitmap |= (1UL << point_idx);
        }
    }
    
    // 通过FIFO发送到CPU1进行HID处理
    if (logical_point_bitmap != 0) {
        multicore_fifo_push_timeout_us(logical_point_bitmap, 0); // 非阻塞发送
    }
}

// 自动灵敏度调整处理
void InputManager::process_auto_sensitivity_adjustment() {
    if (!auto_sensitivity_running_) return;
    
    uint32_t elapsed = (time_us_32() / 1000) - auto_sensitivity_start_time_;
    
    // 自动灵敏度调整算法
    if (elapsed > 5000) { // 5秒超时
        auto_sensitivity_running_ = false;
        log_debug("Auto sensitivity adjustment timeout");
        return;
    }
    
    // 检查当前点位的触摸状态
    if (auto_sensitivity_point_ < 34) {
        bool is_touched = current_states_[auto_sensitivity_point_];
        
        if (is_touched) {
            // 触摸检测到，记录当前灵敏度作为结果
            uint8_t current_sensitivity = get_global_sensitivity();
            auto_sensitivity_result_ = current_sensitivity;
            auto_sensitivity_running_ = false;
            
            log_debug("Auto sensitivity found optimal value: " + std::to_string(current_sensitivity));
        } else {
            // 未检测到触摸，增加灵敏度
            uint8_t current_sensitivity = get_global_sensitivity();
            if (current_sensitivity < 63) {
                set_global_sensitivity(current_sensitivity + 1);
            } else {
                // 达到最大灵敏度仍未检测到，停止调整
                auto_sensitivity_running_ = false;
                auto_sensitivity_result_ = 63;
                log_debug("Auto sensitivity reached maximum value");
            }
        }
    }
}

// 启用调试输出
void InputManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
    log_debug("Debug output " + std::string(enabled ? "enabled" : "disabled"));
}

// 开始自动灵敏度调整
bool InputManager::start_auto_sensitivity_adjustment(uint8_t point_index) {
    if (auto_sensitivity_running_ || !is_point_valid(point_index)) {
        return false;
    }
    
    auto_sensitivity_running_ = true;
    auto_sensitivity_point_ = point_index;
    auto_sensitivity_result_ = 0;
    auto_sensitivity_start_time_ = time_us_32() / 1000;
    
    // 从较低的灵敏度开始
    set_global_sensitivity(16);
    
    log_debug("Started auto sensitivity adjustment for point " + std::to_string(point_index));
    return true;
}

// 停止自动灵敏度调整
bool InputManager::stop_auto_sensitivity_adjustment() {
    if (!auto_sensitivity_running_) {
        return false;
    }
    
    auto_sensitivity_running_ = false;
    log_debug("Stopped auto sensitivity adjustment");
    return true;
}

// 获取自动灵敏度调整结果
bool InputManager::get_auto_sensitivity_result(uint8_t& result) {
    if (auto_sensitivity_running_) {
        return false;
    }
    
    result = auto_sensitivity_result_;
    return true;
}

// 检查自动灵敏度调整是否正在运行
bool InputManager::is_auto_sensitivity_running() const {
    return auto_sensitivity_running_;
}

// 手动触发逻辑区域 (Serial模式)
bool InputManager::trigger_logical_area_manual(uint8_t area_index) {
    if (area_index >= 34 || current_mode_ != InputMode::SERIAL) {
        return false;
    }
    
    if (mai2_serial_) {
        uint32_t state1 = 0;
        uint32_t state2 = 0;
        
        // 正确映射区域到状态位
        if (area_index < 32) {
            state1 = (1UL << area_index);  // 区域0-31映射到state1
        } else {
            state2 = (1UL << (area_index - 32));  // 区域32-33映射到state2
        }
        
        // 发送触摸状态
        mai2_serial_->send_touch_state(state1, state2);
        
        // 短暂延迟后发送释放状态
        sleep_ms(50);
        mai2_serial_->send_touch_state(0, 0);
        
        return true;
    }
    
    return false;
}

// 手动触发逻辑点位 (HID模式)
bool InputManager::trigger_logical_point_manual(uint8_t point_index) {
    if (point_index >= 34 || current_mode_ != InputMode::HID) {
        return false;
    }
    
    if (hid_device_ && hid_mapping_.logical_points[point_index].enabled) {
        uint16_t x = hid_mapping_.logical_points[point_index].x_coord;
        uint16_t y = hid_mapping_.logical_points[point_index].y_coord;
        
        hid_device_->set_touch_point(point_index, x, y, 255);
        hid_device_->send_report();
        
        // 延迟后释放
        sleep_ms(50);
        hid_device_->release_touch_point(point_index);
        hid_device_->send_report();
        
        return true;
    }
    
    return false;
}

// 手动触发HID坐标
bool InputManager::trigger_hid_coordinate_manual(uint16_t x, uint16_t y) {
    if (current_mode_ != InputMode::HID || !hid_device_) {
        return false;
    }
    
    // 使用临时触摸点ID (34)
    hid_device_->set_touch_point(34, x, y, 255);
    hid_device_->send_report();
    
    // 延迟后释放
    sleep_ms(50);
    hid_device_->release_touch_point(34);
    hid_device_->send_report();
    
    return true;
}

// 根据映射更新通道位图
void InputManager::update_channel_bitmaps() {
    // 清空所有通道位图
    for (uint8_t i = 0; i < 6; i++) {
        channel_bitmaps_[i].enabled_channels = 0;
        channel_bitmaps_[i].mapped_channels = 0;
    }
    
    // 根据Serial模式映射计算需要的通道
    for (uint8_t area_idx = 0; area_idx < 34; area_idx++) {
        if (!serial_mapping_.logical_areas[area_idx].enabled) continue;
        
        for (uint8_t point_idx = 0; point_idx < 8; point_idx++) {
            uint8_t physical_point = serial_mapping_.logical_areas[area_idx].physical_points[point_idx];
            if (physical_point < 72) {
                uint8_t device_id;
                uint8_t channel = physical_point_to_device_channel(physical_point, device_id);
                uint8_t device_index = get_device_index_by_id(device_id);
                
                if (device_index < 6) {
                    channel_bitmaps_[device_index].mapped_channels |= (1 << channel);
                }
            }
        }
    }
    
    // 根据HID模式映射计算需要的通道
    for (uint8_t point_idx = 0; point_idx < 34; point_idx++) {
        if (!hid_mapping_.logical_points[point_idx].enabled) continue;
        
        for (uint8_t phys_idx = 0; phys_idx < 8; phys_idx++) {
            uint8_t physical_point = hid_mapping_.logical_points[point_idx].physical_points[phys_idx];
            if (physical_point < 72) {
                uint8_t device_id;
                uint8_t channel = physical_point_to_device_channel(physical_point, device_id);
                uint8_t device_index = get_device_index_by_id(device_id);
                
                if (device_index < 6) {
                    channel_bitmaps_[device_index].mapped_channels |= (1 << channel);
                }
            }
        }
    }
    
    // 启用所有映射的通道
    for (uint8_t i = 0; i < 6; i++) {
        channel_bitmaps_[i].enabled_channels = channel_bitmaps_[i].mapped_channels;
    }
}

// 启用/禁用设备通道
bool InputManager::enable_device_channel(uint8_t device_id, uint8_t channel, bool enabled) {
    if (channel >= 12) return false;
    
    uint8_t device_index = get_device_index_by_id(device_id);
    if (device_index >= 6) return false;
    
    if (enabled) {
        channel_bitmaps_[device_index].enabled_channels |= (1 << channel);
    } else {
        channel_bitmaps_[device_index].enabled_channels &= ~(1 << channel);
    }
    
    return true;
}

// 检查设备通道是否启用
bool InputManager::is_device_channel_enabled(uint8_t device_id, uint8_t channel) const {
    if (channel >= 12) return false;
    
    uint8_t device_index = get_device_index_by_id(device_id);
    if (device_index >= 6) return false;
    
    return (channel_bitmaps_[device_index].enabled_channels & (1 << channel)) != 0;
}

// 获取设备通道位图
uint16_t InputManager::get_device_channel_bitmap(uint8_t device_id) const {
    uint8_t device_index = get_device_index_by_id(device_id);
    if (device_index >= 6) return 0;
    
    return channel_bitmaps_[device_index].enabled_channels;
}

// 将通道位图应用到实际设备
void InputManager::apply_channel_bitmaps_to_devices() {
    for (const auto& pair : devices_) {
        uint8_t device_id = pair.first;
        const DeviceInfo& info = pair.second;
        
        if (info.device && info.connected) {
            uint8_t device_index = get_device_index_by_id(device_id);
            if (device_index < 6) {
                uint16_t enabled_channels = channel_bitmaps_[device_index].enabled_channels;
                info.device->set_enabled_channels(enabled_channels);
            }
        }
    }
}

// 根据设备ID获取设备索引
uint8_t InputManager::get_device_index_by_id(uint8_t device_id) const {
    uint8_t index = 0;
    for (const auto& pair : devices_) {
        if (pair.first == device_id) {
            return index;
        }
        index++;
    }
    return 255; // 无效索引
}

// 将物理点位转换为设备ID和通道
uint8_t InputManager::physical_point_to_device_channel(uint8_t physical_point, uint8_t& device_id) const {
    if (physical_point >= 72) {
        device_id = 0;
        return 0;
    }
    
    uint8_t device_index = physical_point / 12;
    uint8_t channel = physical_point % 12;
    
    // 根据设备索引获取实际设备ID
    uint8_t current_index = 0;
    for (const auto& pair : devices_) {
        if (current_index == device_index) {
            device_id = pair.first;
            return channel;
        }
        current_index++;
    }
    
    device_id = 0;
    return 0;
}

// 获取调试信息
std::string InputManager::get_debug_info() {
    std::string info = "InputManager Debug Info:\n";
    info += "Devices: " + std::to_string(devices_.size()) + "\n";
    info += "Active touches: " + std::to_string(get_active_touch_count()) + "\n";
    info += "Total touches: " + std::to_string(statistics_.total_touches) + "\n";
    info += "Calibrating: " + std::string(calibrating_ ? "Yes" : "No") + "\n";
    
    return info;
}

// 测试点位
bool InputManager::test_point(uint8_t point_index) {
    if (!is_point_valid(point_index)) {
        return false;
    }
    
    // 模拟触摸事件
    process_touch_event(point_index, true, 255);
    sleep_ms(100);
    process_touch_event(point_index, false, 0);
    
    log_debug("Tested point " + std::to_string(point_index));
    return true;
}

// 私有方法实现
void InputManager::scan_devices() {
    // 已优化到loop0中，此函数保留用于兼容性
    // 实际扫描逻辑已内联到loop0以获得最大性能
}

void InputManager::process_device_data(const std::string& device_name, const GTX312L_TouchData& data) {
    // 已优化到loop0中，此函数保留用于兼容性
    // 实际处理逻辑已内联到loop0以获得最大性能
}

// HID触摸输出处理 - 接收来自CPU0的触摸位图并转换为HID报告
void InputManager::process_hid_touch_output(uint32_t touch_bitmap) {
    if (!hid_device_) return;
    
    // 处理逻辑点位到XY坐标的映射
    for (uint8_t i = 0; i < 34; i++) {
        bool touched = (touch_bitmap & (1UL << i)) != 0;
        
        if (hid_mapping_.logical_points[i].enabled) {
            uint16_t x = hid_mapping_.logical_points[i].x_coord;
            uint16_t y = hid_mapping_.logical_points[i].y_coord;
            
            if (touched) {
                hid_device_->set_touch_point(i, x, y, 255);
            } else {
                hid_device_->release_touch_point(i);
            }
        }
    }
    
    // 发送HID报告
    hid_device_->send_report();
}

// 键盘映射处理 - 处理MCP23S17按键状态变化
void InputManager::process_keyboard_mapping(uint16_t key_state, uint16_t changed) {
    if (!hid_device_) return;
    
    // 处理变化的按键
    while (changed) {
        uint8_t pin = __builtin_ctz(changed);
        changed &= changed - 1;
        
        bool pressed = (key_state & (1 << pin)) != 0;
        
        // 查找对应的键盘映射
        for (uint8_t i = 0; i < 16; i++) {
            if (keyboard_mapping_.keys[i].enabled && 
                keyboard_mapping_.keys[i].gpio_pin == pin &&
                keyboard_mapping_.keys[i].is_mcp_pin) {
                
                // 处理组合键 (最多3个HID键)
                for (uint8_t key_idx = 0; key_idx < 3; key_idx++) {
                    uint8_t hid_key = keyboard_mapping_.keys[i].hid_keys[key_idx];
                    if (hid_key == 0) break; // 0表示无效键
                    
                    if (pressed) {
                        hid_device_->press_key(static_cast<HID_KeyCode>(hid_key));
                    } else {
                        hid_device_->release_key(static_cast<HID_KeyCode>(hid_key));
                    }
                }
                break;
            }
        }
    }
    
    // 发送HID键盘报告
    hid_device_->send_keyboard_report();
}
}

void InputManager::process_touch_event(uint8_t point_index, bool pressed, uint8_t pressure) {
    if (!is_point_valid(point_index) || !get_point_enabled(point_index)) {
        return;
    }
    
    bool current_state = current_states_[point_index];
    
    // 直通处理 - 无防抖和延迟
    if (pressed != current_state) {
        if (pressed) {
            handle_touch_press(point_index, pressure);
        } else {
            handle_touch_release(point_index);
        }
        
        current_states_[point_index] = pressed;
        
        // 执行映射
        execute_mapping(point_index, pressed);
        
        // 更新统计
        update_statistics(point_index, pressed);
    }
}

void InputManager::handle_touch_press(uint8_t point_index, uint8_t pressure) {
    TouchEvent event;
    event.type = TouchEvent::Type::PRESS;
    event.point_index = point_index;
    point_to_coordinates(point_index, event.x, event.y);
    event.pressure = pressure;
    event.timestamp = time_us_32() / 1000;
    
    if (touch_event_callback_) {
        touch_event_callback_(event);
    }
    
    log_debug("Touch press: point " + std::to_string(point_index) + ", pressure " + std::to_string(pressure));
}

void InputManager::handle_touch_release(uint8_t point_index) {
    TouchEvent event;
    event.type = TouchEvent::Type::RELEASE;
    event.point_index = point_index;
    point_to_coordinates(point_index, event.x, event.y);
    event.pressure = 0;
    event.timestamp = time_us_32() / 1000;
    event.duration = 0;  // 移除压力处理，直接设为0
    
    if (touch_event_callback_) {
        touch_event_callback_(event);
    }
    
    log_debug("Touch release: point " + std::to_string(point_index));
}

void InputManager::handle_touch_hold(uint8_t point_index) {
    TouchEvent event;
    event.type = TouchEvent::Type::HOLD;
    event.point_index = point_index;
    point_to_coordinates(point_index, event.x, event.y);
    event.pressure = 0;  // 移除压力处理，直接设为0
    event.timestamp = time_us_32() / 1000;
    event.duration = 0;  // 移除压力处理，直接设为0
    
    if (touch_event_callback_) {
        touch_event_callback_(event);
    }
    
    log_debug("Touch hold: point " + std::to_string(point_index));
}

void InputManager::execute_mapping(uint8_t point_index, bool pressed) {
    if (!is_point_valid(point_index) || !mappings_[point_index].enabled) {
        return;
    }
    
    const InputMapping& mapping = mappings_[point_index];
    
    // 检查映射的有效性和条件
    if (mapping.type == InputMapping::Type::NONE) {
        return;
    }
    
    // 记录映射执行时间用于调试
    uint32_t execution_start = time_us_32() / 1000;
    
    switch (mapping.type) {
        case InputMapping::Type::KEYBOARD:
            send_keyboard_input(mapping, pressed);
            break;
        case InputMapping::Type::MOUSE:
            send_mouse_input(mapping, pressed);
            break;
        case InputMapping::Type::GAMEPAD:
            send_gamepad_input(mapping, pressed);
            break;
        case InputMapping::Type::TOUCH:
            send_touch_input(mapping, pressed);
            break;
        case InputMapping::Type::CUSTOM:
            // 自定义处理 - 可扩展用户自定义映射逻辑
            if (mapping.custom_handler) {
                mapping.custom_handler(point_index, pressed, 255); // 使用最大压力值
            }
            break;
        default:
            // 未知映射类型，记录错误
            if (debug_enabled_) {
                log_debug("Unknown mapping type for point " + std::to_string(point_index));
            }
            return;
    }
    
    // 更新映射统计信息
    mapping_statistics_[point_index].execution_count++;
    mapping_statistics_[point_index].last_execution_time = execution_start;
    
    // 通知映射回调
    if (input_mapping_callback_) {
        input_mapping_callback_(point_index, pressed);
    }
    
    // 调试输出
    if (debug_enabled_) {
        uint32_t execution_time = (time_us_32() / 1000) - execution_start;
        if (execution_time > 1) { // 只记录耗时超过1ms的映射
            log_debug("Mapping execution for point " + std::to_string(point_index) + 
                     " took " + std::to_string(execution_time) + "ms");
        }
    }
}

void InputManager::send_keyboard_input(const InputMapping& mapping, bool pressed) {
    if (!hid_device_) return;
    
    HID_KeyCode key = static_cast<HID_KeyCode>(mapping.key_code);
    
    if (pressed) {
        hid_device_->press_key(key, mapping.modifier);
    } else {
        hid_device_->release_key(key);
    }
}

void InputManager::send_mouse_input(const InputMapping& mapping, bool pressed) {
    if (!hid_device_) return;
    
    HID_MouseButton button = static_cast<HID_MouseButton>(mapping.key_code);
    
    if (pressed) {
        hid_device_->press_mouse_button(button);
    } else {
        hid_device_->release_mouse_button(button);
    }
}

void InputManager::send_gamepad_input(const InputMapping& mapping, bool pressed) {
    if (!hid_device_) return;
    
    HID_GamepadButton button = static_cast<HID_GamepadButton>(mapping.key_code);
    hid_device_->set_gamepad_button(button, pressed);
}

void InputManager::send_touch_input(const InputMapping& mapping, bool pressed) {
    if (!hid_device_) return;
    
    uint16_t x, y;
    if (point_to_coordinates(mapping.point_index, x, y)) {
        if (pressed) {
            hid_device_->set_touch_point(mapping.point_index, x, y, 255); // 使用最大压力值
        } else {
            hid_device_->release_touch_point(mapping.point_index);
        }
    }
}

bool InputManager::is_point_valid(uint8_t point_index) const {
    return point_index < 34;
}

uint8_t InputManager::device_channel_to_point_index(const std::string& device_name, uint8_t channel) const {
    // 查找设备在devices_映射中的位置索引
    uint8_t device_index = 0;
    for (const auto& pair : devices_) {
        if (pair.first == device_name) {
            return device_index * 12 + channel;
        }
        device_index++;
    }
    return 0xFF; // 设备未找到
}

void InputManager::point_index_to_device_channel(uint8_t point_index, std::string& device_name, uint8_t& channel) const {
    uint8_t device_index = point_index / 12;
    channel = point_index % 12;
    
    // 根据索引查找设备名称
    uint8_t current_index = 0;
    for (const auto& pair : devices_) {
        if (current_index == device_index) {
            device_name = pair.first;
            return;
        }
        current_index++;
    }
    device_name = ""; // 设备未找到
}

void InputManager::update_statistics(uint8_t point_index, bool pressed) {
    if (pressed) {
        statistics_.total_touches++;
        statistics_.valid_touches++;
        if (point_index < 34) {
            statistics_.point_counts[point_index]++;
        }
        
        // 检查多点触摸
        if (get_active_touch_count() > 1) {
            statistics_.multi_touches++;
        }
    }
}

void InputManager::perform_auto_calibration() {
    static uint32_t last_calibration = 0;
    uint32_t current_time = time_us_32() / 1000;
    
    InputManager_PrivateConfig config = input_manager_get_config_copy();
    if (current_time - last_calibration >= config.calibration_interval_s * 1000) {
        if (!calibrating_) {
            start_calibration(0xFF); // 校准所有设备
        }
        last_calibration = current_time;
    }
}

void InputManager::check_device_connections() {
    for (auto& pair : devices_) {
        DeviceInfo& info = pair.second;
        bool current_connected = info.device && info.device->is_ready();
        
        if (current_connected != info.connected) {
            info.connected = current_connected;
            
            if (device_status_callback_) {
                device_status_callback_(pair.first, current_connected);
            }
            
            log_debug("Device " + std::to_string(pair.first) + " " + 
                     (current_connected ? "connected" : "disconnected"));
        }
    }
}

void InputManager::log_debug(const std::string& message) {
    if (debug_enabled_) {
        USB_LOG_TAG_DEBUG("INPUT_MGR", "%s", message.c_str());
    }
}

void InputManager::process_mcp23s17_input(const MCP23S17_GPIO_State& gpio_state) {
    // 处理MCP23S17的GPIO输入状态
    // 将GPIO状态映射到键盘逻辑区
    
    // 检查每个GPIO引脚的状态变化
    static MCP23S17_GPIO_State last_gpio_state = {0, 0};
    
    // 检查PORTA的变化
    uint8_t porta_changed = gpio_state.port_a ^ last_gpio_state.port_a;
    if (porta_changed) {
        for (uint8_t i = 0; i < 8; i++) {
            if (porta_changed & (1 << i)) {
                bool pressed = (gpio_state.port_a & (1 << i)) == 0; // 假设低电平有效
                // 映射到键盘逻辑区
                process_key_input(i, pressed);
            }
        }
    }
    
    // 检查PORTB的变化
    uint8_t portb_changed = gpio_state.port_b ^ last_gpio_state.port_b;
    if (portb_changed) {
        for (uint8_t i = 0; i < 8; i++) {
            if (portb_changed & (1 << i)) {
                bool pressed = (gpio_state.port_b & (1 << i)) == 0; // 假设低电平有效
                // 映射到键盘逻辑区
                process_key_input(i + 8, pressed);
            }
        }
    }
    
    last_gpio_state = gpio_state;
}

void InputManager::process_hid_keyboard_mapping() {
    if (!hid_device_) return;
    
    static uint8_t hid_keys[6] = {0}; // HID键盘报告最多6个按键
    static uint8_t modifiers = 0;
    uint8_t key_count = 0;
    
    // 快速扫描所有键盘映射
     for (uint8_t i = 0; i < 34; i++) {
         if (current_states_[i] && mappings_[i].enabled && 
             mappings_[i].type == InputMappingType::KEYBOARD) {
             
             uint32_t keycode = mappings_[i].keyboard.keycode;
             
             // 处理修饰键
             if (keycode >= 0xE0 && keycode <= 0xE7) {
                 modifiers |= (1 << (keycode - 0xE0));
             } else if (key_count < 6) {
                 hid_keys[key_count++] = keycode;
             }
         }
     }
    
    // 发送HID键盘报告
    uint8_t report[8] = {modifiers, 0}; // 修饰键 + 保留字节
    memcpy(&report[2], hid_keys, 6);
    hid_device_->send_keyboard_report(report, 8);
    
    // 清空状态
    memset(hid_keys, 0, 6);
    modifiers = 0;
}

void InputManager::process_hid_touch_mapping() {
    if (!hid_device_) return;
    
    struct TouchReport {
        uint8_t contact_id;
        uint16_t x, y;
        uint8_t pressure;
    } touches[10]; // 最多10个触摸点
    
    uint8_t touch_count = 0;
    
    // 扫描所有触摸映射
     for (uint8_t i = 0; i < 34; i++) {
         if (current_states_[i] && mappings_[i].enabled && 
             mappings_[i].type == InputMappingType::TOUCH && touch_count < 10) {
             
             touches[touch_count].contact_id = i;
             touches[touch_count].x = mappings_[i].touch.x;
             touches[touch_count].y = mappings_[i].touch.y;
             touches[touch_count].pressure = 255; // 最大压力
             touch_count++;
         }
     }
    
    // 发送HID触摸报告
    if (touch_count > 0) {
        hid_device_->send_touch_report((uint8_t*)touches, touch_count * sizeof(TouchReport));
    }
}

void InputManager::process_key_input(uint8_t key_index, bool pressed) {
    // 处理单个按键输入 - 直通处理无防抖
    
    if (key_index >= 16) { // MCP23S17最多16个GPIO
        return;
    }
    
    // 直接处理MCP23S17按键映射
    if (hid_device_) {
        // 查找对应的映射配置
         for (uint8_t i = 0; i < 34; i++) {
             if (mappings_[i].enabled && mappings_[i].type == InputMappingType::KEYBOARD &&
                 mappings_[i].keyboard.keycode == key_index + 0x04) { // HID键码偏移
                 
                 uint8_t report[8] = {0};
                 if (pressed) {
                     report[2] = mappings_[i].keyboard.keycode;
                 }
                 hid_device_->send_keyboard_report(report, 8);
                 break;
             }
         }
    }
    
    log_debug("Key " + std::to_string(key_index) + (pressed ? " pressed" : " released"));
}

// 生成设备名称：I2C通道+地址
std::string InputManager::generate_device_name(GTX312L* device) {
    if (!device) {
        return "Unknown";
    }
    
    // 假设GTX312L有获取I2C通道和地址的方法
    // 这里需要根据实际的GTX312L实现来调整
    uint8_t i2c_channel = 0; // 需要从device获取
    uint8_t i2c_address = device->get_address();
    
    return "I2C" + std::to_string(i2c_channel) + "_0x" + 
           std::to_string(i2c_address);
}

// 根据设备名称查找设备
GTX312L* InputManager::find_device_by_name(const std::string& device_name) {
    auto it = devices_.find(device_name);
    if (it != devices_.end()) {
        return it->second.device;
    }
    return nullptr;
}

// 获取所有设备名称
std::vector<std::string> InputManager::get_device_names() {
    std::vector<std::string> names;
    for (const auto& pair : devices_) {
        names.push_back(pair.first);
    }
    return names;
}

// 通过设备名称设置通道灵敏度
bool InputManager::set_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t sensitivity) {
    if (sensitivity > 63) {
        return false;
    }
    
    auto it = devices_.find(device_name);
    if (it != devices_.end() && it->second.device) {
        bool result = it->second.device->set_channel_sensitivity(channel, sensitivity);
        if (result) {
            log_debug("Device " + device_name + " channel " + std::to_string(channel) + " sensitivity set to " + std::to_string(sensitivity));
        }
        return result;
    }
    
    return false;
}

// 通过设备名称获取通道灵敏度
bool InputManager::get_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t& sensitivity) {
    auto it = devices_.find(device_name);
    if (it != devices_.end() && it->second.device) {
        return it->second.device->get_channel_sensitivity(channel, sensitivity);
    }
    
    return false;
}

// 通过设备名称获取通道触摸状态
bool InputManager::get_channel_touch_state_by_name(const std::string& device_name, uint8_t channel, bool& pressed, uint8_t& pressure) {
    auto it = devices_.find(device_name);
    if (it != devices_.end() && it->second.device) {
        // 这里需要根据GTX312L的实际API来实现
        // 假设有get_channel_touch_state方法
        return it->second.device->get_channel_touch_state(channel, pressed, pressure);
    }
    
    return false;
}