#include "input_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include <cstring>
#include <algorithm>
#include "src/protocol/usb_serial_logs/usb_serial_logs.h"

// 静态实例
InputManager *InputManager::instance_ = nullptr;
// 静态配置变量
static InputManager_PrivateConfig static_config_;
// Debug开关静态变量
bool InputManager::debug_enabled_ = false;
// 频率限制相关静态成员变量
uint32_t InputManager::min_interval_us_ = 8333;  // 默认120Hz对应的间隔时间（微秒）

// 单例模式实现
InputManager *InputManager::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new InputManager();
    }
    return instance_;
}

// 私有构造函数
InputManager::InputManager()
    : delay_buffer_head_(0), delay_buffer_count_(0), mcu_gpio_states_(0), mcu_gpio_previous_states_(0), serial_state_(), touch_keyboard_current_time_cache_(0), touch_keyboard_areas_matched_cache_(false), touch_keyboard_hold_satisfied_cache_(false), sample_counter_(0), last_reset_time_(0), current_sample_rate_(0), binding_active_(false), binding_callback_(), binding_state_(BindingState::IDLE), current_binding_index_(0), binding_start_time_(0), binding_timeout_ms_(30000),
      binding_hardware_ops_pending_(false), binding_cancel_pending_(false), calibration_request_pending_(CalibrationRequestType::IDLE), calibration_sensitivity_target_(2), calibration_in_progress_(false), mai2_serial_(nullptr), hid_(nullptr), mcp23s17_(nullptr), config_(inputmanager_get_config_holder()), mcp23s17_available_(false), ui_manager_(nullptr), gpio_keyboard_bitmap_(), touch_bitmap_cache_(), mcp_gpio_states_(), mcp_gpio_previous_states_(), last_sent_serial_state_(), remaining_extra_sends_(0), serial_state_changed_(false)
{

    // 初始化32位触摸状态数组
    for (int32_t i = 0; i < 8; i++)
    {
        touch_device_states_[i] = TouchDeviceState();
    }
    memset(original_channels_backup_, 0, sizeof(original_channels_backup_));

    // 初始化I2C采样stage队列系统
    for (int i = 0; i < 2; i++) {
        i2c_sampling_stages_[i] = I2C_SamplingStage();
    }

    // 初始化MCP GPIO状态
    mcp_gpio_states_.port_a = 0;
    mcp_gpio_states_.port_b = 0;
    mcp_gpio_states_.timestamp = 0;
    mcp_gpio_previous_states_ = mcp_gpio_states_;
}

// 析构函数
InputManager::~InputManager()
{
    deinit();
}

// 初始化
bool InputManager::init(const InitConfig &config)
{
    // 设置外部实例引用
    mai2_serial_ = config.mai2_serial;
    hid_ = config.hid;
    mcp23s17_ = config.mcp23s17;
    mcp23s17_available_ = (mcp23s17_ != nullptr);
    ui_manager_ = config.ui_manager;
    
    // 加载配置
    inputmanager_load_config_from_manager();
    
    // 应用mai2serial配置到实例
    if (mai2_serial_) {
        mai2_serial_->set_config(config_->mai2serial_config);
    }
    // 初始化32位触摸状态数组
    for (int32_t i = 0; i < 8; i++)
    {
        touch_device_states_[i] = TouchDeviceState();
    }

    // 初始化GPIO状态
    mcu_gpio_states_ = 0;
    mcu_gpio_previous_states_ = 0;
    mcp_gpio_states_.port_a = 0;
    mcp_gpio_states_.port_b = 0;
    mcp_gpio_states_.timestamp = 0;
    mcp_gpio_previous_states_ = mcp_gpio_states_;

    return true;
}

// 去初始化
void InputManager::deinit()
{
    // 保存配置
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    inputmanager_write_config_to_manager(*config);

    // 取消所有活动操作
    cancelBinding();

    // 清空设备列表
    touch_sensor_devices_.clear();

    // 重置配置中的设备计数
    config->device_count = 0;
}

// 启动函数 - 分配设备到采样阶段
void InputManager::start() {
    log_info("Starting InputManager - assigning devices to sampling stages");
    
    // 首先清空所有阶段
    for (int bus = 0; bus < 2; bus++) {
        for (int stage = 0; stage < 4; stage++) {
            i2c_sampling_stages_[bus].device_instances[stage] = nullptr;
        }
    }
    
    // 按配置中的阶段分配优先处理
    for (const auto& assignment : config_->stage_assignments) {
        if (assignment.i2c_bus < 2 && assignment.stage < 4 && assignment.device_id != 0xFF) {
            // 使用registerDeviceToStage注册设备
            if (registerDeviceToStage(assignment.stage, assignment.device_id)) {
                log_debug("Assigned device ID " + std::to_string(assignment.device_id) + 
                         " to I2C" + std::to_string(assignment.i2c_bus) + 
                         " stage " + std::to_string(assignment.stage) + " (from config)");
            }
        }
    }
    
    // 按注册顺序分配未配置的设备到空闲阶段
    for (TouchSensor* device : touch_sensor_devices_) {
        if (!device) continue;
        
        uint8_t device_id = device->getModuleMask();
        bool already_assigned = false;
        
        // 检查设备是否已经被分配
        for (int bus = 0; bus < 2; bus++) {
            for (int stage = 0; stage < 4; stage++) {
                if (i2c_sampling_stages_[bus].device_instances[stage] == device) {
                    already_assigned = true;
                    break;
                }
            }
            if (already_assigned) break;
        }
        
        if (already_assigned) continue;
        
        // 获取设备的I2C总线信息
        uint8_t device_i2c_bus = TouchSensor::extractI2CBusFromMask(device->getModuleMask());
        if (device_i2c_bus >= 2) continue;  // 无效总线
        
        // 在对应总线上查找空闲阶段
        bool assigned = false;
        for (int stage = 0; stage < 4; stage++) {
            if (i2c_sampling_stages_[device_i2c_bus].device_instances[stage] == nullptr) {
                // 使用registerDeviceToStage注册设备
                if (registerDeviceToStage(stage, device_id)) {
                    log_debug("Assigned device ID " + std::to_string(device_id) + 
                             " to I2C" + std::to_string(device_i2c_bus) + 
                             " stage " + std::to_string(stage) + " (auto-assigned)");
                    assigned = true;
                    break;
                }
            }
        }
        
        if (!assigned) {
            log_warning("Failed to assign device ID " + std::to_string(device_id) + 
                       " - no free stages on I2C" + std::to_string(device_i2c_bus));
        }
    }
    
    log_info("InputManager start completed");
}

// 注册TouchSensor设备
// TouchSensor统一接口实现
bool InputManager::registerTouchSensor(TouchSensor *device)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    if (!device || config->device_count >= 8)
    {
        log_error("Failed to register touch sensor: device is null or max device count reached");
        return false;
    }
    // 提取8位设备掩码（高8位）
    uint8_t device_id_mask = device->getModuleMask();
    ;

    // 检查mask是否为全0，如果是则忽略该设备
    if (device_id_mask == 0)
    {
        log_warning("Ignoring touch sensor with zero mask: " + device->getDeviceName());
        return false;
    }

    // 通过新的sample()接口获取设备信息
    uint32_t supported_channels = device->getSupportedChannelCount();

    // 检查是否已经在配置中存在该设备映射
    for (int i = 0; i < config->device_count; ++i)
    {
        if (config->touch_device_mappings[i].device_id_mask == device_id_mask)
        {
            // 设备映射已存在，更新连接状态为已连接
            config->touch_device_mappings[i].is_connected = true;

            // 添加到TouchSensor设备列表（如果尚未添加）
            bool device_in_list = false;
            for (auto *existing_device : touch_sensor_devices_)
            {
                if (existing_device == device)
                {
                    device_in_list = true;
                    break;
                }
            }
            if (!device_in_list)
            {
                touch_sensor_devices_.push_back(device);
            }

            // 当设备存在自定义配置时 加载他（修复：确保已存在设备也能加载配置）
            load_touch_device_config(device);

            log_info("设备已连接: " + device->getDeviceName() + " (ID掩码: 0x" + std::to_string(device_id_mask) + ")");
            return true; // 成功更新连接状态
        }
    }
    // 当设备存在自定义配置时 加载他
    load_touch_device_config(device);

    // 添加到TouchSensor设备列表
    touch_sensor_devices_.push_back(device);

    // 初始化TouchDeviceMapping
    TouchDeviceMapping &touch_mapping = config->touch_device_mappings[config->device_count];
    touch_mapping.device_id_mask = device_id_mask;
    touch_mapping.max_channels = std::min((uint8_t)supported_channels, (uint8_t)24u); // 最大24通道
    touch_mapping.is_connected = true;                                                // 新注册的设备标记为已连接

    // 初始化默认映射
    for (uint8_t i = 0; i < touch_mapping.max_channels; i++)
    {
        touch_mapping.sensitivity[i] = 15; // 默认灵敏度
    }
    touch_mapping.enabled_channels_mask = (1u << touch_mapping.max_channels) - 1; // 启用所有支持的通道

    config->device_count++;

    log_debug("Registered touch sensor: " + device->getDeviceName());

    return true;
}

void InputManager::unregisterTouchSensor(TouchSensor *device)
{
    if (!device)
        return;

    // 获取设备ID掩码
    uint8_t device_id_mask = device->getModuleMask();

    // 从TouchSensor设备列表中移除
    auto it = std::find(touch_sensor_devices_.begin(), touch_sensor_devices_.end(), device);
    if (it != touch_sensor_devices_.end())
    {
        touch_sensor_devices_.erase(it);

        // 在配置映射中找到对应设备并标记为未连接
        InputManager_PrivateConfig *config = inputmanager_get_config_holder();
        for (int i = 0; i < config->device_count; i++)
        {
            if (config->touch_device_mappings[i].device_id_mask == device_id_mask)
            {
                config->touch_device_mappings[i].is_connected = false;
                log_info("设备已断开连接: " + device->getDeviceName() + " (ID掩码: 0x" + std::to_string(device_id_mask) + ")");
                break;
            }
        }
    }
}

void InputManager::load_touch_device_config(TouchSensor *device)
{
    if (!device) {
        return;
    }
    
    // 从TouchSensor实例中获取设备ID掩码，避免传入错误掩码
    uint8_t device_id_mask = device->getModuleMask();
    
    // 尝试加载该设备的自定义配置
    ConfigManager *config_mgr = ConfigManager::getInstance();
    if (config_mgr)
    {
        std::string config_key = "TOUCH_DEVICE_CONFIG_" + std::to_string(device_id_mask);
        std::string device_config = config_mgr->get_string_dynamic(config_key);
        if (!device_config.empty())
        {
            if (device->loadConfig(device_config))
            {
                log_info("已加载设备自定义配置: " + device->getDeviceName() + " (ID掩码: 0x" + std::to_string(device_id_mask) + ")");
            }
            else
            {
                log_warning("加载设备配置失败: " + device->getDeviceName() + " (ID掩码: 0x" + std::to_string(device_id_mask) + ")");
            }
        }
    }
}

// 物理键盘映射管理方法
bool InputManager::addPhysicalKeyboard(MCU_GPIO gpio, HID_KeyCode default_key)
{
    // 添加新映射
    PhysicalKeyboardMapping new_mapping(gpio, default_key);
    config_->physical_keyboard_mappings.push_back(new_mapping);
    return true;
}

bool InputManager::addPhysicalKeyboard(MCP_GPIO gpio, HID_KeyCode default_key)
{
    // 添加新映射
    PhysicalKeyboardMapping new_mapping(gpio, default_key);
    config_->physical_keyboard_mappings.push_back(new_mapping);
    return true;
}

bool InputManager::removePhysicalKeyboard(uint8_t gpio_pin)
{
    auto it = std::find_if(config_->physical_keyboard_mappings.begin(),
                           config_->physical_keyboard_mappings.end(),
                           [gpio_pin](const PhysicalKeyboardMapping &mapping)
                           {
                               return mapping.gpio == gpio_pin;
                           });

    if (it != config_->physical_keyboard_mappings.end())
    {
        config_->physical_keyboard_mappings.erase(it);
        return true;
    }

    return false;
}

inline void InputManager::processSerialModeWithDelay()
{
    // 函数级静态变量，避免暴露在类外面
    static Mai2Serial_TouchState delayed_serial_state;
    static uint32_t target_time;
    static uint16_t buffer_idx;
    static uint16_t last_hit_offset = 0;  // 上次命中位置相对于head的偏移
    static uint32_t current_timestamp;
    static uint16_t search_offset;
    static uint32_t last_rate_limit_time = 0;  // 上次发送时间（频率限制用）
    static uint32_t current_time = 0;  // 使用静态变量避免栈分配
    // 使用union位域优化bool变量存储，支持一次性重置所有标志
    static union {
        struct {
            uint8_t found : 1;  // 是否找到符合条件的数据
            uint8_t reserved : 7;  // 预留位，供未来扩展
        };
        uint8_t all_flags;  // 用于一次性重置所有标志位
    } flags;

    // 一次性重置所有标志位，降低CPU开销
    flags.all_flags = 0;
    
    // 如果缓冲区为空，直接返回
    if (delay_buffer_count_ == 0) {
        return;
    }
    
    // 频率限制检查（非阻塞流控）
    if (config_->rate_limit_enabled) {
        current_time = time_us_32();  // 更新当前时间
        
        // 检查是否到达发送时间
        if (last_rate_limit_time != 0 && (current_time - last_rate_limit_time) < min_interval_us_) {
            return;  // 未到发送时间，直接返回
        }
    }
    
    // 如果延迟为0，直接使用最新数据，无需搜索
    if (config_->touch_response_delay_ms == 0) {
        buffer_idx = (delay_buffer_head_ - 1) & (DELAY_BUFFER_SIZE - 1);
        delayed_serial_state = delay_buffer_[buffer_idx].serial_touch_state;
        goto process_aggregation;
    }
    
    // 计算目标时间点（当前时间减去延迟时间）
    target_time = time_us_32() - (config_->touch_response_delay_ms * 1000U);
    
    // 限制偏移范围，防止越界
    if (last_hit_offset >= delay_buffer_count_) {
        last_hit_offset = delay_buffer_count_ / 2;  // 重置到中间位置
    }
    
    // 从上次命中位置开始搜索
    buffer_idx = (delay_buffer_head_ - last_hit_offset) & (DELAY_BUFFER_SIZE - 1);
    current_timestamp = delay_buffer_[buffer_idx].timestamp_us;
    
    if (current_timestamp <= target_time) {
        // 当前位置时间戳过旧，需要向最新方向搜索最贴近目标的位置
        uint16_t best_idx = buffer_idx;
        uint16_t best_offset = last_hit_offset;
        
        // 向最新方向搜索，找到最后一个符合条件的位置
        for (search_offset = last_hit_offset - 1; search_offset > 0; --search_offset) {
            buffer_idx = (delay_buffer_head_ - search_offset) & (DELAY_BUFFER_SIZE - 1);
            if (delay_buffer_[buffer_idx].timestamp_us <= target_time) {
                best_idx = buffer_idx;
                best_offset = search_offset;
            } else {
                break;  // 找到第一个不符合的，停止搜索
            }
        }
        
        buffer_idx = best_idx;
        last_hit_offset = best_offset;
        delayed_serial_state = delay_buffer_[buffer_idx].serial_touch_state;
        goto process_aggregation;
    }
    
    // 当前位置时间戳过新，需要向过去方向搜索第一个符合条件的位置
    for (search_offset = last_hit_offset + 1; search_offset <= delay_buffer_count_; ++search_offset) {
        buffer_idx = (delay_buffer_head_ - search_offset) & (DELAY_BUFFER_SIZE - 1);
        if (delay_buffer_[buffer_idx].timestamp_us <= target_time) {
            last_hit_offset = search_offset;
            flags.found = 1;
            break;
        }
    }
    
    if (!flags.found) {
        // 没找到符合条件的，说明还没到发送时候
        return;
    }
    
    delayed_serial_state = delay_buffer_[buffer_idx].serial_touch_state;

process_aggregation:
    // 功能1: 触发数据聚合处理
    if (config_->data_aggregation_delay_ms > 0 && config_->touch_response_delay_ms >= config_->data_aggregation_delay_ms) {
        // 计算聚合时间范围的起始时间点
        uint32_t aggregation_start_time = time_us_32() - (config_->touch_response_delay_ms * 1000U);
        uint32_t aggregation_end_time = aggregation_start_time + (config_->data_aggregation_delay_ms * 1000U);
        
        // 遍历聚合时间范围内的所有数据，进行AND运算
        Mai2Serial_TouchState aggregated_state = delayed_serial_state;  // 初始化为当前状态
        
        for (uint16_t i = 0; i < delay_buffer_count_; ++i) {
            uint16_t check_idx = (delay_buffer_head_ - 1 - i) & (DELAY_BUFFER_SIZE - 1);
            uint32_t check_timestamp = delay_buffer_[check_idx].timestamp_us;
            
            // 如果时间戳在聚合范围内
            if (check_timestamp >= aggregation_start_time && check_timestamp <= aggregation_end_time) {
                // 进行AND运算：只有在整个时间段内都触发的通道才保持触发状态
                aggregated_state.parts.state1 &= delay_buffer_[check_idx].serial_touch_state.parts.state1;
                aggregated_state.parts.state2 &= delay_buffer_[check_idx].serial_touch_state.parts.state2;
            } else if (check_timestamp < aggregation_start_time) {
                break;  // 时间戳过旧，停止搜索
            }
        }
        
        delayed_serial_state = aggregated_state;
    }
    
    // 功能2: 仅改变时发送判断
    serial_state_changed_ = (delayed_serial_state.raw != last_sent_serial_state_.raw);
    
    bool should_send = false;
    
    if (config_->send_only_on_change) {
        // 如果启用仅改变时发送
        if (serial_state_changed_) {
            should_send = true;
            // 数据改变时重置额外发送次数
            remaining_extra_sends_ = config_->extra_send_count;
        } else if (remaining_extra_sends_ > 0) {
            // 数据未改变但还有额外发送次数
            should_send = true;
            remaining_extra_sends_--;
        }
    } else {
        // 未启用仅改变时发送，总是发送
        should_send = true;
        if (serial_state_changed_) {
            remaining_extra_sends_ = config_->extra_send_count;
        }
    }
    
    // 发送数据
    if (should_send) {
        serial_state_ = delayed_serial_state;  // 为确保当外键映射启用时时间是同步的
        // 仅在发送成功时更新last_sent_serial_state_，确保发送失败时保持差异检测
        if (mai2_serial_->send_touch_data(delayed_serial_state)) {
            last_sent_serial_state_ = delayed_serial_state;  // 更新上次发送状态
            
            // 更新频率限制时间戳（仅在发送成功时更新）
            if (config_->rate_limit_enabled) {
                last_rate_limit_time = current_time;
            }
        }
        // 注意：连续发送模式(send_only_on_change=false)不受发送成功与否影响
    }
}

void InputManager::clearPhysicalKeyboards()
{
    config_->physical_keyboard_mappings.clear();
}

// 触摸键盘映射管理方法
void InputManager::setTouchKeyboardEnabled(bool enabled)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    config->touch_keyboard_enabled = enabled;
}

bool InputManager::getTouchKeyboardEnabled() const
{
    return config_->touch_keyboard_enabled;
}

inline void InputManager::setTouchKeyboardMode(TouchKeyboardMode mode)
{
    config_->touch_keyboard_mode = mode;
}

inline TouchKeyboardMode InputManager::getTouchKeyboardMode() const
{
    return config_->touch_keyboard_mode;
}

// 触摸键盘映射管理方法实现
bool InputManager::addTouchKeyboardMapping(uint64_t area_mask, uint32_t hold_time_ms, HID_KeyCode key, bool trigger_once)
{
    // 检查参数有效性
    if (area_mask == 0 || key == HID_KeyCode::KEY_NONE) {
        return false;
    }
    
    // 检查是否已存在相同的映射
    for (const auto& mapping : config_->touch_keyboard_mappings) {
        if (mapping.area_mask == area_mask && mapping.key == key) {
            return false; // 已存在相同映射
        }
    }
    
    // 添加新映射
    TouchKeyboardMapping new_mapping(area_mask, hold_time_ms, key, trigger_once);
    config_->touch_keyboard_mappings.push_back(new_mapping);
    
    return true;
}

bool InputManager::removeTouchKeyboardMapping(uint64_t area_mask, HID_KeyCode key)
{
    auto it = std::find_if(config_->touch_keyboard_mappings.begin(),
                           config_->touch_keyboard_mappings.end(),
                           [area_mask, key](const TouchKeyboardMapping& mapping) {
                               return mapping.area_mask == area_mask && mapping.key == key;
                           });
    
    if (it != config_->touch_keyboard_mappings.end()) {
        config_->touch_keyboard_mappings.erase(it);
        return true;
    }
    
    return false;
}

const std::vector<TouchKeyboardMapping>& InputManager::getTouchKeyboardMappings() const
{
    return config_->touch_keyboard_mappings;
}

inline void InputManager::checkTouchKeyboardTrigger()
{       
    // 缓存当前时间，避免重复系统调用
    touch_keyboard_current_time_cache_ = us_to_ms(time_us_32());
    
    // 遍历所有触摸键盘映射，独立处理每个映射
    for (auto& mapping : config_->touch_keyboard_mappings) {
        // 使用位操作宏进行高效区域匹配检查
          touch_keyboard_areas_matched_cache_ = MAI2_TOUCH_CHECK_MASK(serial_state_, mapping.area_mask);
          if (__builtin_expect(touch_keyboard_areas_matched_cache_, 0)) {
              // 区域匹配，检查是否刚开始按下
              if (__builtin_expect(mapping.press_timestamp == 0, 0)) {
                  mapping.press_timestamp = touch_keyboard_current_time_cache_;
              }
  
              touch_keyboard_hold_satisfied_cache_ = (mapping.hold_time_ms == 0) || 
                                                    ((touch_keyboard_current_time_cache_ - mapping.press_timestamp) >= mapping.hold_time_ms);
            
            // 处理触发逻辑
            if (mapping.trigger_once) {
                // trigger_once：同一次触摸只触发一次；必须离开区域后才能再次触发；触发后下一次检查立即松开
                if (mapping.has_triggered == TOUCH_KEYBOARD_TRIGGLE_STAGE_NONE) {
                    if (__builtin_expect(touch_keyboard_hold_satisfied_cache_, 0)) {
                        hid_->press_key(mapping.key);
                        mapping.key_pressed = true;
                        mapping.has_triggered = TOUCH_KEYBOARD_TRIGGLE_STAGE_PRESS;
                    }
                } else if (mapping.has_triggered == TOUCH_KEYBOARD_TRIGGLE_STAGE_PRESS) {
                    // 立即松开（下一次检查）
                    hid_->release_key(mapping.key);
                    mapping.has_triggered = TOUCH_KEYBOARD_TRIGGLE_STAGE_RELEASE;
                }
                // RELEASE 状态下保持不触发，直到区域不匹配时在else分支中重置为 NONE
            } else {
                // 正常模式：满足条件就按下按键
                if (__builtin_expect(touch_keyboard_hold_satisfied_cache_ && !mapping.key_pressed, 0)) {
                    hid_->press_key(mapping.key);
                    mapping.key_pressed = true;
                }
            }
        } else {
            // 区域不匹配，释放按键
            if (__builtin_expect(mapping.key_pressed, 0)) {
                mapping.has_triggered = TOUCH_KEYBOARD_TRIGGLE_STAGE_NONE;
                hid_->release_key(mapping.key);
                mapping.key_pressed = false;
            }
            mapping.press_timestamp = 0;
        }
    }
    
    return;
}

// 设置工作模式
inline bool InputManager::setWorkMode(InputWorkMode mode)
{
    config_->work_mode = mode;
    return true;
}

// CPU0核心循环 - TouchSensor触摸采样和Serial/HID处理
void InputManager::task0()
{
    // 更新所有设备的触摸状态
    updateTouchStates();

    mai2_serial_->task();

    // 处理校准请求
    if (calibration_request_pending_ != CalibrationRequestType::IDLE)
        processCalibrationRequest();

    if (calibration_in_progress_) {
        getCalibrationProgress();
        return;
    }
    
    // 处理绑定状态
    if (binding_active_)
    {
        // 如果需要执行硬件操作，先执行
        if (binding_hardware_ops_pending_)
        {
            enableAllChannels();
            clearSerialMappings();
            binding_hardware_ops_pending_ = false;
        }
        processBinding();
        Mai2Serial_TouchState empty;
        mai2_serial_->send_touch_data(empty); // 直接发送空数据 让内部蒙版启动
        return;
    }

    if (getWorkMode() == InputWorkMode::SERIAL_MODE)
    {
        processSerialModeWithDelay();
    }
}

// CPU1核心循环 - 键盘处理和HID发送
void InputManager::task1()
{
    updateGPIOStates();
    processGPIOKeyboard(); // 现在直接调用HID的press_key/release_key方法

    switch (config_->work_mode)
    {
        case InputWorkMode::HID_MODE:
            sendHIDTouchData();
            break;
        case InputWorkMode::SERIAL_MODE:
            // 检查触摸键盘触发
            if (config_->touch_keyboard_enabled)
                checkTouchKeyboardTrigger();
            break;
        default:
            break;
    }
    hid_->task();
}

// 开始Serial绑定
void InputManager::startSerialBinding(InteractiveBindingCallback callback)
{
    if (getWorkMode() != InputWorkMode::SERIAL_MODE)
    {
        if (callback)
        {
            callback(false, "Not in Serial mode");
        }
        return;
    }

    // 初始化Serial绑定状态
    binding_active_ = true;
    binding_callback_ = callback;
    binding_state_ = BindingState::PREPARE;
    current_binding_index_ = 0;
    binding_start_time_ = to_ms_since_boot(get_absolute_time());
    binding_hardware_ops_pending_ = true; // 标记需要在task0中执行硬件操作
}

// 开始自动Serial绑定
bool InputManager::startAutoSerialBinding()
{
    if (getWorkMode() != InputWorkMode::SERIAL_MODE)
    {
        return false;
    }

    // 初始化自动绑定状态
    binding_active_ = true;
    binding_callback_ = nullptr; // 自动绑区不使用回调
    binding_state_ = BindingState::PREPARE;
    current_binding_index_ = 0;
    binding_start_time_ = to_ms_since_boot(get_absolute_time());
    binding_hardware_ops_pending_ = true; // 标记需要在task0中执行硬件操作

    return true;
}

// 取消绑定
void InputManager::cancelBinding()
{
    if (binding_active_)
    {
        // 重置绑定状态
        binding_active_ = false;
        binding_callback_ = nullptr;
        binding_state_ = BindingState::IDLE;
        binding_hardware_ops_pending_ = false;
        current_binding_index_ = 0;
        binding_start_time_ = 0;

        mai2_serial_->clear_manually_triggle_area();
    }
}

// 检查自动绑区是否完成
bool InputManager::isAutoSerialBindingComplete() const
{
    return binding_active_ && binding_state_ == BindingState::PROCESSING;
}

// 确认自动绑区结果
void InputManager::confirmAutoSerialBinding()
{
    log_debug("confirmAutoSerialBinding() called");

    if (binding_active_ && binding_state_ == BindingState::PROCESSING)
    {
        log_debug("Confirming auto Serial binding, updating channel states");
        // 绑定完成但不自动保存配置

        // 重置绑定状态
        binding_active_ = false;
        binding_callback_ = nullptr;
        binding_state_ = BindingState::IDLE;

        // 设置映射关系
        updateChannelStatesAfterBinding();
        log_debug("Auto Serial binding confirmed successfully");
    }
    else
    {
        log_warning("confirmAutoSerialBinding called but not in correct binding state");
    }
}

// 获取当前绑定状态
BindingState InputManager::getBindingState() const
{
    return binding_state_;
}

// 获取当前绑定区域索引
uint8_t InputManager::getCurrentBindingIndex() const
{
    return current_binding_index_;
}

void InputManager::requestCancelBinding()
{
    // 设置取消标志位，让task0处理实际的取消操作
    binding_cancel_pending_ = true;
}

// 设置灵敏度
void InputManager::setSensitivity(uint8_t device_id_mask, uint8_t channel, uint8_t sensitivity)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels)
    {
        log_debug("setSensitivity: device_id_mask=" + std::to_string(device_id_mask) +
                  "channel=" + std::to_string(channel) +
                  "sensitivity=" + std::to_string(sensitivity));
        mapping->sensitivity[channel] = sensitivity;
        // 使用新的TouchSensor接口设置通道灵敏度
        TouchSensor *device = findTouchSensorByIdMask(device_id_mask);
        if (device)
        {
            // 将内部灵敏度值转换为0-99范围
            uint8_t normalized_sensitivity = (sensitivity > 99) ? 99 : sensitivity;
            device->setChannelSensitivity(channel, normalized_sensitivity);
        }
    }
}

uint8_t InputManager::getSensitivity(uint8_t device_id_mask, uint8_t channel)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    return (mapping && channel < mapping->max_channels) ? mapping->sensitivity[channel] : 15;
}

bool InputManager::setSensitivityByDeviceName(const std::string &device_name, uint8_t channel, uint8_t sensitivity)
{
    // 通过设备名称设置灵敏度
    for (size_t i = 0; i < touch_sensor_devices_.size(); ++i)
    {
        TouchSensor *device = touch_sensor_devices_[i];
        if (device && device->getDeviceName() == device_name)
        {
            setSensitivity(device->getModuleMask(), channel, sensitivity);
            return true;
        }
    }
    return false;
}

// 设置Serial映射 - 反向映射版本
void InputManager::setSerialMapping(uint8_t device_id_mask, uint8_t channel, Mai2_TouchArea area)
{
    if (area >= 1 && area <= 34)
    {
        // 反向映射：区域 -> 通道，使用32位物理通道地址，索引0-33对应区域1-34
        static_config_.area_channel_mappings.serial_mappings[area - 1].channel = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    }
}

void InputManager::setHIDMapping(uint8_t device_id_mask, uint8_t channel, float x, float y)
{
    // 反向映射：找到空闲的HID区域或复用现有区域
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);

    // 查找是否已有该通道的映射
    int target_index = -1;
    for (int i = 0; i < 10; i++)
    {
        if (static_config_.area_channel_mappings.hid_mappings[i].channel == physical_address)
        {
            target_index = i;
            break;
        }
    }

    // 如果没有找到，寻找空闲位置
    if (target_index == -1)
    {
        for (int i = 0; i < 10; i++)
        {
            if (static_config_.area_channel_mappings.hid_mappings[i].channel == 0xFFFFFFFF)
            {
                target_index = i;
                break;
            }
        }
    }

    // 设置HID映射
    if (target_index != -1)
    {
        static_config_.area_channel_mappings.hid_mappings[target_index].channel = physical_address;
        static_config_.area_channel_mappings.hid_mappings[target_index].coordinates = {x, y};
    }
}

Mai2_TouchArea InputManager::getSerialMapping(uint8_t device_id_mask, uint8_t channel)
{
    // 反向查找：通过通道找到对应的Mai2区域，索引0-33对应区域1-34
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (uint8_t area_idx = 0; area_idx < 34; area_idx++)
    {
        if (static_config_.area_channel_mappings.serial_mappings[area_idx].channel == physical_address)
        {
            return (Mai2_TouchArea)(area_idx + 1);  // 返回区域1-34
        }
    }
    return MAI2_NO_USED;
}

TouchAxis InputManager::getHIDMapping(uint8_t device_id_mask, uint8_t channel)
{
    // 反向查找：通过通道找到对应的HID坐标
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (int i = 0; i < 10; i++)
    {
        if (static_config_.area_channel_mappings.hid_mappings[i].channel == physical_address)
        {
            return static_config_.area_channel_mappings.hid_mappings[i].coordinates;
        }
    }
    return TouchAxis{0.0f, 0.0f};
}

void InputManager::setTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel, HID_KeyCode key)
{
    // 反向映射：按键 -> 通道，使用32位物理通道地址
    static_config_.area_channel_mappings.keyboard_mappings[key].channel = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
}

HID_KeyCode InputManager::getTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel)
{
    // 反向查找：通过通道找到对应的按键
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (const auto &kb_pair : static_config_.area_channel_mappings.keyboard_mappings)
    {
        if (kb_pair.second.channel == physical_address)
        {
            return kb_pair.first;
        }
    }
    return HID_KeyCode::KEY_NONE;
}

bool InputManager::hasAvailableSerialMapping() const
{
    // 检查是否所有34个区域都已映射
    for (int i = 0; i < 34; i++)
    {
        if (static_config_.area_channel_mappings.serial_mappings[i].channel == 0xFFFFFFFF || 
            static_config_.area_channel_mappings.serial_mappings[i].channel == 0)
        {
            return false;  // 发现未映射的区域，返回false
        }
    }
    return true;  // 所有区域都已映射
}

// 设备级别的灵敏度管理接口
void InputManager::setDeviceChannelSensitivity(uint8_t device_id_mask, uint8_t channel, uint8_t sensitivity)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels)
    {
        mapping->setChannelSensitivity(channel, sensitivity);

        // 同时更新硬件设备的灵敏度
        for (auto *device : touch_sensor_devices_)
        {
            if (device->getModuleMask() == device_id_mask)
            {
                device->setChannelSensitivity(channel, sensitivity);
                break;
            }
        }
    }
}

uint8_t InputManager::getDeviceChannelSensitivity(uint8_t device_id_mask, uint8_t channel)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels)
    {
        return mapping->sensitivity[channel];
    }
    return 15; // 默认灵敏度
}

// 通过逻辑区域映射设置物理通道灵敏度
void InputManager::setSerialAreaSensitivity(Mai2_TouchArea area, uint8_t sensitivity)
{
    if (area < 1 || area > 34)
        return;

    // 遍历所有设备映射，找到绑定该区域的通道，索引0-33对应区域1-34
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (auto &mapping : config->touch_device_mappings)
    {
        const auto &area_mapping = static_config_.area_channel_mappings.serial_mappings[area - 1];  // 区域1-34对应索引0-33
        if (area_mapping.channel != 0xFFFFFFFF)
        {
            // 从32位物理地址解码出通道号
            uint8_t channel = decodeChannelNumber(area_mapping.channel);
            setDeviceChannelSensitivity(mapping.device_id_mask, channel, sensitivity);
        }
    }
}

void InputManager::setHIDAreaSensitivity(uint8_t hid_area_index, uint8_t sensitivity)
{
    if (hid_area_index >= 10)
        return;

    // 遍历所有设备映射，找到对应的HID区域
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (auto &mapping : config->touch_device_mappings)
    {
        const auto &hid_mapping = static_config_.area_channel_mappings.hid_mappings[hid_area_index];
        if (hid_mapping.channel != 0xFFFFFFFF)
        {
            // 从32位物理地址解码出通道号
            uint8_t channel = decodeChannelNumber(hid_mapping.channel);
            setDeviceChannelSensitivity(mapping.device_id_mask, channel, sensitivity);
        }
    }
}

void InputManager::setKeyboardSensitivity(HID_KeyCode key, uint8_t sensitivity)
{
    // 遍历所有设备映射，找到绑定该按键的通道
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (auto &mapping : config->touch_device_mappings)
    {
        auto it = static_config_.area_channel_mappings.keyboard_mappings.find(key);
        if (it != static_config_.area_channel_mappings.keyboard_mappings.end() && it->second.channel != 0xFFFFFFFF)
        {
            // 从32位物理地址解码出通道号
            uint8_t channel = decodeChannelNumber(it->second.channel);
            setDeviceChannelSensitivity(mapping.device_id_mask, channel, sensitivity);
        }
    }
}

// 启用所有通道 - 使用TouchSensor统一接口
void InputManager::enableAllChannels()
{
    // 使用新的TouchSensor接口启用所有通道
    for (auto *device : touch_sensor_devices_)
    {
        uint32_t supported_channels = device->getSupportedChannelCount();

        for (uint8_t ch = 0; ch < supported_channels; ch++)
        {
            device->setChannelEnabled(ch, true);
        }
    }
}

// 仅启用已映射的通道 - 使用TouchSensor统一接口
void InputManager::enableMappedChannels()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();

    for (int i = 0; i < MAX_TOUCH_DEVICE && i < touch_sensor_devices_.size(); i++)
    {
        auto *device = touch_sensor_devices_[i];
        auto &mapping = config->touch_device_mappings[i];
        uint32_t supported_channels = device->getSupportedChannelCount();

        for (uint8_t ch = 0; ch < supported_channels && ch < mapping.max_channels; ch++)
        {
            bool has_mapping = false;
            uint32_t physical_address = encodePhysicalChannelAddress(mapping.device_id_mask, 1 << ch);

            if (work_mode == InputWorkMode::SERIAL_MODE)
            {
                // 使用反向映射查找Serial区域
                has_mapping = false;
                for (int area_idx = 1; area_idx <= 34; area_idx++)
                {
                    if (static_config_.area_channel_mappings.serial_mappings[area_idx - 1].channel == physical_address)
                    {
                        has_mapping = true;
                        break;
                    }
                }
            }
            else if (work_mode == InputWorkMode::HID_MODE)
            {
                // 使用反向映射查找HID坐标
                has_mapping = false;
                for (const auto &hid_mapping : static_config_.area_channel_mappings.hid_mappings)
                {
                    if (hid_mapping.channel == physical_address)
                    {
                        has_mapping = (hid_mapping.coordinates.x != 0.0f || hid_mapping.coordinates.y != 0.0f);
                        break;
                    }
                }
            }

            // 只有在enabled_channels_mask中启用且有映射的通道才启用
            bool ch_available = (mapping.enabled_channels_mask & (1 << ch)) != 0;
            bool enabled = ch_available && has_mapping;

            // 使用新的TouchSensor接口设置通道使能状态
            device->setChannelEnabled(ch, enabled);
        }
    }
}

// 清空当前绑区的串口映射
void InputManager::clearSerialMappings()
{
    // 清空所有串口映射，将所有区域设置为未映射状态
    for (int area_idx = 0; area_idx < 34; area_idx++)
    {
        static_config_.area_channel_mappings.serial_mappings[area_idx].channel = 0xFFFFFFFF; // 0xFFFFFFFF表示未映射
    }
    
    log_info("Serial mappings cleared");
}

void InputManager::updateChannelStatesAfterBinding()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();

    for (int i = 0; i < MAX_TOUCH_DEVICE; i++)
    {
        auto &mapping = config->touch_device_mappings[i];

        for (uint8_t ch = 0; ch < mapping.max_channels; ch++)
        {
            bool has_mapping = false;
            uint32_t physical_address = encodePhysicalChannelAddress(mapping.device_id_mask, 1 << ch);

            if (work_mode == InputWorkMode::SERIAL_MODE)
            {
                // 使用反向映射查找Serial区域
                has_mapping = false;
                for (int area_idx = 1; area_idx <= 34; area_idx++)
                {
                    if (static_config_.area_channel_mappings.serial_mappings[area_idx - 1].channel == physical_address)
                    {
                        has_mapping = true;
                        break;
                    }
                }
            }
            else if (work_mode == InputWorkMode::HID_MODE)
            {
                // 使用反向映射查找HID坐标
                has_mapping = false;
                for (const auto &hid_mapping : static_config_.area_channel_mappings.hid_mappings)
                {
                    if (hid_mapping.channel == physical_address)
                    {
                        has_mapping = (hid_mapping.coordinates.x != 0.0f || hid_mapping.coordinates.y != 0.0f);
                        break;
                    }
                }
            }

            // 如果没有映射，自动关闭enabled_channels_mask中的对应通道
            if (!has_mapping)
            {
                mapping.enabled_channels_mask &= ~(1 << ch); // 清除对应位
            }
        }
    }

    // 重新应用通道映射 TODO: 当前模块尚未debug完成 暂时别用
    //enableMappedChannels();
}

// 更新触摸状态 - 异步阶段性采样
inline void InputManager::updateTouchStates()
{
    static TouchSensor* _target_device = nullptr;

    // 遍历每个I2C总线，进行阶段性采样
    for (uint8_t bus = 0; bus < 2; bus++) {
        
        // 如果当前阶段被锁定（正在采样中），跳过
        if (i2c_sampling_stages_[bus].stage_locked) {
            continue;
        }
        
        // 获取当前阶段的设备实例
        _target_device = i2c_sampling_stages_[bus].device_instances[i2c_sampling_stages_[bus].current_stage];
        if (_target_device == nullptr) {
            i2c_sampling_stages_[bus].next_stage();
            continue;
        }
        
        // 检查当前设备是否准备好采样
        if (!_target_device->sample_ready()) {
            continue;
        }
        
        // 锁定当前阶段并发起异步采样
        i2c_sampling_stages_[bus].stage_locked = true;
        _target_device->sample(InputManager::async_touchsampleresult);
    }
}

// 处理自动校准控制 - 根据mai2serial发送状态控制AD7147设备的自动校准
inline void InputManager::updateAutoCalibrationControl()
{
    // 根据mai2serial发送状态控制自动校准
    static bool last_serial_ok = false;
    static bool current_serial_ok = false;
    current_serial_ok = mai2_serial_->get_serial_ok();
    if (current_serial_ok != last_serial_ok) {
        // 发送状态发生变化，更新所有AD7147设备的自动校准状态
        for (TouchSensor* sensor : touch_sensor_devices_) {
            sensor->setAutoCalibration(!current_serial_ok);
        }
        last_serial_ok = current_serial_ok;
    }
}

// 发送HID触摸数据 - 使用32位TouchSensor接口的统一实现
inline void InputManager::sendHIDTouchData()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();

    // 预计算32位触摸设备映射指针数组，避免重复查找
    TouchDeviceMapping *touch_mappings[8] = {nullptr};
    const int touch_device_count = config->device_count;

    // 预处理32位触摸设备映射，建立快速查找表
    for (int i = 0; i < touch_device_count; i++)
    {
        const uint32_t device_id_mask = touch_device_states_[i].current_touch_mask;
        touch_mappings[i] = findTouchDeviceMapping(device_id_mask);
    }

    // 处理所有32位触摸设备的HID数据
    for (int i = 0; i < touch_device_count; i++)
    {
        const TouchDeviceMapping *mapping = touch_mappings[i];
        if (!mapping)
            continue;

        const uint32_t current_touch_mask = touch_device_states_[i].current_touch_mask;
        if (!current_touch_mask)
            continue; // 无触摸数据

        const uint8_t max_channels = mapping->max_channels;

        // 使用位运算快速处理所有可用通道
        for (uint8_t ch = 0; ch < max_channels && ch < 32; ch++)
        {
            const uint32_t ch_mask = (1UL << ch);
            if (!(current_touch_mask & ch_mask))
                continue;

            // 检查是否有有效HID映射 - 使用反向映射查找
            TouchAxis hid_area = {0.0f, 0.0f};
            // 生成当前通道的32位物理地址
            uint32_t physical_address = encodePhysicalChannelAddress(mapping->device_id_mask, 1 << ch);
            // 查找该通道对应的HID坐标
            for (const auto &hid_mapping : static_config_.area_channel_mappings.hid_mappings)
            {
                if (hid_mapping.channel == physical_address)
                {
                    hid_area = hid_mapping.coordinates;
                    break;
                }
            }
            if (hid_area.x == 0.0f && hid_area.y == 0.0f)
                continue;

            // 计算唯一的触摸点ID：设备索引(3位) + 通道号(6位)
            // 支持最多8个设备，每个设备64个通道
            uint8_t unique_contact_id = ((i & 0x07) << 6) | (ch & 0x3F);

            // 创建触摸点报告
            HID_TouchPoint touch_point;
            touch_point.press = true;
            touch_point.id = unique_contact_id;

            // 转换坐标到HID范围 (0-65535)
            touch_point.x = (uint16_t)(hid_area.x * 65535.0f);
            touch_point.y = (uint16_t)(hid_area.y * 65535.0f);

            // 发送触摸点
            if (hid_)
            {
                hid_->send_touch_report(touch_point);
            }
        }
    }
}

// 地址转设备编号 0-N
int32_t InputManager::findTouchDeviceIndex(uint8_t device_id_mask)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (int i = 0; i < config->device_count; i++)
    {
        if (config->touch_device_mappings[i].device_id_mask == device_id_mask)
        {
            return i;
        }
    }
    return -1;
}

TouchDeviceMapping *InputManager::findTouchDeviceMapping(uint8_t device_id_mask)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    int index = findTouchDeviceIndex(device_id_mask);
    return (index >= 0) ? &config->touch_device_mappings[index] : nullptr;
}

TouchSensor *InputManager::findTouchSensorByIdMask(uint8_t device_id_mask)
{
    // 直接遍历touch_sensor_devices_向量查找匹配的设备
    for (TouchSensor* sensor : touch_sensor_devices_)
    {
        if (sensor && sensor->getModuleMask() == device_id_mask)
        {
            return sensor;
        }
    }
    return nullptr;
}

// [默认配置注册函数] - 注册所有InputManager的默认配置到ConfigManager
void inputmanager_register_default_configs(config_map_t &default_map)
{
    // 注册InputManager默认配置
    default_map[INPUTMANAGER_WORK_MODE] = ConfigValue((uint8_t)0);            // 默认工作模式
    default_map[INPUTMANAGER_TOUCH_KEYBOARD_ENABLED] = ConfigValue(false);    // 默认关闭触摸键盘
    default_map[INPUTMANAGER_TOUCH_KEYBOARD_MODE] = ConfigValue((uint8_t)0);  // 默认触摸键盘模式
    default_map[INPUTMANAGER_TOUCH_RESPONSE_DELAY] = ConfigValue((uint8_t)50, (uint8_t)0, (uint8_t)100); // 默认触摸响应延迟
    default_map[INPUTMANAGER_MAI2SERIAL_BAUD_RATE] = ConfigValue((uint32_t)9600, (uint32_t)9600, (uint32_t)6000000); // Mai2Serial波特率，范围9600-6000000
    
    // Serial模式新功能配置
    default_map[INPUTMANAGER_SEND_ONLY_ON_CHANGE] = ConfigValue(false);       // 默认关闭仅改变时发送
    default_map[INPUTMANAGER_DATA_AGGREGATION_DELAY] = ConfigValue((uint8_t)0, (uint8_t)0, (uint8_t)100); // 数据聚合延迟，范围0-100ms
    default_map[INPUTMANAGER_EXTRA_SEND_COUNT] = ConfigValue((uint8_t)0, (uint8_t)0, (uint8_t)10);        // 额外发送次数，范围0-10次
    
    // 频率限制配置
    default_map[INPUTMANAGER_RATE_LIMIT_ENABLED] = ConfigValue(false);        // 默认关闭频率限制
    default_map[INPUTMANAGER_RATE_LIMIT_FREQUENCY] = ConfigValue((uint16_t)120, (uint16_t)10, (uint16_t)1000); // 频率限制，范围10-1000Hz

    // 阶段分配配置
    default_map[INPUTMANAGER_STAGE_ASSIGNMENTS] = ConfigValue(std::string(""));  // 阶段分配配置

    default_map[INPUTMANAGER_TOUCH_DEVICES] = ConfigValue(std::string(""));      // 触摸设备映射数据
    default_map[INPUTMANAGER_PHYSICAL_KEYBOARDS] = ConfigValue(std::string(""));
    default_map[INPUTMANAGER_AREA_CHANNEL_MAPPINGS] = ConfigValue(std::string(""));  // 区域通道映射配置
}

// [配置保管函数] - 返回静态配置变量的指针
InputManager_PrivateConfig *inputmanager_get_config_holder()
{
    return &static_config_;
}

// [配置加载函数] - 从ConfigManager加载所有配置到静态配置变量
bool inputmanager_load_config_from_manager()
{
    ConfigManager *config_mgr = ConfigManager::getInstance();
    if (!config_mgr)
    {
        return false;
    }

    // 加载工作模式
    static_config_.work_mode = static_cast<InputWorkMode>(config_mgr->get_uint8(INPUTMANAGER_WORK_MODE));

    // 加载触摸键盘启用状态
    static_config_.touch_keyboard_enabled = config_mgr->get_bool(INPUTMANAGER_TOUCH_KEYBOARD_ENABLED);

    // 加载触摸键盘模式
    static_config_.touch_keyboard_mode = static_cast<TouchKeyboardMode>(config_mgr->get_uint8(INPUTMANAGER_TOUCH_KEYBOARD_MODE));

    // 加载触摸响应延迟
    static_config_.touch_response_delay_ms = config_mgr->get_uint8(INPUTMANAGER_TOUCH_RESPONSE_DELAY);
    
    // 加载Serial模式新功能配置
    static_config_.send_only_on_change = config_mgr->get_bool(INPUTMANAGER_SEND_ONLY_ON_CHANGE);
    static_config_.data_aggregation_delay_ms = config_mgr->get_uint8(INPUTMANAGER_DATA_AGGREGATION_DELAY);
    static_config_.extra_send_count = config_mgr->get_uint8(INPUTMANAGER_EXTRA_SEND_COUNT);
    
    // 加载频率限制配置
    static_config_.rate_limit_enabled = config_mgr->get_bool(INPUTMANAGER_RATE_LIMIT_ENABLED);
    static_config_.rate_limit_frequency = config_mgr->get_uint16(INPUTMANAGER_RATE_LIMIT_FREQUENCY);
    
    // 通过调用设置函数来预计算最小间隔时间，确保复用逻辑
    InputManager* instance = InputManager::getInstance();
    if (instance && static_config_.rate_limit_frequency > 0) {
        instance->setRateLimitFrequency(static_config_.rate_limit_frequency);
    }
    
    // 加载Mai2Serial配置
    static_config_.mai2serial_config.baud_rate = config_mgr->get_uint32(INPUTMANAGER_MAI2SERIAL_BAUD_RATE);

    // 加载TouchDevice设备映射数据
    std::string devices_str = config_mgr->get_string(INPUTMANAGER_TOUCH_DEVICES);
    if (!devices_str.empty())
    {
        size_t expected_size = sizeof(TouchDeviceMapping) * static_config_.device_count;
        if (devices_str.size() >= expected_size)
        {
            // 使用循环逐个复制TouchDeviceMapping对象，同时过滤mask为0的设备
            const TouchDeviceMapping *src = reinterpret_cast<const TouchDeviceMapping *>(devices_str.data());
            uint8_t valid_device_count = 0;
            for (size_t i = 0; i < expected_size / sizeof(TouchDeviceMapping); i++)
            {
                // 只复制mask不为0的设备配置
                if (src[i].device_id_mask != 0)
                {
                    static_config_.touch_device_mappings[valid_device_count] = src[i];
                    // 配置加载时，所有设备初始状态设为未连接
                    static_config_.touch_device_mappings[valid_device_count].is_connected = false;
                    valid_device_count++;
                }
            }
            // 更新有效设备数量
            static_config_.device_count = valid_device_count;
        }
    }

    // 加载物理键盘映射数据
    std::string physical_keyboards_str = config_mgr->get_string(INPUTMANAGER_PHYSICAL_KEYBOARDS);
    if (!physical_keyboards_str.empty())
    {
        size_t mapping_count = physical_keyboards_str.size() / sizeof(PhysicalKeyboardMapping);
        static_config_.physical_keyboard_mappings.clear();
        static_config_.physical_keyboard_mappings.resize(mapping_count);
        std::memcpy(static_config_.physical_keyboard_mappings.data(),
                    physical_keyboards_str.data(),
                    physical_keyboards_str.size());
    }

    // 加载区域通道映射配置
    std::string area_mappings_str = config_mgr->get_string(INPUTMANAGER_AREA_CHANNEL_MAPPINGS);
    if (!area_mappings_str.empty() && area_mappings_str.size() >= sizeof(AreaChannelMappingConfig))
    {
        // 使用拷贝赋值替代memcpy，避免非平凡可复制对象警告
        const AreaChannelMappingConfig* source = reinterpret_cast<const AreaChannelMappingConfig*>(area_mappings_str.data());
        static_config_.area_channel_mappings = *source;
    }

    // 加载阶段分配配置
    std::string stage_assignments_str = config_mgr->get_string(INPUTMANAGER_STAGE_ASSIGNMENTS);
    if (!stage_assignments_str.empty())
    {
        size_t assignment_count = stage_assignments_str.size() / sizeof(InputManager_PrivateConfig::StageAssignment);
        static_config_.stage_assignments.clear();
        static_config_.stage_assignments.resize(assignment_count);
        std::memcpy(static_config_.stage_assignments.data(),
                    stage_assignments_str.data(),
                    stage_assignments_str.size());
    }

    // 应用加载的配置到实际硬件设备
    if (instance)
    {
        // 根据设备模式遍历设备映射表，对存在映射关系的设备进行预注册
        // 所有配置的设备在此阶段都标记为未连接状态
        InputManager::log_info("预注册配置中的触摸设备映射，设备数量: " + std::to_string(static_config_.device_count));

        // 应用触摸设备的灵敏度设置
        for (int i = 0; i < static_config_.device_count; i++)
        {
            const TouchDeviceMapping &mapping = static_config_.touch_device_mappings[i];
            InputManager::log_debug("预注册设备ID掩码: 0x" + std::to_string(mapping.device_id_mask) + ", 状态: 未连接");

            // 使用公有方法setSensitivity来应用灵敏度设置
            for (uint8_t channel = 0; channel < mapping.max_channels; channel++)
            {
                uint8_t sensitivity = mapping.sensitivity[channel];
                instance->setSensitivity(mapping.device_id_mask, channel, sensitivity);
            }
        }
    }

    return true;
}

// UI专用接口实现
void InputManager::get_all_device_status(TouchDeviceStatus *data)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (int i = 0; i < config->device_count; i++)
    {
        // 复制设备映射配置
        data[i].touch_device = config->touch_device_mappings[i];

        // 获取当前触摸状态
        data[i].touch_states_32bit = touch_device_states_[i].current_touch_mask;

        // 设置连接状态 - 根据实际连接状态反馈
        data[i].is_connected = config->touch_device_mappings[i].is_connected;

        // 生成设备名称（8位ID掩码转换为HEX）
        uint8_t device_id_mask = data[i].touch_device.device_id_mask;
        char hex_name[12];
        snprintf(hex_name, sizeof(hex_name), "%02X", device_id_mask);

        data[i].device_name = std::string(hex_name);
        data[i].device_type = TouchSensor::identifyICType(device_id_mask & 0x7F);
    }
}

// 通过设备名称获取TouchSensor实例
TouchSensor *InputManager::getTouchSensorByDeviceName(const std::string &device_name)
{
    // 获取所有设备状态
    int device_count = get_device_count();
    if (device_count == 0)
    {
        return nullptr;
    }

    TouchDeviceStatus device_status[device_count];
    get_all_device_status(device_status);

    // 查找匹配设备名称的已连接设备
    for (uint8_t i = 0; i < device_count; i++)
    {
        if (device_status[i].device_name == device_name)
        {
            // 找到匹配的设备，通过设备ID掩码获取TouchSensor实例
            uint8_t device_id_mask = device_status[i].touch_device.device_id_mask;
            return findTouchSensorByIdMask(device_id_mask);
        }
    }

    return nullptr;
}

// [配置读取函数] - 返回当前配置的副本
InputManager_PrivateConfig inputmanager_get_config_copy()
{
    return static_config_;
}

// [配置写入函数] - 将配置写入ConfigManager并保存
bool inputmanager_write_config_to_manager(const InputManager_PrivateConfig &config)
{
    ConfigManager *config_mgr = ConfigManager::getInstance();
    if (!config_mgr)
    {
        return false;
    }

    // 写入工作模式
    config_mgr->set_uint8(INPUTMANAGER_WORK_MODE, static_cast<uint8_t>(config.work_mode));

    // 写入触摸键盘启用状态
    config_mgr->set_bool(INPUTMANAGER_TOUCH_KEYBOARD_ENABLED, config.touch_keyboard_enabled);

    // 写入触摸键盘模式
    config_mgr->set_uint8(INPUTMANAGER_TOUCH_KEYBOARD_MODE, static_cast<uint8_t>(config.touch_keyboard_mode));

    // 写入触摸响应延迟
    config_mgr->set_uint8(INPUTMANAGER_TOUCH_RESPONSE_DELAY, config.touch_response_delay_ms);
    
    // 写入Serial模式新功能配置
    config_mgr->set_bool(INPUTMANAGER_SEND_ONLY_ON_CHANGE, config.send_only_on_change);
    config_mgr->set_uint8(INPUTMANAGER_DATA_AGGREGATION_DELAY, config.data_aggregation_delay_ms);
    config_mgr->set_uint8(INPUTMANAGER_EXTRA_SEND_COUNT, config.extra_send_count);
    
    // 写入频率限制配置
    config_mgr->set_bool(INPUTMANAGER_RATE_LIMIT_ENABLED, config.rate_limit_enabled);
    config_mgr->set_uint16(INPUTMANAGER_RATE_LIMIT_FREQUENCY, config.rate_limit_frequency);
    
    // 保存Mai2Serial配置
    config_mgr->set_uint32(INPUTMANAGER_MAI2SERIAL_BAUD_RATE, config.mai2serial_config.baud_rate);

    // 写入TouchDevice设备映射数据
    if (config.device_count > 0)
    {
        size_t devices_size = sizeof(TouchDeviceMapping) * config.device_count;
        std::string devices_data(devices_size, '\0');
        std::memcpy(&devices_data[0], config.touch_device_mappings, devices_size);
        config_mgr->set_string(INPUTMANAGER_TOUCH_DEVICES, devices_data);
    }

    // 写入物理键盘映射数据
    if (!config.physical_keyboard_mappings.empty())
    {
        size_t keyboards_size = sizeof(PhysicalKeyboardMapping) * config.physical_keyboard_mappings.size();
        std::string keyboards_data(keyboards_size, '\0');
        std::memcpy(&keyboards_data[0],
                    config.physical_keyboard_mappings.data(),
                    keyboards_size);
        config_mgr->set_string(INPUTMANAGER_PHYSICAL_KEYBOARDS, keyboards_data);
    }

    // 写入区域通道映射配置
    {
        size_t area_mappings_size = sizeof(AreaChannelMappingConfig);
        std::string area_mappings_data(area_mappings_size, '\0');
        std::memcpy(&area_mappings_data[0],
                    &config.area_channel_mappings,
                    area_mappings_size);
        config_mgr->set_string(INPUTMANAGER_AREA_CHANNEL_MAPPINGS, area_mappings_data);
    }

    // 写入阶段分配配置
    if (!config.stage_assignments.empty())
    {
        size_t assignments_size = sizeof(InputManager_PrivateConfig::StageAssignment) * config.stage_assignments.size();
        std::string assignments_data(assignments_size, '\0');
        std::memcpy(&assignments_data[0],
                    config.stage_assignments.data(),
                    assignments_size);
        config_mgr->set_string(INPUTMANAGER_STAGE_ASSIGNMENTS, assignments_data);
    }

    // 保存支持saveConfig的触摸设备的自定义配置
    InputManager *instance = InputManager::getInstance();
    if (instance)
    {
        const std::vector<TouchSensor *> &devices = instance->getTouchSensorDevices();
        for (TouchSensor *device : devices)
        {
            USB_LOG_DEBUG("尝试保存TouchSensor设置 %s", device->getDeviceName().c_str());
            if (device)
            {
                std::string device_config = device->saveConfig();
                USB_LOG_DEBUG("设置 %s 配置:%s", device->getDeviceName().c_str(), device_config.c_str());
                if (!device_config.empty())
                {
                    uint8_t device_id_mask = device->getModuleMask();
                    std::string config_key = "TOUCH_DEVICE_CONFIG_" + std::to_string(device_id_mask);
                    config_mgr->set_string_dynamic(config_key, device_config);
                }
            }
        }
    }

    // 更新静态配置
    static_config_ = config;

    return true;
}

// 绑定处理主函数
void InputManager::processBinding()
{
    // 检查是否有取消绑定请求
    if (binding_cancel_pending_)
    {
        binding_cancel_pending_ = false;
        cancelBinding();
        return;
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // 检查超时
    if (current_time - binding_start_time_ > binding_timeout_ms_)
    {
        log_warning("Binding timeout detected, elapsed: " + std::to_string(current_time - binding_start_time_) + "ms");
        if (binding_callback_)
        {
            binding_callback_(false, "Binding timeout");
        }
        cancelBinding();
        return;
    }

    // 根据绑定状态处理
    switch (binding_state_)
    {
    case BindingState::PREPARE:
    case BindingState::WAIT_TOUCH:
    case BindingState::PROCESSING:
        // 根据当前工作模式决定处理函数
        if (getWorkMode() == InputWorkMode::SERIAL_MODE)
        {
            processSerialBinding();
        }
        break;

    default:
        log_warning("Unknown binding state: " + std::to_string(static_cast<int>(binding_state_)));
        break;
    }
}

// 获取Serial绑定区域的辅助函数
static Mai2_TouchArea getSerialBindingArea(uint8_t index)
{
    // Mai2_TouchArea枚举是连续的，从MAI2_AREA_A1=1开始到MAI2_AREA_E8=34
    if (index < 34)
    {
        return static_cast<Mai2_TouchArea>(MAI2_AREA_A1 + index);
    }
    return MAI2_NO_USED;
}

// Serial绑定处理
void InputManager::processSerialBinding()
{
    static uint8_t binding_device_addr = 0;
    static uint8_t binding_channel = 0;
    static uint8_t initial_binding_device_addr = 0;  // 记录初始触摸的设备
    static uint8_t initial_binding_channel = 0;      // 记录初始触摸的通道

    switch (binding_state_)
    {
    case BindingState::PREPARE:
        // 准备绑定当前区域
        if (current_binding_index_ < 34)
        {
            // 获取当前要绑定的区域
            Mai2_TouchArea current_area = getSerialBindingArea(current_binding_index_);

            // 清除手动触发区域，然后设置当前区域为触发
            mai2_serial_->clear_manually_triggle_area();
            mai2_serial_->manually_triggle_area(current_area);

            // 切换到等待触摸状态
            binding_state_ = BindingState::WAIT_TOUCH;
            binding_start_time_ = to_ms_since_boot(get_absolute_time());
        }
        else
        {
            // 所有区域绑定完成
            binding_state_ = BindingState::IDLE;
            binding_active_ = false;
            mai2_serial_->clear_manually_triggle_area();
        }
        break;

    case BindingState::WAIT_TOUCH:
    {
        // 检测触摸输入 - 查找有且只有一个通道触发的情况
        uint8_t touched_device_id = 0;
        uint8_t touched_channel = 0;
        int touch_count = 0;

        for (int dev_idx = 0; dev_idx < config_->device_count; dev_idx++)
        {
            const uint8_t device_id_mask = touch_device_states_[dev_idx].parts.device_mask;
            TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
            if (!mapping)
                continue;

            const uint8_t max_channels = mapping->max_channels;
            const uint32_t channel_mask = touch_device_states_[dev_idx].parts.channel_mask;

            // 计算当前设备的触摸通道数
            for (uint8_t ch = 0; ch < max_channels && ch < 24; ch++)
            {
                const uint32_t ch_mask = (1UL << ch);
                if (channel_mask & ch_mask)
                {
                    touch_count++;
                    touched_device_id = device_id_mask;
                    touched_channel = ch;
                }
            }
        }

        // 检查是否有且只有一个通道触发
        if (touch_count == 1)
        {
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            
            // 检查是否是第一次检测到触摸，或者是否是同一个设备和通道
            if (current_time - binding_start_time_ < 100) // 前100ms内记录初始触摸
            {
                initial_binding_device_addr = touched_device_id;
                initial_binding_channel = touched_channel;
            }
            else if (touched_device_id != initial_binding_device_addr || touched_channel != initial_binding_channel)
            {
                // 触摸的设备或通道发生变化，重置计时器（防止狸猫换太子）
                binding_start_time_ = to_ms_since_boot(get_absolute_time());
                initial_binding_device_addr = touched_device_id;
                initial_binding_channel = touched_channel;
            }
            else if (current_time - binding_start_time_ >= 1000)
            {   // 持续1秒且始终是同一个设备和通道
                // 记录触摸的通道ID并切换到处理状态
                binding_device_addr = touched_device_id;
                binding_channel = touched_channel;
                binding_state_ = BindingState::PROCESSING;
            }
        }
        else
        {
            // 重置计时器（没有触摸或多个触摸）
            binding_start_time_ = to_ms_since_boot(get_absolute_time());
            initial_binding_device_addr = 0;
            initial_binding_channel = 0;
        }
    }
    break;

    case BindingState::PROCESSING:
    {
        // 检查当前区域是否已经有映射，以及该通道是否已经绑定到其他区域
        Mai2_TouchArea current_area = getSerialBindingArea(current_binding_index_);
        bool should_bind = true;

        // 检查当前通道是否已经绑定到任何区域
        Mai2_TouchArea existing_mapping = getSerialMapping(binding_device_addr, binding_channel);
        if (existing_mapping != MAI2_NO_USED)
        {
            // 通道已经绑定到其他区域，跳过绑定
            should_bind = false;
        }

        if (should_bind)
        {
            // 注册该通道到当前区域
            setSerialMapping(binding_device_addr, binding_channel, current_area);
            // 移动到下一个区域
            current_binding_index_++;
        }

        // 检查是否超过枚举最大值
        if (current_binding_index_ >= 34)
        {
            // 绑定结束
            binding_state_ = BindingState::IDLE;
            binding_active_ = false;
            mai2_serial_->clear_manually_triggle_area();
        }
        else
        {
            // 继续下一个区域
            binding_state_ = BindingState::PREPARE;
        }
    }
    break;

    default:
        break;
    }
}

// 备份通道状态
void InputManager::backupChannelStates()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (uint8_t i = 0; i < config->device_count && i < 8; i++)
    {
        for (uint8_t ch = 0; ch < 12; ch++)
        {
            original_channels_backup_[i][ch] = (config->touch_device_mappings[i].enabled_channels_mask & (1 << ch)) ? 1 : 0;
        }
    }
}

// 恢复通道状态 TODO: 存在异常 暂时别用
void InputManager::restoreChannelStates()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (uint8_t i = 0; i < config->device_count && i < 8 && i < touch_sensor_devices_.size(); i++)
    {
        auto *device = touch_sensor_devices_[i];
        uint32_t supported_channels = device->getSupportedChannelCount();

        for (uint8_t ch = 0; ch < supported_channels && ch < 12; ch++)
        {
            bool enabled = (original_channels_backup_[i][ch] != 0);

            if (enabled)
            {
                config->touch_device_mappings[i].enabled_channels_mask |= (1 << ch); // 设置位
            }
            else
            {
                config->touch_device_mappings[i].enabled_channels_mask &= ~(1 << ch); // 清除位
            }

            // 使用新的TouchSensor接口恢复通道使能状态
            device->setChannelEnabled(ch, enabled);
        }
    }
}

// 获取Mai2区域名称
const char *InputManager::getMai2AreaName(Mai2_TouchArea area)
{
    if (area >= MAI2_NO_USED && area <= MAI2_AREA_E8)
    {
        return mai2_area_names[area];
    }
    return "UNKNOWN";
}

// GPIO键盘处理方法实现
void InputManager::updateGPIOStates()
{
    // 高性能GPIO状态读取：使用位运算批量处理

    // 批量读取MCU GPIO状态 - 避免循环，使用硬件寄存器直接读取
    // RP2040 GPIO状态寄存器：SIO_BASE + SIO_GPIO_IN_OFFSET
    mcu_gpio_states_ = sio_hw->gpio_in & 0x3FFFFFFF; // 30位GPIO掩码

    // 读取MCP23S17 GPIO状态
    if (mcp23s17_available_ && mcp23s17_)
    {
        mcp23s17_->read_all_gpio(mcp_gpio_states_);
    }
}

void InputManager::processGPIOKeyboard()
{
    static KeyboardBitmap prev_keyboard_state; // 跟踪上一次的按键状态
    static KeyboardBitmap current_keyboard_state;

    gpio_mcu_changed_ = mcu_gpio_states_ ^ mcu_gpio_previous_states_;
    gpio_mcu_inverted_ = ~mcu_gpio_states_; // 低电平有效
    gpio_mcp_changed_a_ = mcp_gpio_states_.port_a ^ mcp_gpio_previous_states_.port_a;
    gpio_mcp_changed_b_ = mcp_gpio_states_.port_b ^ mcp_gpio_previous_states_.port_b;
    gpio_mcp_inverted_a_ = ~mcp_gpio_states_.port_a; // 低电平有效
    gpio_mcp_inverted_b_ = ~mcp_gpio_states_.port_b;

    // 快速跳过：如果没有GPIO变化则直接返回
    if (!gpio_mcu_changed_ && !gpio_mcp_changed_a_ && !gpio_mcp_changed_b_)
    {
        return;
    }

    current_keyboard_state.clear();

    // 零内存分配：使用类成员缓存变量获取所有指针和计数
    gpio_mappings_cache_ = config_->physical_keyboard_mappings.data();
    gpio_mapping_count_cache_ = config_->physical_keyboard_mappings.size();


    for (size_t i = 0; i < gpio_mapping_count_cache_; ++i)
    {
        gpio_mapping_ptr_cache_ = &gpio_mappings_cache_[i];
        gpio_pin_cache_ = gpio_mapping_ptr_cache_->gpio;
        gpio_pin_num_cache_ = get_gpio_pin_number(gpio_pin_cache_);

        // 使用位运算快速判断GPIO类型和状态
        if ((gpio_pin_cache_ & 0xC0) == 0x00)
        { // MCU GPIO
            gpio_current_state_cache_ = (gpio_mcu_inverted_ >> gpio_pin_num_cache_) & 1;
        }
        else
        { // MCP GPIO
            if (gpio_pin_num_cache_ <= 8)
            { // PORTA
                gpio_bit_pos_cache_ = gpio_pin_num_cache_ - 1;
                gpio_current_state_cache_ = (gpio_mcp_inverted_a_ >> gpio_bit_pos_cache_) & 1;
            }
            else
            { // PORTB
                gpio_bit_pos_cache_ = gpio_pin_num_cache_ - 9;
                gpio_current_state_cache_ = (gpio_mcp_inverted_b_ >> gpio_bit_pos_cache_) & 1;
            }
        }

        // 普通键盘处理：设置当前状态
        if (gpio_current_state_cache_)
        {
            if (gpio_mapping_ptr_cache_->default_key != HID_KeyCode::KEY_NONE)
            {
                current_keyboard_state.setKey(gpio_mapping_ptr_cache_->default_key, true);
            }
            // 内联逻辑键处理避免函数调用，使用类成员缓存变量
            for (size_t j = 0; j < gpio_logical_count_cache_; ++j)
            {
                if (gpio_logical_mappings_cache_[j].gpio_id == gpio_pin_cache_)
                {
                    // 使用类成员缓存指针和位运算展开循环
                    gpio_keys_ptr_cache_ = gpio_logical_mappings_cache_[j].keys;
                    if (gpio_keys_ptr_cache_[0] != HID_KeyCode::KEY_NONE)
                        current_keyboard_state.setKey(gpio_keys_ptr_cache_[0], true);
                    if (gpio_keys_ptr_cache_[1] != HID_KeyCode::KEY_NONE)
                        current_keyboard_state.setKey(gpio_keys_ptr_cache_[1], true);
                    if (gpio_keys_ptr_cache_[2] != HID_KeyCode::KEY_NONE)
                        current_keyboard_state.setKey(gpio_keys_ptr_cache_[2], true);
                    break;
                }
            }
        }
    }

    // 比较当前状态与上一次状态，发送按键变化事件
    if (hid_)
    {
        for (uint8_t i = 0; i < SUPPORTED_KEYS_COUNT; i++)
        {
            HID_KeyCode key = supported_keys[i];
            bool current_pressed = current_keyboard_state.getKey(key);
            bool prev_pressed = prev_keyboard_state.getKey(key);

            if (current_pressed != prev_pressed)
            {
                if (current_pressed)
                {
                    hid_->press_key(key);
                }
                else
                {
                    hid_->release_key(key);
                }
            }
        }
    }

    // 更新上一次状态
    prev_keyboard_state = current_keyboard_state;
    mcu_gpio_previous_states_ = mcu_gpio_states_;
    mcp_gpio_previous_states_ = mcp_gpio_states_;
}

// 获取触摸IC采样速率
uint32_t InputManager::getTouchSampleRate()
{
    // 返回实际测量的采样频率
    return static_cast<uint32_t>(current_sample_rate_);
}

inline void InputManager::incrementSampleCounter()
{
    static uint32_t current_time;
    // 检查是否需要重置计数器（每秒一次）
    current_time = time_us_32();
    sample_counter_++;
    if (current_time - last_reset_time_ >= 1000000)
    {
        current_sample_rate_ = sample_counter_;
        sample_counter_ = 0;
        last_reset_time_ = current_time;
    }
}

void InputManager::resetSampleCounter()
{
    sample_counter_ = 0;
    current_sample_rate_ = 0;
    last_reset_time_ = to_ms_since_boot(get_absolute_time());
}

// 获取HID键盘回报速率
uint32_t InputManager::getHIDReportRate()
{
    if (hid_ && hid_->is_initialized())
    {
        // 返回实际测试的HID报告速率 (Hz)
        return hid_->get_report_rate();
    }
    return 0; // HID未初始化
}

// 获取物理键盘映射列表
const std::vector<PhysicalKeyboardMapping> &InputManager::getPhysicalKeyboards() const
{
    return config_->physical_keyboard_mappings;
}

// 触摸响应延迟管理实现
void InputManager::setTouchResponseDelay(uint8_t delay_ms)
{
    if (delay_ms > 100)
        delay_ms = 100; // 限制最大延迟为100ms
    config_->touch_response_delay_ms = delay_ms;

    // 清空延迟缓冲区
    delay_buffer_head_ = 0;
    delay_buffer_count_ = 0;
}

uint8_t InputManager::getTouchResponseDelay() const
{
    return config_->touch_response_delay_ms;
}

// Serial模式新功能接口实现
void InputManager::setSendOnlyOnChange(bool enabled)
{
    config_->send_only_on_change = enabled;
}

bool InputManager::getSendOnlyOnChange() const
{
    return config_->send_only_on_change;
}

void InputManager::setDataAggregationDelay(uint8_t delay_ms)
{
    if (delay_ms > 100)
        delay_ms = 100; // 限制最大延迟为100ms
    config_->data_aggregation_delay_ms = delay_ms;
}

uint8_t InputManager::getDataAggregationDelay() const
{
    return config_->data_aggregation_delay_ms;
}

void InputManager::setExtraSendCount(uint8_t count)
{
    if (count > 10)
        count = 10; // 限制最大额外发送次数为10
    config_->extra_send_count = count;
}

uint8_t InputManager::getExtraSendCount() const
{
    return config_->extra_send_count;
}

// 频率限制接口实现
void InputManager::setRateLimitEnabled(bool enabled)
{
    config_->rate_limit_enabled = enabled;
}

bool InputManager::getRateLimitEnabled() const
{
    return config_->rate_limit_enabled;
}

void InputManager::setRateLimitFrequency(uint16_t frequency)
{
    if (frequency < 10)
        frequency = 10;   // 限制最小频率为10Hz
    if (frequency > 1000)
        frequency = 1000; // 限制最大频率为1000Hz
    config_->rate_limit_frequency = frequency;
    
    // 预计算最小间隔时间（微秒）
    min_interval_us_ = 1000000U / frequency;
}

uint16_t InputManager::getRateLimitFrequency() const
{
    return config_->rate_limit_frequency;
}

// 获取当前配置副本
InputManager_PrivateConfig InputManager::getConfig() const
{
    return inputmanager_get_config_copy();
}

// 获取Mai2Serial配置
Mai2Serial_Config InputManager::getMai2SerialConfig() const
{
    return config_->mai2serial_config;
}

// 设置Mai2Serial配置
bool InputManager::setMai2SerialConfig(const Mai2Serial_Config& config)
{
    // 更新内部配置
    config_->mai2serial_config = config;
    
    // 应用配置到mai2_serial_实例
    if (mai2_serial_) {
        bool apply_result = mai2_serial_->set_config(config);
        return apply_result;
    }
    
    return true;  // 配置已保存到内部
}

inline void InputManager::storeDelayedSerialState()
{
    static uint32_t current_time_us;
    static uint32_t channel;
    static Mai2Serial_TouchState local_serial_state_;
    local_serial_state_.clear();
    current_time_us = time_us_32();

    // 优化的Serial触摸状态计算
    for (int i = 0; i < config_->device_count; i++)
    {
        // 直接使用0-33索引，避免额外运算
         for (int area_idx = 0; area_idx < 34; area_idx++)
         {
            channel = static_config_.area_channel_mappings.serial_mappings[area_idx].channel;  // 直接索引0-33
            if (channel == 0xFFFFFFFF) continue;
            
            // 优化位运算检查
            if ((touch_device_states_[i].current_touch_mask & (channel | 0xFF000000)) == channel)
            {
                // 直接位操作，避免额外计算
                if (area_idx < 32)
                    local_serial_state_.parts.state1 |= (1UL << area_idx);
                else
                    local_serial_state_.parts.state2 |= (1UL << (area_idx - 32));
            }
        }
    }
    delay_buffer_[delay_buffer_head_].timestamp_us = current_time_us;
    delay_buffer_[delay_buffer_head_].serial_touch_state = local_serial_state_;

    // 优化缓冲区指针更新
    delay_buffer_head_ = (delay_buffer_head_ + 1) % DELAY_BUFFER_SIZE;
    if (delay_buffer_count_ < DELAY_BUFFER_SIZE)
    {
        delay_buffer_count_++;
    }
}

// 静态日志函数实现
void InputManager::log_debug(const std::string &msg)
{
    if (debug_enabled_)
    {
        auto *logger = USB_SerialLogs::get_global_instance();
        if (logger)
        {
            logger->debug(msg, "InputManager");
        }
    }
}

void InputManager::log_info(const std::string &msg)
{
    auto *logger = USB_SerialLogs::get_global_instance();
    if (logger)
    {
        logger->info(msg, "InputManager");
    }
}

void InputManager::log_warning(const std::string &msg)
{
    auto *logger = USB_SerialLogs::get_global_instance();
    if (logger)
    {
        logger->warning(msg, "InputManager");
    }
}

void InputManager::log_error(const std::string &msg)
{
    auto *logger = USB_SerialLogs::get_global_instance();
    if (logger)
    {
        logger->error(msg, "InputManager");
    }
}

// Debug开关控制函数
void InputManager::set_debug_enabled(bool enabled)
{
    debug_enabled_ = enabled;
    if (enabled)
    {
        auto *logger = USB_SerialLogs::get_global_instance();
        if (logger)
        {
            logger->info("InputManager debug logging enabled", "InputManager");
        }
    }
}

bool InputManager::is_debug_enabled()
{
    return debug_enabled_;
}

// 校准管理函数实现
void InputManager::calibrateAllSensors()
{
    // 设置校准请求标志，实际校准将在task0中执行
    calibration_request_pending_ = CalibrationRequestType::REQUEST_NORMAL;
}

void InputManager::calibrateSelectedChannels()
{
    // 设置特殊校准请求标志，实际校准将在task0中执行
    if (!calibration_in_progress_) {
        calibration_request_pending_ = CalibrationRequestType::REQUEST_SUPER;
    }
}

void InputManager::setCalibrationTargetByBitmap(uint32_t channel_bitmap, uint8_t target_sensitivity)
{
    if (calibration_in_progress_) return;
    // 解析bitmap中的设备和通道信息
    uint8_t device_mask = (channel_bitmap >> 24) & 0xFF;
    uint32_t channel_mask = channel_bitmap & 0xFFFFFF;
    
    // 查找对应的传感器
    TouchSensor* sensor = findTouchSensorByIdMask(device_mask);
    if (!sensor) {
        log_warning("setCalibrationTargetByBitmap: Device not found for mask " + std::to_string(device_mask));
        return;
    }
    
    // 为支持按通道校准的设备批量设置校准目标灵敏度
    // 遍历通道掩码，为每个设置的通道设置校准目标
    for (uint8_t channel = 0; channel < 24; channel++) {
        if (channel_mask & (1 << channel)) {
            log_info("Setting calibration target sensitivity " + std::to_string(target_sensitivity) + " for device " + std::to_string(device_mask) + " channel " + std::to_string(channel));
            // 调用传感器的设置校准目标方法
            sensor->setChannelCalibrationTarget(channel, target_sensitivity);
        }
    }
    
    // 注意：此函数仅设置校准目标，不自动发起校准
    // 需要单独调用calibrateSelectedChannels()来发起特殊校准
}

void InputManager::calibrateAllSensorsWithTarget(uint8_t sensitivity_target)
{
    if (!calibration_in_progress_) {
        // 存储灵敏度目标参数
        calibration_sensitivity_target_ = sensitivity_target;
        
        // 设置校准请求标志，实际校准将在task0中执行
        calibration_request_pending_ = CalibrationRequestType::REQUEST_NORMAL;
    }
}

void InputManager::processCalibrationRequest()
{
    if (calibration_request_pending_ == CalibrationRequestType::REQUEST_NORMAL) {
        log_info("Starting normal simultaneous calibration for all sensors");
        
        // 收集所有支持校准的传感器
        std::vector<TouchSensor*> calibration_sensors;
        for (TouchSensor *sensor : touch_sensor_devices_) {
            if (sensor && sensor->supports_calibration_) {
                calibration_sensors.push_back(sensor);
            }
        }
        
        if (calibration_sensors.empty()) {
            log_info("No sensors support calibration");
            calibration_request_pending_ = CalibrationRequestType::IDLE;
            return;
        }
        
        // 同时启动所有传感器的校准
        for (TouchSensor *sensor : calibration_sensors) {
            log_info("Starting calibration for sensor: " + sensor->getDeviceName());
            sensor->calibrateSensor(calibration_sensitivity_target_);
        }
        
        // 校准已发起，清除请求标志并退出，让传感器自行完成校准工作
        log_info("Normal calibration initiated for all sensors, sensors will complete calibration independently");
        calibration_request_pending_ = CalibrationRequestType::IDLE;
        calibration_in_progress_ = true;
    }
    else if (calibration_request_pending_ == CalibrationRequestType::REQUEST_SUPER) {
        log_info("Starting special calibration for selected channels");
        
        // 收集所有支持校准的传感器
        std::vector<TouchSensor*> calibration_sensors;
        for (TouchSensor *sensor : touch_sensor_devices_) {
            if (sensor && sensor->supports_calibration_) {
                calibration_sensors.push_back(sensor);
            }
        }
        
        if (calibration_sensors.empty()) {
            log_info("No sensors support calibration");
            calibration_request_pending_ = CalibrationRequestType::IDLE;
            return;
        }
        
        // 对每个传感器调用单独的启动函数
        for (TouchSensor *sensor : calibration_sensors) {
            log_info("Starting special calibration for sensor: " + sensor->getDeviceName());
            sensor->startCalibration();
        }
        
        // 校准已发起，清除请求标志并退出
        log_info("Special calibration initiated for selected channels");
        calibration_request_pending_ = CalibrationRequestType::IDLE;
        calibration_in_progress_ = true;
    }
}

uint8_t InputManager::getCalibrationProgress()
{
    // 收集所有支持校准的传感器
    std::vector<TouchSensor*> calibration_sensors;
    for (TouchSensor *sensor : touch_sensor_devices_) {
        if (sensor && sensor->supports_calibration_) {
            calibration_sensors.push_back(sensor);
        }
    }
    
    // 如果没有支持校准的传感器，返回255表示完成
    if (calibration_sensors.empty()) {
        return 255;
    }
    
    // 计算所有传感器的平均进度
    uint32_t total_progress = 0;
    uint32_t sensor_count = 0;
    bool any_calibrating = false;
    
    for (TouchSensor *sensor : calibration_sensors) {
        uint8_t progress = sensor->getCalibrationProgress();
        total_progress += progress;
        sensor_count++;
        
        // 如果任何传感器的进度小于255，说明还在校准中
        if (progress < 255) {
            any_calibrating = true;
        }
    }
    
    // 如果没有传感器，返回255表示完成
    if (sensor_count == 0) {
        return 255;
    }
    
    // 如果所有传感器都完成了校准（进度都是255），返回255
    if (!any_calibrating) {
        calibration_in_progress_ = false;
        return 255;
    }
    
    // 计算平均进度
    uint32_t average_progress = total_progress / sensor_count;
    
    // 确保进度不超过254（255保留给完成状态）
    return (average_progress >= 255) ? 254 : static_cast<uint8_t>(average_progress);
}

// 根据设备ID掩码获取设备名称 - UI显示时调用
std::string InputManager::getDeviceNameByMask(uint32_t device_and_channel_mask) const
{
    // 从合并数据中提取设备ID（高8位）
    uint8_t device_id_mask = (device_and_channel_mask >> 24) & 0xFF;
    
    // 遍历所有注册的触摸传感器设备
    for (TouchSensor* sensor : touch_sensor_devices_)
    {
        if (sensor && sensor->getModuleMask() == device_id_mask)
        {
            return sensor->getDeviceName();
        }
    }
    
    // 如果找不到设备，返回默认名称
    return "Unknown Device (0x" + std::to_string(device_id_mask) + ")";
}

// 根据设备ID掩码获取设备类型 - UI显示时调用
TouchSensorType InputManager::getDeviceTypeByMask(uint32_t device_and_channel_mask) const
{
    // 从合并数据中提取设备ID（高8位）
    uint8_t device_id_mask = (device_and_channel_mask >> 24) & 0xFF;
    
    // 遍历所有注册的触摸传感器设备
    for (TouchSensor* sensor : touch_sensor_devices_)
    {
        if (sensor && sensor->getModuleMask() == device_id_mask)
        {
            // 根据设备地址识别设备类型
            return TouchSensor::identifyICType(device_id_mask & 0x7F);
        }
    }
    
    return TouchSensorType::UNKNOWN;
}

// 静态异步采样结果处理函数
void InputManager::async_touchsampleresult(const TouchSampleResult& result) {
    static InputManager* instance = getInstance();
    // 提取设备掩码和I2C总线信息
    static uint8_t device_mask;
    static uint8_t i2c_bus;
    static int8_t device_index;

    device_mask = result.module_mask;
    i2c_bus = TouchSensor::extractI2CBusFromMask(device_mask);
    device_index = -1;

    if (result.timestamp_us == 0) {
        // 约定时间戳为0时 代表采样失败 解锁当前阶段 重新采样
        instance->i2c_sampling_stages_[i2c_bus].stage_locked = false;
        return;
    };

    for (int8_t i = 0; i < instance->touch_sensor_devices_.size(); i++) {
        if (instance->touch_sensor_devices_[i]->getModuleMask() == device_mask) {
            device_index = i;
            break;
        }
    }
    
    if (device_index == -1) return;
    
    // 更新设备状态
    instance->touch_device_states_[device_index].previous_touch_mask = 
        instance->touch_device_states_[device_index].current_touch_mask;
    instance->touch_device_states_[device_index].current_touch_mask = result.touch_mask;
    instance->touch_device_states_[device_index].timestamp_us = result.timestamp_us;
    // 解锁当前阶段并递增到下一个阶段
    instance->i2c_sampling_stages_[i2c_bus].stage_locked = false;
    instance->i2c_sampling_stages_[i2c_bus].next_stage();

    // 增加采样计数器
    instance->incrementSampleCounter();
    // 存储延迟状态
    instance->storeDelayedSerialState();
}

// 设备注册到阶段的接口实现
bool InputManager::registerDeviceToStage(uint8_t stage, uint8_t device_id) {
    if (device_id == 0) {
        return false;
    }
    
    // 从device_id中解析i2c_bus
    uint8_t i2c_bus = TouchSensor::extractI2CBusFromMask(device_id);
    if (i2c_bus >= 2 || stage >= 4) {
        return false;
    }
    
    // 根据device_id查找对应的TouchSensor实例
    TouchSensor* device_instance = nullptr;
    for (TouchSensor* device : touch_sensor_devices_) {
        if (device && device->getModuleMask() == device_id) {
            device_instance = device;
            break;
        }
    }
    
    // 存储实例地址（如果找不到设备则存储nullptr）
    i2c_sampling_stages_[i2c_bus].device_instances[stage] = device_instance;
    return device_instance != nullptr;
}

bool InputManager::unregisterDeviceFromStage(uint8_t i2c_bus, uint8_t stage) {
    if (i2c_bus >= 2 || stage >= 4) {
        return false;
    }
    
    i2c_sampling_stages_[i2c_bus].device_instances[stage] = nullptr;
    return true;
}

uint8_t InputManager::getStageDeviceId(uint8_t i2c_bus, uint8_t stage) const {
    if (i2c_bus >= 2 || stage >= 4) {
        return 0;
    }
    
    TouchSensor* device_instance = i2c_sampling_stages_[i2c_bus].device_instances[stage];
    return device_instance ? device_instance->getModuleMask() : 0;
}

bool InputManager::overrideStageDeviceId(uint8_t stage, uint8_t device_id) {
    // 从device_id中解析i2c_bus（如果device_id为0则跳过解析）
    uint8_t i2c_bus = 0;
    if (device_id != 0) {
        i2c_bus = TouchSensor::extractI2CBusFromMask(device_id);
        if (i2c_bus >= 2) {
            return false;
        }
    }
    
    if (stage >= 4) {
        return false;
    }
    
    // 根据device_id查找对应的TouchSensor实例
    TouchSensor* device_instance = nullptr;
    if (device_id != 0) {
        for (TouchSensor* device : touch_sensor_devices_) {
            if (device && device->getModuleMask() == device_id) {
                device_instance = device;
                break;
            }
        }
    }
    
    // 存储实例地址
    i2c_sampling_stages_[i2c_bus].device_instances[stage] = device_instance;
    return (device_id == 0) || (device_instance != nullptr);
}

// 阶段分配管理接口实现
bool InputManager::setStageAssignment(uint8_t stage, uint8_t device_id) {
    // 从device_id中解析i2c_bus
    uint8_t i2c_bus = TouchSensor::extractI2CBusFromMask(device_id);
    if (i2c_bus >= 2 || stage >= 4) {
        return false;
    }
    
    // 查找现有的分配记录
    auto it = std::find_if(config_->stage_assignments.begin(), config_->stage_assignments.end(),
        [i2c_bus, stage](const InputManager_PrivateConfig::StageAssignment& assignment) {
            return assignment.i2c_bus == i2c_bus && assignment.stage == stage;
        });
    
    if (it != config_->stage_assignments.end()) {
        // 更新现有记录
        it->device_id = device_id;
    } else {
        // 添加新记录
        config_->stage_assignments.emplace_back(i2c_bus, stage, device_id);
    }
    
    // 立即应用到运行时阶段
    return registerDeviceToStage(stage, device_id);
}

bool InputManager::clearStageAssignment(uint8_t i2c_bus, uint8_t stage) {
    if (i2c_bus >= 2 || stage >= 4) {
        return false;
    }
    
    // 从配置中移除
    config_->stage_assignments.erase(
        std::remove_if(config_->stage_assignments.begin(), config_->stage_assignments.end(),
            [i2c_bus, stage](const InputManager_PrivateConfig::StageAssignment& assignment) {
                return assignment.i2c_bus == i2c_bus && assignment.stage == stage;
            }),
        config_->stage_assignments.end());
    
    // 清除运行时阶段
    return unregisterDeviceFromStage(i2c_bus, stage);
}

uint8_t InputManager::getStageAssignment(uint8_t i2c_bus, uint8_t stage) const {
    if (i2c_bus >= 2 || stage >= 4) {
        return 0xFF;
    }
    
    // 首先检查配置中的分配
    auto it = std::find_if(config_->stage_assignments.begin(), config_->stage_assignments.end(),
        [i2c_bus, stage](const InputManager_PrivateConfig::StageAssignment& assignment) {
            return assignment.i2c_bus == i2c_bus && assignment.stage == stage;
        });
    
    if (it != config_->stage_assignments.end()) {
        return it->device_id;
    }
    
    // 如果配置中没有，返回运行时的分配
    return getStageDeviceId(i2c_bus, stage);
}

void InputManager::clearAllStageAssignments() {
    config_->stage_assignments.clear();
    
    // 清除所有运行时阶段
    for (int bus = 0; bus < 2; bus++) {
        for (int stage = 0; stage < 4; stage++) {
            unregisterDeviceFromStage(bus, stage);
        }
    }
}

const std::vector<InputManager_PrivateConfig::StageAssignment>& InputManager::getStageAssignments() const {
    return config_->stage_assignments;
}
