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
    : delay_buffer_head_(0), delay_buffer_count_(0), mcu_gpio_states_(0), mcu_gpio_previous_states_(0), sample_counter_(0), last_reset_time_(0), current_sample_rate_(0), binding_active_(false), binding_callback_(), binding_state_(BindingState::IDLE), current_binding_index_(0), binding_start_time_(0), binding_timeout_ms_(30000) // 30秒超时
      ,
      binding_hardware_ops_pending_(false), binding_cancel_pending_(false), calibration_request_pending_(false), auto_adjust_context_(), mai2_serial_(nullptr), hid_(nullptr), mcp23s17_(nullptr), config_(inputmanager_get_config_holder()), mcp23s17_available_(false), ui_manager_(nullptr), gpio_keyboard_bitmap_(), touch_bitmap_cache_(), mcp_gpio_states_(), mcp_gpio_previous_states_()
{

    // 初始化32位触摸状态数组
    for (int32_t i = 0; i < 8; i++)
    {
        touch_device_states_[i] = TouchDeviceState();
    }
    memset(original_channels_backup_, 0, sizeof(original_channels_backup_));

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

    // 加载配置
    inputmanager_load_config_from_manager();

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
    static Mai2Serial_TouchState delayed_serial_state;

    // 获取延迟后的Serial状态
    if (!getDelayedSerialState(delayed_serial_state))
    {
        return;
    }

    // 直接发送延迟后的Serial状态，无需重新计算
    if (mai2_serial_)
    {
        mai2_serial_->send_touch_data(delayed_serial_state);
    }
}

void InputManager::clearPhysicalKeyboards()
{
    config_->physical_keyboard_mappings.clear();
}

// 逻辑按键映射管理方法
bool InputManager::addLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key)
{
    // 检查是否已存在该GPIO映射
    for (auto &mapping : config_->logical_key_mappings)
    {
        if (mapping.gpio_id == gpio_id)
        {
            // 查找空位添加新按键（最多3个）
            for (int i = 0; i < 3; i++)
            {
                if (mapping.keys[i] == HID_KeyCode::KEY_NONE)
                {
                    mapping.keys[i] = key;
                    mapping.key_count++;
                    return true;
                }
            }
            return false; // 已满
        }
    }

    // 添加新映射
    LogicalKeyMapping new_mapping;
    new_mapping.gpio_id = gpio_id;
    new_mapping.keys[0] = key;
    new_mapping.key_count = 1;
    config_->logical_key_mappings.push_back(new_mapping);

    return true;
}

bool InputManager::removeLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key)
{
    for (auto &mapping : config_->logical_key_mappings)
    {
        if (mapping.gpio_id == gpio_id)
        {
            // 查找并移除指定的按键
            for (int i = 0; i < 3; i++)
            {
                if (mapping.keys[i] == key)
                {
                    mapping.keys[i] = HID_KeyCode::KEY_NONE;
                    mapping.key_count--;

                    // 如果没有按键了，移除整个映射
                    if (mapping.key_count == 0)
                    {
                        auto it = std::find_if(config_->logical_key_mappings.begin(),
                                               config_->logical_key_mappings.end(),
                                               [gpio_id](const LogicalKeyMapping &m)
                                               {
                                                   return m.gpio_id == gpio_id;
                                               });
                        if (it != config_->logical_key_mappings.end())
                        {
                            config_->logical_key_mappings.erase(it);
                        }
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

bool InputManager::clearLogicalKeyMapping(uint8_t gpio_id)
{
    auto it = std::find_if(config_->logical_key_mappings.begin(),
                           config_->logical_key_mappings.end(),
                           [gpio_id](const LogicalKeyMapping &mapping)
                           {
                               return mapping.gpio_id == gpio_id;
                           });

    if (it != config_->logical_key_mappings.end())
    {
        config_->logical_key_mappings.erase(it);
        return true;
    }

    return false;
}

// 触摸键盘映射管理方法
void InputManager::setTouchKeyboardEnabled(bool enabled)
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    config->touch_keyboard_enabled = enabled;
}

inline bool InputManager::getTouchKeyboardEnabled() const
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

    // 处理自动灵敏度调整状态机
    if (auto_adjust_context_.active)
    {
        processAutoAdjustSensitivity();
    }

    // 处理校准请求
    if (calibration_request_pending_)
        processCalibrationRequest();

    // 处理绑定状态
    if (binding_active_)
    {
        // 如果需要执行硬件操作，先执行
        if (binding_hardware_ops_pending_)
        {
            backupChannelStates();
            enableAllChannels();
            binding_hardware_ops_pending_ = false;
        }
        processBinding();
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

    if (config_->work_mode == InputWorkMode::HID_MODE)
    {
        sendHIDTouchData();
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

// 开始HID绑定
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
        // 恢复原始通道状态
        restoreChannelStates();

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

        // 恢复原始通道状态
        restoreChannelStates();

        // 重置绑定状态
        binding_active_ = false;
        binding_callback_ = nullptr;
        binding_state_ = BindingState::IDLE;

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

// 自动调整指定通道的灵敏度
uint8_t InputManager::autoAdjustSensitivity(uint8_t device_id_mask, uint8_t channel)
{
    if (channel > 24)
    {
        return 0; // 无效通道
    }

    // 查找TouchSensor设备
    TouchSensor *device = findTouchSensorByIdMask(device_id_mask);

    if (!device)
    {
        return 0; // 设备未找到
    }

    // 如果状态机正在运行，返回当前灵敏度
    if (auto_adjust_context_.active)
    {
        return getSensitivity(device_id_mask, channel);
    }

    // 初始化状态机
    auto_adjust_context_.device_id_mask = device_id_mask;
    auto_adjust_context_.channel = channel;
    auto_adjust_context_.original_sensitivity = getSensitivity(device_id_mask, channel);
    auto_adjust_context_.current_sensitivity = 1;
    auto_adjust_context_.touch_found_sensitivity = 0;
    auto_adjust_context_.touch_lost_sensitivity = 0;
    auto_adjust_context_.state = AutoAdjustState::FIND_TOUCH_START;
    auto_adjust_context_.state_start_time = to_ms_since_boot(get_absolute_time());
    auto_adjust_context_.active = true;

    return auto_adjust_context_.original_sensitivity;
}

// 状态机处理函数
void InputManager::processAutoAdjustSensitivity()
{
    millis_t current_time = to_ms_since_boot(get_absolute_time());
    millis_t elapsed = current_time - auto_adjust_context_.state_start_time;

    // 查找TouchSensor设备
    TouchSensor *device = nullptr;
    device = findTouchSensorByIdMask(auto_adjust_context_.device_id_mask);

    if (!device)
    {
        // 设备未找到，结束状态机
        auto_adjust_context_.active = false;
        return;
    }

    switch (auto_adjust_context_.state)
    {
    case AutoAdjustState::FIND_TOUCH_START:
        // 设置当前灵敏度并开始等待稳定
        setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel,
                       auto_adjust_context_.current_sensitivity);
        auto_adjust_context_.state = AutoAdjustState::FIND_TOUCH_WAIT;
        auto_adjust_context_.state_start_time = current_time;
        break;

    case AutoAdjustState::FIND_TOUCH_WAIT:
        if (elapsed >= auto_adjust_context_.stabilize_duration)
        {
            // 检查是否检测到触摸（通过 sample() 获取当前通道位图）
            TouchSampleResult result = device->sample();
            uint32_t current_touch_state = result.channel_mask; // 直接使用union字段
            if (current_touch_state & (1 << auto_adjust_context_.channel))
            {
                // 找到触摸，记录灵敏度
                auto_adjust_context_.touch_found_sensitivity = auto_adjust_context_.current_sensitivity;
                auto_adjust_context_.current_sensitivity = auto_adjust_context_.touch_found_sensitivity;
                auto_adjust_context_.state = AutoAdjustState::FIND_RELEASE_START;
                auto_adjust_context_.state_start_time = current_time;
            }
            else
            {
                // 未找到触摸，增加灵敏度
                auto_adjust_context_.current_sensitivity++;
                if (auto_adjust_context_.current_sensitivity > 255)
                {
                    // 未找到触摸，恢复原始灵敏度并结束
                    setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel,
                                   auto_adjust_context_.original_sensitivity);
                    auto_adjust_context_.active = false;
                    return;
                }
                auto_adjust_context_.state = AutoAdjustState::FIND_TOUCH_START;
                auto_adjust_context_.state_start_time = current_time;
            }
        }
        break;

    case AutoAdjustState::FIND_RELEASE_START:
        // 设置当前灵敏度并开始等待稳定
        setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel,
                       auto_adjust_context_.current_sensitivity);
        auto_adjust_context_.state = AutoAdjustState::FIND_RELEASE_WAIT;
        auto_adjust_context_.state_start_time = current_time;
        break;

    case AutoAdjustState::FIND_RELEASE_WAIT:
        if (elapsed >= auto_adjust_context_.stabilize_duration)
        {
            // 检查是否失去触摸（通过 sample() 获取当前通道位图）
            TouchSampleResult result = device->sample();
            uint32_t current_touch_state = result.channel_mask; // 直接使用union字段
            if (!(current_touch_state & (1 << auto_adjust_context_.channel)))
            {
                // 失去触摸，记录灵敏度
                auto_adjust_context_.touch_lost_sensitivity = auto_adjust_context_.current_sensitivity;
                auto_adjust_context_.state = AutoAdjustState::VERIFY_THRESHOLD;
                auto_adjust_context_.state_start_time = current_time;
            }
            else
            {
                // 仍有触摸，减小灵敏度
                auto_adjust_context_.current_sensitivity--;
                if (auto_adjust_context_.current_sensitivity < 1)
                {
                    // 无法找到释放点，使用中间值
                    uint8_t middle_sensitivity = (auto_adjust_context_.touch_found_sensitivity + 1) / 2;
                    setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel, middle_sensitivity);
                    auto_adjust_context_.active = false;
                    return;
                }
                auto_adjust_context_.state = AutoAdjustState::FIND_RELEASE_START;
                auto_adjust_context_.state_start_time = current_time;
            }
        }
        break;

    case AutoAdjustState::VERIFY_THRESHOLD:
        // 验证临界点
        {
            uint8_t critical_sensitivity = auto_adjust_context_.touch_lost_sensitivity + 1;
            setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel, critical_sensitivity);
            auto_adjust_context_.state = AutoAdjustState::COMPLETE;
            auto_adjust_context_.state_start_time = current_time;
        }
        break;

    case AutoAdjustState::COMPLETE:
        if (elapsed >= auto_adjust_context_.stabilize_duration)
        {
            // 最终验证（通过 sample() 获取当前通道位图）
            TouchSampleResult result = device->sample();
            uint32_t current_touch_state = result.channel_mask;
            uint8_t final_sensitivity;

            if (current_touch_state & (1 << auto_adjust_context_.channel))
            {
                // 找到了临界点
                final_sensitivity = auto_adjust_context_.touch_lost_sensitivity + 1;
            }
            else
            {
                // 需要使用中间值
                final_sensitivity = (auto_adjust_context_.touch_lost_sensitivity + auto_adjust_context_.touch_found_sensitivity) / 2;
            }

            setSensitivity(auto_adjust_context_.device_id_mask, auto_adjust_context_.channel, final_sensitivity);
            auto_adjust_context_.active = false;
        }
        break;

    default:
        auto_adjust_context_.active = false;
        break;
    }
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
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels && area >= 1 && area <= 34)
    {
        // 反向映射：区域 -> 通道，使用32位物理通道地址
        mapping->serial_mappings[area].channel = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
        updateChannelStatesAfterBinding();
    }
}

void InputManager::setHIDMapping(uint8_t device_id_mask, uint8_t channel, float x, float y)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels)
    {
        // 反向映射：找到空闲的HID区域或复用现有区域
        uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);

        // 查找是否已有该通道的映射
        int target_index = -1;
        for (int i = 0; i < 10; i++)
        {
            if (mapping->hid_mappings[i].channel == physical_address)
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
                if (mapping->hid_mappings[i].channel == 0xFFFFFFFF)
                {
                    target_index = i;
                    break;
                }
            }
        }

        // 设置HID映射
        if (target_index != -1)
        {
            mapping->hid_mappings[target_index].channel = physical_address;
            mapping->hid_mappings[target_index].coordinates = {x, y};
        }

        updateChannelStatesAfterBinding();
    }
}

Mai2_TouchArea InputManager::getSerialMapping(uint8_t device_id_mask, uint8_t channel)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (!mapping || channel >= mapping->max_channels)
        return MAI2_NO_USED;

    // 反向查找：通过通道找到对应的Mai2区域
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (uint8_t area = 1; area <= 34; area++)
    {
        if (mapping->serial_mappings[area].channel == physical_address)
        {
            return (Mai2_TouchArea)area;
        }
    }
    return MAI2_NO_USED;
}

TouchAxis InputManager::getHIDMapping(uint8_t device_id_mask, uint8_t channel)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (!mapping || channel >= mapping->max_channels)
        return TouchAxis{0.0f, 0.0f};

    // 反向查找：通过通道找到对应的HID坐标
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (int i = 0; i < 10; i++)
    {
        if (mapping->hid_mappings[i].channel == physical_address)
        {
            return mapping->hid_mappings[i].coordinates;
        }
    }
    return TouchAxis{0.0f, 0.0f};
}

void InputManager::setTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel, HID_KeyCode key)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (mapping && channel < mapping->max_channels)
    {
        // 反向映射：按键 -> 通道，使用32位物理通道地址
        mapping->keyboard_mappings[key].channel = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    }
}

HID_KeyCode InputManager::getTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel)
{
    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
    if (!mapping || channel >= mapping->max_channels)
        return HID_KeyCode::KEY_NONE;

    // 反向查找：通过通道找到对应的按键
    uint32_t physical_address = encodePhysicalChannelAddress(device_id_mask, 1 << channel);
    for (const auto &kb_pair : mapping->keyboard_mappings)
    {
        if (kb_pair.second.channel == physical_address)
        {
            return kb_pair.first;
        }
    }
    return HID_KeyCode::KEY_NONE;
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

    // 遍历所有设备映射，找到绑定该区域的通道
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    for (auto &mapping : config->touch_device_mappings)
    {
        const auto &area_mapping = mapping.serial_mappings[area];
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
        const auto &hid_mapping = mapping.hid_mappings[hid_area_index];
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
        auto it = mapping.keyboard_mappings.find(key);
        if (it != mapping.keyboard_mappings.end() && it->second.channel != 0xFFFFFFFF)
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

    for (int i = 0; i < INPUTMANAGER_MAX_TOUCH_DEVICES && i < touch_sensor_devices_.size(); i++)
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
                    if (mapping.serial_mappings[area_idx].channel == physical_address)
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
                for (const auto &hid_mapping : mapping.hid_mappings)
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

void InputManager::updateChannelStatesAfterBinding()
{
    InputManager_PrivateConfig *config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();

    for (int i = 0; i < INPUTMANAGER_MAX_TOUCH_DEVICES; i++)
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
                    if (mapping.serial_mappings[area_idx].channel == physical_address)
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
                for (const auto &hid_mapping : mapping.hid_mappings)
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

    // 重新应用通道映射
    enableMappedChannels();
}

// 更新触摸状态
inline void InputManager::updateTouchStates()
{
    // 使用TouchSensor统一采样接口更新触摸状态（微秒时间戳）
    static TouchSensor *device;
    static TouchSampleResult result;
    for (int i = 0; i < (int)touch_sensor_devices_.size(); i++)
    {
        device = touch_sensor_devices_[i];

        // 保存之前的触摸状态
        touch_device_states_[i].previous_touch_mask = touch_device_states_[i].current_touch_mask;
        result = device->sample();
        touch_device_states_[i].current_touch_mask = result.touch_mask;
        touch_device_states_[i].parts.device_mask = result.module_mask;
        touch_device_states_[i].timestamp_us = result.timestamp_us; // 直接使用us级时间戳
    }
    // 增加采样计数器
    incrementSampleCounter();
    // 序列化发送前的延迟缓冲不变
    storeDelayedSerialState();
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
            for (const auto &hid_mapping : mapping->hid_mappings)
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

// 16位地址兼容性方法已移除，统一使用32位地址处理方法

// 32位地址处理辅助方法
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
    default_map[INPUTMANAGER_TOUCH_KEYBOARD_ENABLED] = ConfigValue(true);     // 默认启用触摸键盘
    default_map[INPUTMANAGER_TOUCH_RESPONSE_DELAY] = ConfigValue((uint8_t)0); // 默认触摸响应延迟

    // 二进制数据配置使用空字符串作为默认值
    // GTX312L配置项已移除 - 统一使用TouchDeviceMapping
    default_map[INPUTMANAGER_PHYSICAL_KEYBOARDS] = ConfigValue(std::string(""));
    default_map[INPUTMANAGER_LOGICAL_MAPPINGS] = ConfigValue(std::string(""));
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

    // 加载逻辑按键映射数据
    std::string logical_mappings_str = config_mgr->get_string(INPUTMANAGER_LOGICAL_MAPPINGS);
    if (!logical_mappings_str.empty())
    {
        size_t mapping_count = logical_mappings_str.size() / sizeof(LogicalKeyMapping);
        static_config_.logical_key_mappings.clear();
        static_config_.logical_key_mappings.resize(mapping_count);
        std::memcpy(static_config_.logical_key_mappings.data(),
                    logical_mappings_str.data(),
                    logical_mappings_str.size());
    }

    // 应用加载的配置到实际硬件设备
    InputManager *instance = InputManager::getInstance();
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

    // 写入逻辑按键映射数据
    if (!config.logical_key_mappings.empty())
    {
        size_t mappings_size = sizeof(LogicalKeyMapping) * config.logical_key_mappings.size();
        std::string mappings_data(mappings_size, '\0');
        std::memcpy(&mappings_data[0],
                    config.logical_key_mappings.data(),
                    mappings_size);
        config_mgr->set_string(INPUTMANAGER_LOGICAL_MAPPINGS, mappings_data);
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
            log_debug("Processing Serial binding, state: " + std::to_string(static_cast<int>(binding_state_)));
            if (binding_callback_ == nullptr)
            {
                // 自动绑定模式
                processAutoSerialBinding();
            }
            else
            {
                // 交互绑定模式
                processSerialBinding();
            }
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
            if (current_time - binding_start_time_ >= 1000)
            {   // 持续1秒
                // 记录触摸的通道ID并切换到处理状态
                // 记录触摸的设备和通道信息
                binding_device_addr = touched_device_id;
                binding_channel = touched_channel;
                binding_state_ = BindingState::PROCESSING;
            }
        }
        else
        {
            // 重置计时器（没有触摸或多个触摸）
            binding_start_time_ = to_ms_since_boot(get_absolute_time());
        }
    }
    break;

    case BindingState::PROCESSING:
    {
        // 检查已绑定的区域中是否存在该通道ID
        Mai2_TouchArea current_area = getSerialBindingArea(current_binding_index_);
        bool channel_already_bound = false;

        // 检查所有已绑定的区域
        for (uint8_t i = 0; i < current_binding_index_; i++)
        {
            Mai2_TouchArea check_area = getSerialBindingArea(i);
            Mai2_TouchArea existing_mapping = getSerialMapping(binding_device_addr, binding_channel);
            if (existing_mapping == check_area)
            {
                channel_already_bound = true;
                break;
            }
        }

        if (!channel_already_bound)
        {
            // 注册该通道到当前区域
            setSerialMapping(binding_device_addr, binding_channel, current_area);
        }

        // 移动到下一个区域
        current_binding_index_++;

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

// 自动Serial绑定处理
void InputManager::processAutoSerialBinding()
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    switch (binding_state_)
    {
    case BindingState::PREPARE:
        // 开始扫描所有触摸输入
        binding_state_ = BindingState::WAIT_TOUCH;

        if (binding_callback_)
        {
            binding_callback_(true, "开始扫描触摸输入，请按顺序触摸所有区域");
        }

        break;

    case BindingState::WAIT_TOUCH:
        // 检测触摸输入并自动分配
        {
            bool touch_detected = false;

            // 检查所有设备的触摸状态 - 使用32位TouchSensor接口
            for (int dev_idx = 0; dev_idx < config_->device_count; dev_idx++)
            {
                const uint32_t current_state = touch_device_states_[dev_idx].current_touch_mask;
                const uint32_t previous_state = touch_device_states_[dev_idx].previous_touch_mask;
                const uint32_t new_touches = current_state & ~previous_state;

                if (new_touches != 0)
                {
                    const uint32_t device_id_mask = touch_device_states_[dev_idx].current_touch_mask;
                    TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
                    if (!mapping)
                        continue;

                    const uint8_t max_channels = mapping->max_channels;

                    // 找到第一个新触摸的通道
                    for (uint8_t ch = 0; ch < max_channels && ch < 32; ch++)
                    {
                        const uint32_t ch_mask = (1UL << ch);
                        if (new_touches & ch_mask)
                        {
                            // 检查这个通道是否已经被绑定
                            Mai2_TouchArea existing_area = getSerialMapping(device_id_mask, ch);
                            if (existing_area == MAI2_NO_USED && current_binding_index_ < 34)
                            {
                                // 自动分配下一个可用的Mai2区域
                                Mai2_TouchArea target_area = getSerialBindingArea(current_binding_index_);
                                setSerialMapping(device_id_mask, ch, target_area);

                                current_binding_index_++;
                                touch_detected = true;

                                if (binding_callback_)
                                {
                                    char message[128];
                                    snprintf(message, sizeof(message), "绑定成功：%s (%d/34)",
                                             getMai2AreaName(target_area), current_binding_index_);
                                    binding_callback_(true, message);
                                }

                                // 如果所有区域都已绑定，进入处理状态
                                if (current_binding_index_ >= 34)
                                {
                                    binding_state_ = BindingState::PROCESSING;

                                    if (binding_callback_)
                                    {
                                        binding_callback_(true, "自动绑定完成，请确认保存");
                                    }
                                }

                                break;
                            }
                        }
                    }

                    if (touch_detected)
                        break;
                }
            }
        }
        break;

    case BindingState::PROCESSING:
        // 处理绑定完成逻辑
        if (current_time - binding_start_time_ > binding_timeout_ms_)
        {
            // 超时完成但不自动保存

            if (binding_callback_)
            {
                char message[64];
                snprintf(message, sizeof(message), "自动绑定完成，共绑定%d个区域", current_binding_index_);
                binding_callback_(true, message);
            }

            // 恢复原始通道状态
            restoreChannelStates();

            // 重置绑定状态
            binding_active_ = false;
            binding_callback_ = nullptr;
            binding_state_ = BindingState::IDLE;
        }
        // 用户可以通过UI确认或取消
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

// 恢复通道状态
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
    gpio_logical_mappings_cache_ = config_->logical_key_mappings.data();
    gpio_logical_count_cache_ = config_->logical_key_mappings.size();

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

// 清除所有逻辑按键映射
void InputManager::clearAllLogicalKeyMappings()
{
    config_->logical_key_mappings.clear();
    inputmanager_write_config_to_manager(*config_);
}

// 获取逻辑按键映射列表
const std::vector<LogicalKeyMapping> &InputManager::getLogicalKeyMappings() const
{
    return config_->logical_key_mappings;
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

    // 持久化保存配置
    inputmanager_write_config_to_manager(*config_);
}

uint8_t InputManager::getTouchResponseDelay() const
{
    return config_->touch_response_delay_ms;
}

// 获取当前配置副本
InputManager_PrivateConfig InputManager::getConfig() const
{
    return inputmanager_get_config_copy();
}

inline void InputManager::storeDelayedSerialState()
{
    // 获取当前时间戳（微秒）
    static uint32_t current_time_us;
    static Mai2Serial_TouchState serial_state;
    static Mai2_TouchArea area;
    static uint8_t bit_index;
    static uint32_t bit_mask;
    // 计算当前Serial状态
    current_time_us = time_us_32();
    serial_state.parts.state1 = 0;
    serial_state.parts.state2 = 0;

    // 计算Serial触摸状态
    for (int i = 0; i < config_->device_count; i++)
    {
        const uint32_t device_id_mask = touch_device_states_[i].current_touch_mask;
        TouchDeviceMapping *mapping = findTouchDeviceMapping(device_id_mask);
        if (!mapping)
            continue;

        // 处理所有支持的通道
        for (uint8_t ch = 0; ch < mapping->max_channels; ch++)
        {
            if (!(touch_device_states_[i].current_touch_mask & (1UL << ch)))
                continue;
            // 处理Serial触摸映射 - 使用反向映射查找
            area = MAI2_NO_USED;
            // 生成当前通道的32位物理地址
            uint32_t physical_address = encodePhysicalChannelAddress(mapping->device_id_mask, 1 << ch);
            // 查找该通道对应的区域
            for (int area_idx = 1; area_idx <= 34; area_idx++)
            {
                if (mapping->serial_mappings[area_idx].channel == physical_address)
                {
                    area = static_cast<Mai2_TouchArea>(area_idx);
                    break;
                }
            }
            if (area >= 1 && area <= 34)
            {
                bit_index = area - 1;
                bit_mask = (1UL << (bit_index & 31));
                if (bit_index < 32)
                    serial_state.parts.state1 |= bit_mask;
                else
                    serial_state.parts.state2 |= bit_mask;
            }
        }
    }

    // 存储计算好的Serial状态到缓冲区
    delay_buffer_[delay_buffer_head_].timestamp_us = current_time_us;
    delay_buffer_[delay_buffer_head_].serial_touch_state = serial_state;

    // 更新缓冲区指针
    delay_buffer_head_ = (delay_buffer_head_ + 1) % DELAY_BUFFER_SIZE;
    if (delay_buffer_count_ < DELAY_BUFFER_SIZE)
    {
        delay_buffer_count_++;
    }
}

bool InputManager::getDelayedSerialState(Mai2Serial_TouchState &delayed_state)
{
    static uint32_t target_time;
    static uint8_t buffer_idx;

    // 零延迟路径 - 直接使用当前状态，避免所有缓冲区操作
    if (config_->touch_response_delay_ms == 0)
    {
        delayed_state = {0};

        // 批量处理所有设备，减少循环开销
        for (uint8_t i = 0; i < config_->device_count; ++i)
        {
            uint32_t active_channels = touch_device_states_[i].current_touch_mask &
                                       config_->touch_device_mappings[i].enabled_channels_mask;

            while (active_channels)
            {
                uint8_t channel = __builtin_ctz(active_channels);
                active_channels &= ~(1U << channel);

                // 直接查找映射区域
                uint32_t physical_addr = encodePhysicalChannelAddress(
                    config_->touch_device_mappings[i].device_id_mask,
                    1U << channel);

                for (uint8_t area = 1; area <= 34; ++area)
                {
                    if (config_->touch_device_mappings[i].serial_mappings[area].channel == physical_addr)
                    {
                        (area <= 32) ? delayed_state.parts.state1 |= (1UL << (area - 1)) : delayed_state.parts.state2 |= (1UL << (area - 33));
                        break;
                    }
                }
            }
        }
        return true;
    }

    // 延迟路径 - 从缓冲区获取
    target_time = time_us_32() - (config_->touch_response_delay_ms * 1000U);
    buffer_idx = delay_buffer_head_;
    // 从最新数据向前搜索，找到目标时间点0
    for (uint8_t i = 0; i < delay_buffer_count_; ++i)
    {
        buffer_idx = (buffer_idx - 1) & (DELAY_BUFFER_SIZE - 1);
        if (delay_buffer_[buffer_idx].timestamp_us <= target_time)
        {
            delayed_state = delay_buffer_[buffer_idx].serial_touch_state;
            return true;
        }
    }

    delayed_state = {0};
    return false;
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
    calibration_request_pending_ = true;
}

void InputManager::processCalibrationRequest()
{
    static bool calibration_started = false;
    
    // 如果是新的校准请求，同时启动所有传感器的校准
    if (!calibration_started) {
        log_info("Starting calibration for all sensors simultaneously");
        
        // 同时启动所有支持校准的传感器
        int calibration_sensor_count = 0;
        for (TouchSensor *sensor : touch_sensor_devices_) {
            if (sensor && sensor->supports_calibration_) {
                log_info("Starting calibration for sensor: " + sensor->getDeviceName());
                sensor->calibrateSensor();
                calibration_sensor_count++;
            }
        }
        
        if (calibration_sensor_count == 0) {
            log_info("No sensors support calibration");
            calibration_request_pending_ = false;
            return;
        }
        
        calibration_started = true;
        log_info("Started calibration for " + std::to_string(calibration_sensor_count) + " sensors");
        return;
    }
    
    // 检查所有传感器的校准进度
    bool all_completed = true;
    for (TouchSensor *sensor : touch_sensor_devices_) {
        if (sensor && sensor->supports_calibration_) {
            uint8_t progress = sensor->getCalibrationProgress();
            if (progress < 255) {
                all_completed = false;
                break;
            }
        }
    }
    
    // 如果所有传感器校准完成，重置状态
    if (all_completed) {
        calibration_request_pending_ = false;
        calibration_started = false;
        log_info("All sensor calibration completed");
    }
}

uint8_t InputManager::getCalibrationProgress()
{
    uint32_t total_progress = 0;
    uint32_t calibration_sensor_count = 0;

    // 遍历所有注册的触摸传感器设备
    for (TouchSensor *sensor : touch_sensor_devices_)
    {
        if (sensor && sensor->supports_calibration_)
        {
            // 获取每个传感器的校准进度
            uint8_t sensor_progress = sensor->getCalibrationProgress();
            total_progress += sensor_progress;
            calibration_sensor_count++;
        }
    }

    // 如果没有支持校准的传感器，返回255表示完成
    if (calibration_sensor_count == 0)
    {
        return 255;
    }
    
    // 计算平均进度，确保结果不超过255
    uint32_t average_progress = total_progress / calibration_sensor_count;
    return (average_progress > 255) ? 255 : (uint8_t)average_progress;
}

// 收集所有支持校准设备的异常通道 - 使用静态预分配数组
InputManager::AbnormalChannelResult InputManager::collectAbnormalChannels()
{
    AbnormalChannelResult result;
    
    log_info("Collecting abnormal channels from all calibration-supported sensors");
    
    // 遍历所有注册的触摸传感器设备
    for (TouchSensor* sensor : touch_sensor_devices_)
    {
        if (sensor && sensor->supports_calibration_ && result.device_count < MAX_TOUCH_DEVICE)
        {
            // 获取异常通道bitmap
            uint32_t abnormal_mask = sensor->getAbnormalChannelMask();
            
            if (abnormal_mask != 0)
            {
                uint8_t device_id_mask = sensor->getModuleMask();
                
                // 点亮异常IC的LED
                sensor->setLEDEnabled(true);
                
                // 构造合并数据：高8位设备ID，低24位异常通道掩码
                uint32_t combined_mask = ((uint32_t)device_id_mask << 24) | (abnormal_mask & 0xFFFFFF);
                result.devices[result.device_count] = AbnormalChannelInfo(combined_mask);
                result.device_count++;
                
                log_info("Found abnormal channels - Device ID: 0x" + 
                        std::to_string(device_id_mask) + ", Channel mask: 0x" + 
                        std::to_string(abnormal_mask & 0xFFFFFF) + ", Combined: 0x" + 
                        std::to_string(combined_mask) + ", LED enabled");
            }
            else
            {
                // 没有异常通道，关闭LED
                sensor->setLEDEnabled(false);
            }
        }
    }
    
    log_info("Collected " + std::to_string(result.device_count) + " devices with abnormal channels");
    
    return result;
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
