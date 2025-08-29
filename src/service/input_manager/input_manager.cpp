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
InputManager* InputManager::instance_ = nullptr;

// 单例模式实现
InputManager* InputManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new InputManager();
    }
    return instance_;
}

// 私有构造函数
InputManager::InputManager() 
    : sample_counter_(0)
    , last_reset_time_(0)
    , current_sample_rate_(0)
    , binding_active_(false)
    , binding_state_(BindingState::IDLE)
    , current_binding_index_(0)
    , binding_start_time_(0)
    , binding_timeout_ms_(30000)  // 30秒超时
    , hid_binding_device_addr_(0)
    , hid_binding_channel_(0)
    , hid_binding_x_(0.0f)
    , hid_binding_y_(0.0f)
    , mai2_serial_(nullptr)
    , hid_(nullptr)
    , mcp23s17_(nullptr)
    , config_(inputmanager_get_config_holder())
    , mcp23s17_available_(false)
    , gpio_keyboard_bitmap_()
    , touch_bitmap_cache_()
    , mcu_gpio_states_(0)
    , mcu_gpio_previous_states_(0)
    {
    
    // 初始化触摸状态数组
    for (int i = 0; i < 8; i++) {
        current_touch_states_[i].physical_addr = 0;
        current_touch_states_[i].timestamp = 0;
        previous_touch_states_[i].physical_addr = 0;
        previous_touch_states_[i].timestamp = 0;
    }
    memset(original_channels_backup_, 0, sizeof(original_channels_backup_));
    
    // 初始化MCP GPIO状态
    mcp_gpio_states_.port_a = 0;
    mcp_gpio_states_.port_b = 0;
    mcp_gpio_states_.timestamp = 0;
    mcp_gpio_previous_states_ = mcp_gpio_states_;
}

// 析构函数
InputManager::~InputManager() {
    deinit();
}

// 初始化
bool InputManager::init(const InitConfig& config) {
    // 设置外部实例引用
    mai2_serial_ = config.mai2_serial;
    hid_ = config.hid;
    mcp23s17_ = config.mcp23s17;
    mcp23s17_available_ = (mcp23s17_ != nullptr);
    ui_manager_ = config.ui_manager;
    // 初始化触摸状态数组
    for (int i = 0; i < 8; i++) {
        current_touch_states_[i] = {};
        previous_touch_states_[i] = {};
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
void InputManager::deinit() {
    // 保存配置
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    inputmanager_write_config_to_manager(*config);
    
    // 取消所有活动操作
    cancelBinding();
    
    // 清空设备列表
    gtx312l_devices_.clear();
    
    // 重置配置中的设备计数
    config->device_count = 0;
}

// 注册GTX312L设备
// TouchSensor统一接口实现
bool InputManager::registerTouchSensor(TouchSensor* device) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    
    if (!device || config->device_count >= 8) {
        return false;
    }
    
    // 获取设备ID掩码
    uint32_t module_id_mask = device->getModuleIdMask();
    uint16_t device_addr = (module_id_mask >> 16) & 0xFFFF;
    
    // 检查是否已经注册
    for (const auto& registered_device : touch_sensor_devices_) {
        if (registered_device->getModuleIdMask() == module_id_mask) {
            return false; // 已经注册
        }
    }
    
    // 添加到TouchSensor设备列表
    touch_sensor_devices_.push_back(device);
    
    // 初始化设备映射
    config->device_mappings[config->device_count].device_addr = device_addr;
    // 默认灵敏度已在构造函数中设置
    
    config->device_count++;
    return true;
}

void InputManager::unregisterTouchSensor(TouchSensor* device) {
    if (!device) return;
    
    uint32_t module_id_mask = device->getModuleIdMask();
    uint16_t device_addr = (module_id_mask >> 16) & 0xFFFF;
    
    // 从TouchSensor设备列表中移除
    auto it = std::find(touch_sensor_devices_.begin(), touch_sensor_devices_.end(), device);
    if (it != touch_sensor_devices_.end()) {
        touch_sensor_devices_.erase(it);
        
        // 重新整理设备映射数组
        int removed_index = findDeviceIndex(device_addr);
        if (removed_index >= 0) {
            for (int i = removed_index; i < config_->device_count - 1; i++) {
                config_->device_mappings[i] = config_->device_mappings[i + 1];
            }
            config_->device_count--;
        }
    }
}

// 保持向后兼容的GTX312L接口
bool InputManager::registerGTX312L(GTX312L* device) {
    // 直接调用TouchSensor接口
    bool result = registerTouchSensor(static_cast<TouchSensor*>(device));
    
    if (result) {
        // 同时添加到GTX312L设备列表以保持兼容性
        gtx312l_devices_.push_back(device);
    }
    
    return result;
}

// 注销GTX312L设备 - 保持向后兼容
void InputManager::unregisterGTX312L(GTX312L* device) {
    if (!device) return;
    
    // 调用TouchSensor接口
    unregisterTouchSensor(static_cast<TouchSensor*>(device));
    
    // 从GTX312L设备列表中移除以保持兼容性
    auto it = std::find(gtx312l_devices_.begin(), gtx312l_devices_.end(), device);
    if (it != gtx312l_devices_.end()) {
        gtx312l_devices_.erase(it);
    }
}

// 物理键盘映射管理方法
bool InputManager::addPhysicalKeyboard(MCU_GPIO gpio, HID_KeyCode default_key) {
    // 添加新映射
    PhysicalKeyboardMapping new_mapping(gpio, default_key);
    config_->physical_keyboard_mappings.push_back(new_mapping);
    return true;
}

bool InputManager::addPhysicalKeyboard(MCP_GPIO gpio, HID_KeyCode default_key) {
    // 添加新映射
    PhysicalKeyboardMapping new_mapping(gpio, default_key);
    config_->physical_keyboard_mappings.push_back(new_mapping);
    return true;
}

bool InputManager::removePhysicalKeyboard(uint8_t gpio_pin) {
    auto it = std::find_if(config_->physical_keyboard_mappings.begin(), 
                          config_->physical_keyboard_mappings.end(),
                          [gpio_pin](const PhysicalKeyboardMapping& mapping) {
                              return mapping.gpio == gpio_pin;
                          });
    
    if (it != config_->physical_keyboard_mappings.end()) {
        config_->physical_keyboard_mappings.erase(it);
        return true;
    }
    
    return false;
}

void InputManager::clearPhysicalKeyboards() {
    config_->physical_keyboard_mappings.clear();
}

// 逻辑按键映射管理方法
bool InputManager::addLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key) {
    // 检查是否已存在该GPIO映射
    for (auto& mapping : config_->logical_key_mappings) {
        if (mapping.gpio_id == gpio_id) {
            // 查找空位添加新按键（最多3个）
            for (int i = 0; i < 3; i++) {
                if (mapping.keys[i] == HID_KeyCode::KEY_NONE) {
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

bool InputManager::removeLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key) {
    for (auto& mapping : config_->logical_key_mappings) {
        if (mapping.gpio_id == gpio_id) {
            // 查找并移除指定的按键
            for (int i = 0; i < 3; i++) {
                if (mapping.keys[i] == key) {
                    mapping.keys[i] = HID_KeyCode::KEY_NONE;
                    mapping.key_count--;
                    
                    // 如果没有按键了，移除整个映射
                    if (mapping.key_count == 0) {
                        auto it = std::find_if(config_->logical_key_mappings.begin(), 
                                              config_->logical_key_mappings.end(),
                                              [gpio_id](const LogicalKeyMapping& m) {
                                                  return m.gpio_id == gpio_id;
                                              });
                        if (it != config_->logical_key_mappings.end()) {
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

bool InputManager::clearLogicalKeyMapping(uint8_t gpio_id) {
    auto it = std::find_if(config_->logical_key_mappings.begin(), 
                          config_->logical_key_mappings.end(),
                          [gpio_id](const LogicalKeyMapping& mapping) {
                              return mapping.gpio_id == gpio_id;
                          });
    
    if (it != config_->logical_key_mappings.end()) {
        config_->logical_key_mappings.erase(it);
        return true;
    }
    
    return false;
}

// 触摸键盘映射管理方法
void InputManager::setTouchKeyboardEnabled(bool enabled) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    config->touch_keyboard_enabled = enabled;
}

inline bool InputManager::getTouchKeyboardEnabled() const {
    return config_->touch_keyboard_enabled;
}

inline void InputManager::setTouchKeyboardMode(TouchKeyboardMode mode) {
    config_->touch_keyboard_mode = mode;
}

inline TouchKeyboardMode InputManager::getTouchKeyboardMode() const {
    return config_->touch_keyboard_mode;
}

// 设置工作模式
inline bool InputManager::setWorkMode(InputWorkMode mode) {
    config_->work_mode = mode;
    return true;
}

// CPU0核心循环 - GTX312L触摸采样和Serial/HID处理
void InputManager::loop0() {
    // 更新所有设备的触摸状态
    updateTouchStates();
    
    // 处理自动灵敏度调整状态机
    if (auto_adjust_context_.active) {
        processAutoAdjustSensitivity();
    }
    
    // 处理绑定状态
    if (binding_active_) {
        processBinding();
        return;
    }
    if (getWorkMode() == InputWorkMode::SERIAL_MODE) {
        processSerialMode();
    }
}

// CPU1核心循环 - 键盘处理和HID发送
void InputManager::loop1() {
    updateGPIOStates();
    processGPIOKeyboard(); // 现在直接调用HID的press_key/release_key方法

    if (config_->work_mode == InputWorkMode::HID_MODE) {
        sendHIDTouchData();
    }
    hid_->task();
}

// 开始Serial绑定
void InputManager::startSerialBinding(InteractiveBindingCallback callback) {
    if (getWorkMode() != InputWorkMode::SERIAL_MODE) {
        if (callback) {
            callback(false, "Not in Serial mode");
        }
        return;
    }
    
    // 初始化Serial绑定状态
    binding_active_ = true;
    binding_callback_ = callback;
    binding_state_ = BindingState::SERIAL_BINDING_INIT;
    current_binding_index_ = 0;
    binding_start_time_ = to_ms_since_boot(get_absolute_time());
    
    // 通过UI管理器显示绑定开始状态
    if (ui_manager_) {
        ui_manager_->show_binding_status("开始Serial绑定", false);
    }
    
    // 备份当前通道状态并启用所有通道
    backupChannelStates();
    enableAllChannels();
}

// 开始HID绑定
void InputManager::startHIDBinding(InteractiveBindingCallback callback) {
    if (getWorkMode() != InputWorkMode::HID_MODE) {
        if (callback) {
            callback(false, "Not in HID mode");
        }
        return;
    }
    
    // 初始化HID绑定状态
    binding_active_ = true;
    binding_callback_ = callback;
    binding_state_ = BindingState::HID_BINDING_INIT;
    binding_start_time_ = to_ms_since_boot(get_absolute_time());
    hid_binding_device_addr_ = 0;
    hid_binding_channel_ = 0;
    hid_binding_x_ = 0.0f;
    hid_binding_y_ = 0.0f;
    
    // 通过UI管理器显示HID绑定开始状态
    if (ui_manager_) {
        ui_manager_->show_binding_status("开始HID绑定", false);
    }
    
    // 备份当前通道状态并启用所有通道
    backupChannelStates();
    enableAllChannels();
}

// 开始自动Serial绑定
bool InputManager::startAutoSerialBinding() {
    if (getWorkMode() != InputWorkMode::SERIAL_MODE) {
        return false;
    }
    
    // 初始化自动绑定状态
    binding_active_ = true;
    binding_callback_ = nullptr;  // 自动绑区不使用回调
    binding_state_ = BindingState::AUTO_SERIAL_BINDING_INIT;
    current_binding_index_ = 0;
    binding_start_time_ = to_ms_since_boot(get_absolute_time());
    
    // 通过UI管理器显示自动绑定开始状态
    if (ui_manager_) {
        ui_manager_->show_binding_status("开始自动Serial绑定", false);
    }
    
    // 备份当前通道状态并启用所有通道
    backupChannelStates();
    enableAllChannels();
    
    return true;
}

// 取消绑定
void InputManager::cancelBinding() {
    if (binding_active_) {
        // 恢复原始通道状态
        restoreChannelStates();
        
        // 重置绑定状态
        binding_active_ = false;
        binding_callback_ = nullptr;
        binding_state_ = BindingState::IDLE;
        current_binding_index_ = 0;
        binding_start_time_ = 0;
        
        // 重置HID绑定相关变量
        hid_binding_device_addr_ = 0;
        hid_binding_channel_ = 0;
        hid_binding_x_ = 0.0f;
        hid_binding_y_ = 0.0f;
    }
}

// 设置HID绑定坐标
void InputManager::setHIDCoordinates(float x, float y) {
    if (binding_active_ && binding_state_ == BindingState::HID_BINDING_SET_COORDS) {
        hid_binding_x_ = x;
        hid_binding_y_ = y;
    }
}

// 确认HID绑定
void InputManager::confirmHIDBinding() {
    if (binding_active_ && binding_state_ == BindingState::HID_BINDING_SET_COORDS) {
        binding_state_ = BindingState::HID_BINDING_COMPLETE;
        updateChannelStatesAfterBinding();
    }
}

// 检查自动绑区是否完成
bool InputManager::isAutoSerialBindingComplete() const {
    return binding_active_ && binding_state_ == BindingState::AUTO_SERIAL_BINDING_WAIT;
}

// 确认自动绑区结果
void InputManager::confirmAutoSerialBinding() {
    if (binding_active_ && binding_state_ == BindingState::AUTO_SERIAL_BINDING_WAIT) {
        binding_state_ = BindingState::AUTO_SERIAL_BINDING_COMPLETE;
        updateChannelStatesAfterBinding();
    }
}

// 自动调整指定通道的灵敏度
uint8_t InputManager::autoAdjustSensitivity(uint16_t device_addr, uint8_t channel) {
    if (channel >= 12) {
        return 0; // 无效通道
    }
    
    // 查找设备
    GTX312L* device = nullptr;
    for (auto* dev : gtx312l_devices_) {
        if (dev->get_physical_device_address().get_device_mask() == device_addr) {
            device = dev;
            break;
        }
    }
    
    if (!device) {
        return 0; // 设备未找到
    }
    
    // 如果状态机正在运行，返回当前灵敏度
    if (auto_adjust_context_.active) {
        return getSensitivity(device_addr, channel);
    }
    
    // 初始化状态机
    auto_adjust_context_.device_addr = device_addr;
    auto_adjust_context_.channel = channel;
    auto_adjust_context_.original_sensitivity = getSensitivity(device_addr, channel);
    auto_adjust_context_.current_sensitivity = 1;
    auto_adjust_context_.touch_found_sensitivity = 0;
    auto_adjust_context_.touch_lost_sensitivity = 0;
    auto_adjust_context_.state = AutoAdjustState::FIND_TOUCH_START;
    auto_adjust_context_.state_start_time = to_ms_since_boot(get_absolute_time());
    auto_adjust_context_.active = true;
    
    return auto_adjust_context_.original_sensitivity;
}

// 状态机处理函数
void InputManager::processAutoAdjustSensitivity() {    
    millis_t current_time = to_ms_since_boot(get_absolute_time());
    millis_t elapsed = current_time - auto_adjust_context_.state_start_time;
    
    // 优化设备查找 - 使用缓存的设备指针或快速查找
    GTX312L* device = nullptr;
    static GTX312L* cached_device = nullptr;
    static uint16_t cached_device_addr = 0;
    
    // 检查缓存是否有效
    if (cached_device && cached_device_addr == auto_adjust_context_.device_addr) {
        device = cached_device;
    } else {
        // 使用指针遍历避免迭代器开销
        GTX312L** device_ptr = gtx312l_devices_.data();
        const size_t device_count = gtx312l_devices_.size();
        
        for (size_t i = 0; i < device_count; i++, device_ptr++) {
            if ((*device_ptr)->get_physical_device_address().get_device_mask() == auto_adjust_context_.device_addr) {
                device = *device_ptr;
                cached_device = device;
                cached_device_addr = auto_adjust_context_.device_addr;
                break;
            }
        }
    }
    
    if (!device) {
        // 设备未找到，清除缓存并结束状态机
        cached_device = nullptr;
        cached_device_addr = 0;
        auto_adjust_context_.active = false;
        return;
    }
    
    switch (auto_adjust_context_.state) {
        case AutoAdjustState::FIND_TOUCH_START:
            // 设置当前灵敏度并开始等待稳定
            setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, 
                          auto_adjust_context_.current_sensitivity);
            auto_adjust_context_.state = AutoAdjustState::FIND_TOUCH_WAIT;
            auto_adjust_context_.state_start_time = current_time;
            break;
            
        case AutoAdjustState::FIND_TOUCH_WAIT:
            if (elapsed >= auto_adjust_context_.stabilize_duration) {
                // 检查是否检测到触摸
                GTX312L_SampleResult result = device->sample_touch_data();
                if (result.physical_addr.mask & (1 << auto_adjust_context_.channel)) {
                    // 找到触摸，记录灵敏度
                    auto_adjust_context_.touch_found_sensitivity = auto_adjust_context_.current_sensitivity;
                    auto_adjust_context_.current_sensitivity = auto_adjust_context_.touch_found_sensitivity;
                    auto_adjust_context_.state = AutoAdjustState::FIND_RELEASE_START;
                    auto_adjust_context_.state_start_time = current_time;
                } else {
                    // 未找到触摸，增加灵敏度
                    auto_adjust_context_.current_sensitivity++;
                    if (auto_adjust_context_.current_sensitivity > 255) {
                        // 未找到触摸，恢复原始灵敏度并结束
                        setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, 
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
            setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, 
                          auto_adjust_context_.current_sensitivity);
            auto_adjust_context_.state = AutoAdjustState::FIND_RELEASE_WAIT;
            auto_adjust_context_.state_start_time = current_time;
            break;
            
        case AutoAdjustState::FIND_RELEASE_WAIT:
            if (elapsed >= auto_adjust_context_.stabilize_duration) {
                // 检查是否失去触摸
                GTX312L_SampleResult result = device->sample_touch_data();
                if (!(result.physical_addr.mask & (1 << auto_adjust_context_.channel))) {
                    // 失去触摸，记录灵敏度
                    auto_adjust_context_.touch_lost_sensitivity = auto_adjust_context_.current_sensitivity;
                    auto_adjust_context_.state = AutoAdjustState::VERIFY_THRESHOLD;
                    auto_adjust_context_.state_start_time = current_time;
                } else {
                    // 仍有触摸，减小灵敏度
                    auto_adjust_context_.current_sensitivity--;
                    if (auto_adjust_context_.current_sensitivity < 1) {
                        // 无法找到释放点，使用中间值
                        uint8_t middle_sensitivity = (auto_adjust_context_.touch_found_sensitivity + 1) / 2;
                        setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, middle_sensitivity);
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
                setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, critical_sensitivity);
                auto_adjust_context_.state = AutoAdjustState::COMPLETE;
                auto_adjust_context_.state_start_time = current_time;
            }
            break;
            
        case AutoAdjustState::COMPLETE:
            if (elapsed >= auto_adjust_context_.stabilize_duration) {
                // 最终验证
                GTX312L_SampleResult result = device->sample_touch_data();
                uint8_t final_sensitivity;
                
                if (result.physical_addr.mask & (1 << auto_adjust_context_.channel)) {
                    // 找到了临界点
                    final_sensitivity = auto_adjust_context_.touch_lost_sensitivity + 1;
                } else {
                    // 需要使用中间值
                    final_sensitivity = (auto_adjust_context_.touch_lost_sensitivity + auto_adjust_context_.touch_found_sensitivity) / 2;
                }
                
                setSensitivity(auto_adjust_context_.device_addr, auto_adjust_context_.channel, final_sensitivity);
                auto_adjust_context_.active = false;
            }
            break;
            
        default:
            auto_adjust_context_.active = false;
            break;
    }
}



// 设置灵敏度
void InputManager::setSensitivity(uint16_t device_addr, uint8_t channel, uint8_t sensitivity) {
    if (channel >= 12) return;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    if (mapping) {
        mapping->sensitivity[channel] = sensitivity;
        
        // 应用到实际设备
        for (auto* device : gtx312l_devices_) {
            if (device->get_physical_device_address().get_device_mask() == device_addr) {
                device->set_sensitivity(channel, sensitivity);
                break;
            }
        }
    }
}

// 获取灵敏度
uint8_t InputManager::getSensitivity(uint16_t device_addr, uint8_t channel) {
    if (channel >= 12) return 15;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    return mapping ? mapping->sensitivity[channel] : 15;
}

// 通过设备名称设置通道灵敏度
bool InputManager::set_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t sensitivity) {
    if (channel >= 12) return false;
    
    // 遍历所有设备找到匹配的设备名称
    for (auto* device : gtx312l_devices_) {
        if (device && device->get_device_name() == device_name) {
            uint16_t device_addr = device->get_physical_device_address().get_device_mask();
            setSensitivity(device_addr, channel, sensitivity);
            return true;
        }
    }
    return false;
}

// 设置Serial映射
void InputManager::setSerialMapping(uint16_t device_addr, uint8_t channel, Mai2_TouchArea area) {
    if (channel >= 12) return;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    if (mapping) {
        mapping->serial_area[channel] = area;
        updateChannelStatesAfterBinding();
    }
}

// 设置HID映射
void InputManager::setHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y) {
    if (channel >= 12) return;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    if (mapping) {
        mapping->hid_area[channel] = TouchAxis(x, y);
        updateChannelStatesAfterBinding();
    }
}

// 获取Serial映射
Mai2_TouchArea InputManager::getSerialMapping(uint16_t device_addr, uint8_t channel) {
    if (channel >= 12) return MAI2_NO_USED;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    return mapping ? mapping->serial_area[channel] : MAI2_NO_USED;
}

// 设置键盘映射
void InputManager::setTouchKeyboardMapping(uint16_t device_addr, uint8_t channel, HID_KeyCode key) {
    if (channel >= 12) return;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    if (mapping) {
        mapping->keyboard_keys[channel] = key;
    }
}

// 获取触摸键盘映射
HID_KeyCode InputManager::getTouchKeyboardMapping(uint16_t device_addr, uint8_t channel) {
    if (channel >= 12) return HID_KeyCode::KEY_NONE;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    return mapping ? mapping->keyboard_keys[channel] : HID_KeyCode::KEY_NONE;
}

// 启用所有通道 - 支持TouchSensor统一接口
void InputManager::enableAllChannels() {
    // 优先使用TouchSensor接口
    for (auto* device : touch_sensor_devices_) {
        uint32_t supported_channels = device->getSupportedChannelCount();
        
        // 尝试转换为GTX312L以使用特定接口
        GTX312L* gtx_device = static_cast<GTX312L*>(device);
        if (gtx_device) {
            for (uint8_t ch = 0; ch < supported_channels && ch < 12; ch++) {
                gtx_device->set_channel_enable(ch, true);
            }
        }
        // 对于其他TouchSensor类型，可以在此添加相应的处理逻辑
    }
    
    // 保持向后兼容性 - 处理未通过TouchSensor接口注册的GTX312L设备
    for (auto* device : gtx312l_devices_) {
        // 检查是否已通过TouchSensor接口处理
        bool already_processed = false;
        for (auto* ts_device : touch_sensor_devices_) {
            if (static_cast<TouchSensor*>(device) == ts_device) {
                already_processed = true;
                break;
            }
        }
        
        if (!already_processed) {
            for (uint8_t ch = 0; ch < 12; ch++) {
                device->set_channel_enable(ch, true);
            }
        }
    }
}

// 仅启用已映射的通道 - 支持TouchSensor统一接口
void InputManager::enableMappedChannels() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();
    
    // 优先使用TouchSensor接口
    for (int i = 0; i < config->device_count && i < touch_sensor_devices_.size(); i++) {
        auto* device = touch_sensor_devices_[i];
        auto& mapping = config->device_mappings[i];
        uint32_t supported_channels = device->getSupportedChannelCount();
        
        // 尝试转换为GTX312L以使用特定接口
         GTX312L* gtx_device = static_cast<GTX312L*>(device);
         if (gtx_device) {
            for (uint8_t ch = 0; ch < supported_channels && ch < 12; ch++) {
                bool has_mapping = false;
                
                if (work_mode == InputWorkMode::SERIAL_MODE) {
                    has_mapping = (mapping.serial_area[ch] != MAI2_NO_USED);
                } else if (work_mode == InputWorkMode::HID_MODE) {
                    has_mapping = (mapping.hid_area[ch].x != 0.0f || mapping.hid_area[ch].y != 0.0f);
                }
                
                // 只有在CH_available中启用且有映射的通道才启用
                bool ch_available = (mapping.CH_available.value & (1 << ch)) != 0;
                bool enabled = ch_available && has_mapping;
                gtx_device->set_channel_enable(ch, enabled);
            }
        }
        // 对于其他TouchSensor类型，可以在此添加相应的处理逻辑
    }
    
    // 保持向后兼容性 - 处理未通过TouchSensor接口注册的GTX312L设备
    int touch_sensor_count = touch_sensor_devices_.size();
    for (int i = touch_sensor_count; i < config->device_count && i < gtx312l_devices_.size(); i++) {
        auto* device = gtx312l_devices_[i];
        auto& mapping = config->device_mappings[i];
        
        for (uint8_t ch = 0; ch < 12; ch++) {
            bool has_mapping = false;
            
            if (work_mode == InputWorkMode::SERIAL_MODE) {
                has_mapping = (mapping.serial_area[ch] != MAI2_NO_USED);
            } else if (work_mode == InputWorkMode::HID_MODE) {
                has_mapping = (mapping.hid_area[ch].x != 0.0f || mapping.hid_area[ch].y != 0.0f);
            }
            
            // 只有在CH_available中启用且有映射的通道才启用
            bool ch_available = (mapping.CH_available.value & (1 << ch)) != 0;
            bool enabled = ch_available && has_mapping;
            device->set_channel_enable(ch, enabled);
        }
    }
}

void InputManager::updateChannelStatesAfterBinding() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();
    
    for (int i = 0; i < config->device_count; i++) {
        auto& mapping = config->device_mappings[i];
        
        for (uint8_t ch = 0; ch < 12; ch++) {
            bool has_mapping = false;
            
            if (work_mode == InputWorkMode::SERIAL_MODE) {
                has_mapping = (mapping.serial_area[ch] != MAI2_NO_USED);
            } else if (work_mode == InputWorkMode::HID_MODE) {
                has_mapping = (mapping.hid_area[ch].x != 0.0f || mapping.hid_area[ch].y != 0.0f);
            }
            
            // 如果没有映射，自动关闭CH_available中的对应通道
            if (!has_mapping) {
                mapping.CH_available.value &= ~(1 << ch);  // 清除对应位
            }
        }
    }
    
    // 重新应用通道映射
    enableMappedChannels();
}

// 更新触摸状态
inline void InputManager::updateTouchStates() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    millis_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 使用TouchSensor统一接口更新触摸状态
    for (int i = 0; i < config->device_count && i < touch_sensor_devices_.size(); i++) {
        auto* device = touch_sensor_devices_[i];
        
        // 保存之前的状态
        previous_touch_states_[i] = current_touch_states_[i];
        
        // 使用TouchSensor统一接口获取触摸状态
        uint32_t touch_state = device->getCurrentTouchState();
        
        // 构造GTX312L_SampleResult结构体以保持兼容性
        current_touch_states_[i].physical_addr.mask = touch_state & 0xFFFF;
        current_touch_states_[i].timestamp = current_time;
        
        // 增加采样计数器
        incrementSampleCounter();
    }
    
    // 如果还有GTX312L设备未通过TouchSensor接口注册，保持兼容性
    int touch_sensor_count = touch_sensor_devices_.size();
    for (int i = touch_sensor_count; i < config->device_count && i < gtx312l_devices_.size(); i++) {
        auto* device = gtx312l_devices_[i];
        
        // 保存之前的状态
        previous_touch_states_[i] = current_touch_states_[i];
        
        // 获取当前触摸状态，直接存储GTX312L_SampleResult
        current_touch_states_[i] = device->sample_touch_data();
        
        // 增加采样计数器
        incrementSampleCounter();
        current_touch_states_[i].timestamp = current_time;
    }
}

// 处理Serial模式 - 高性能优化版本
inline void InputManager::processSerialMode() {
    Mai2Serial_TouchState touch_state;
    touch_state.parts.state1 = 0;
    touch_state.parts.state2 = 0;
    
    // 使用静态触摸键盘bitmap，避免重复构造
    static KeyboardBitmap touch_keyboard_bitmap;
    touch_keyboard_bitmap.clear();
    
    // 预计算设备映射指针数组，避免重复查找
    GTX312L_DeviceMapping* device_mappings[8];
    const int device_count = config_->device_count;
    
    // 预处理设备映射，建立快速查找表
    for (int i = 0; i < device_count; i++) {
        const uint16_t device_addr = current_touch_states_[i].physical_addr.mask & 0xF000;
        device_mappings[i] = nullptr;
        
        const GTX312L_DeviceMapping* mapping_ptr = config_->device_mappings;
        for (int j = 0; j < device_count; j++, mapping_ptr++) {
            if ((mapping_ptr->device_addr & 0xF000) == device_addr) {
                device_mappings[i] = const_cast<GTX312L_DeviceMapping*>(mapping_ptr);
                break;
            }
        }
    }
    
    // 主处理循环 - 优化版本
    for (int i = 0; i < device_count; i++) {
        const GTX312L_DeviceMapping* mapping = device_mappings[i];
        if (!mapping) continue;
        
        const uint16_t current_mask = current_touch_states_[i].physical_addr.mask;
        
        // 使用位运算批量处理12个通道
        for (uint8_t ch = 0; ch < 12; ch++) {
            const uint16_t ch_mask = (1 << ch);
            const bool touched = (current_mask & ch_mask) != 0;
            
            // 处理Serial触摸映射 - 使用位运算优化
            const Mai2_TouchArea area = mapping->serial_area[ch];
            if (area != MAI2_NO_USED && area >= 1 && area <= 34) {
                const uint8_t bit_index = area - 1;
                const uint32_t bit_mask = (1UL << (bit_index & 31));
                
                // 使用位运算选择state1或state2
                if (bit_index < 32) {
                    touch_state.parts.state1 = touched ? 
                        (touch_state.parts.state1 | bit_mask) : 
                        (touch_state.parts.state1 & ~bit_mask);
                } else {
                    touch_state.parts.state2 = touched ? 
                        (touch_state.parts.state2 | bit_mask) : 
                        (touch_state.parts.state2 & ~bit_mask);
                }
            }
            
            // 处理触摸键盘映射 - 内联setKey避免函数调用
            const HID_KeyCode key = mapping->keyboard_keys[ch];
            if (key != HID_KeyCode::KEY_NONE) {
                const uint8_t bit_idx = touch_keyboard_bitmap.getBitIndex(key);
                if (bit_idx < 128) {
                    const uint64_t key_mask = (1ULL << (bit_idx & 63));
                    if (bit_idx < 64) {
                        touch_keyboard_bitmap.bitmap_low = touched ? 
                            (touch_keyboard_bitmap.bitmap_low | key_mask) : 
                            (touch_keyboard_bitmap.bitmap_low & ~key_mask);
                    } else {
                        touch_keyboard_bitmap.bitmap_high = touched ? 
                            (touch_keyboard_bitmap.bitmap_high | key_mask) : 
                            (touch_keyboard_bitmap.bitmap_high & ~key_mask);
                    }
                }
            }
        }
    }
    
    // Serial数据发送
    if (mai2_serial_) {
        mai2_serial_->send_touch_state(touch_state.parts.state1, touch_state.parts.state2);
    }
}

// 发送HID触摸数据
inline void InputManager::sendHIDTouchData() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    
    // 预计算设备映射指针数组，避免重复查找
    GTX312L_DeviceMapping* device_mappings[8] = {nullptr};
    const int device_count = config->device_count;
    
    // 预处理设备映射，建立快速查找表
    for (int i = 0; i < device_count; i++) {
        uint16_t device_addr = current_touch_states_[i].physical_addr.mask & 0xF000;
        
        // 查找对应的映射配置
        GTX312L_DeviceMapping* mapping_ptr = config->device_mappings;
        for (int j = 0; j < device_count; j++, mapping_ptr++) {
            if ((mapping_ptr->device_addr & 0xF000) == device_addr) {
                device_mappings[i] = mapping_ptr;
                break;
            }
        }
    }
    
    // 处理所有设备的触摸数据
    for (int i = 0; i < device_count; i++) {
        GTX312L_DeviceMapping* mapping = device_mappings[i];
        if (!mapping) continue;
        
        uint16_t current_mask = current_touch_states_[i].physical_addr.mask;
        if (!current_mask) continue; // 无触摸数据
        
        // 使用位运算快速处理所有通道
        TouchAxis* hid_area_ptr = mapping->hid_area;
        for (uint8_t ch = 0; ch < 12; ch++, hid_area_ptr++) {
            if (!(current_mask & (1 << ch))) continue;
            
            // 检查是否有有效映射
            if (hid_area_ptr->x == 0.0f && hid_area_ptr->y == 0.0f) continue;
            
            // 计算唯一的触摸点ID：设备索引(4位) + 通道号(4位)
            uint8_t unique_contact_id = (i << 4) | ch;
            
            // 创建触摸点报告
            HID_TouchPoint touch_point;
            touch_point.press = true;
            touch_point.id = unique_contact_id;
            
            // 转换坐标到HID范围 (0-65535)
            touch_point.x = (uint16_t)(hid_area_ptr->x * 65535.0f);
            touch_point.y = (uint16_t)(hid_area_ptr->y * 65535.0f);
            
            // 发送触摸点
            if (hid_) {
                hid_->send_touch_report(touch_point);
            }
        }
    }
}

// 查找设备索引
int InputManager::findDeviceIndex(uint16_t device_addr) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    
    for (int i = 0; i < config->device_count; i++) {
        if (config->device_mappings[i].device_addr == device_addr) {
            return i;
        }
    }
    return -1;
}

// 查找设备映射
GTX312L_DeviceMapping* InputManager::findDeviceMapping(uint16_t device_addr) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    int index = findDeviceIndex(device_addr);
    return (index >= 0) ? &config->device_mappings[index] : nullptr;
}

// 静态配置变量
static InputManager_PrivateConfig static_config_;

// [默认配置注册函数] - 注册所有InputManager的默认配置到ConfigManager
void inputmanager_register_default_configs(config_map_t& default_map) {
    // 注册InputManager默认配置
    default_map[INPUTMANAGER_WORK_MODE] = ConfigValue((uint8_t)0);  // 默认工作模式
    default_map[INPUTMANAGER_TOUCH_KEYBOARD_ENABLED] = ConfigValue(true);  // 默认启用触摸键盘
    default_map[INPUTMANAGER_DEVICE_COUNT] = ConfigValue((uint8_t)0);  // 默认设备数量
    
    // 二进制数据配置使用空字符串作为默认值
    default_map[INPUTMANAGER_GTX312L_DEVICES] = ConfigValue(std::string(""));
    default_map[INPUTMANAGER_PHYSICAL_KEYBOARDS] = ConfigValue(std::string(""));
    default_map[INPUTMANAGER_LOGICAL_MAPPINGS] = ConfigValue(std::string(""));
}

// [配置保管函数] - 返回静态配置变量的指针
InputManager_PrivateConfig* inputmanager_get_config_holder() {
    return &static_config_;
}

// [配置加载函数] - 从ConfigManager加载所有配置到静态配置变量
bool inputmanager_load_config_from_manager() {
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    // 加载工作模式
    static_config_.work_mode = static_cast<InputWorkMode>(config_mgr->get_uint8(INPUTMANAGER_WORK_MODE));
    
    // 加载触摸键盘启用状态
    static_config_.touch_keyboard_enabled = config_mgr->get_bool(INPUTMANAGER_TOUCH_KEYBOARD_ENABLED);
    
    // 加载触摸键盘模式
    static_config_.touch_keyboard_mode = static_cast<TouchKeyboardMode>(config_mgr->get_uint8(INPUTMANAGER_TOUCH_KEYBOARD_MODE));
    
    // 加载设备数量
    static_config_.device_count = config_mgr->get_uint8(INPUTMANAGER_DEVICE_COUNT);
    
    // 加载GTX312L设备映射数据
    std::string devices_str = config_mgr->get_string(INPUTMANAGER_GTX312L_DEVICES);
    if (!devices_str.empty()) {
        size_t expected_size = sizeof(GTX312L_DeviceMapping) * static_config_.device_count;
        if (devices_str.size() >= expected_size) {
            std::memcpy(static_config_.device_mappings, devices_str.data(), expected_size);
        }
    }
    
    // 加载物理键盘映射数据
    std::string physical_keyboards_str = config_mgr->get_string(INPUTMANAGER_PHYSICAL_KEYBOARDS);
    if (!physical_keyboards_str.empty()) {
        size_t mapping_count = physical_keyboards_str.size() / sizeof(PhysicalKeyboardMapping);
        static_config_.physical_keyboard_mappings.clear();
        static_config_.physical_keyboard_mappings.resize(mapping_count);
        std::memcpy(static_config_.physical_keyboard_mappings.data(), 
                   physical_keyboards_str.data(),
                     physical_keyboards_str.size());
    }
    
    // 加载逻辑按键映射数据
    std::string logical_mappings_str = config_mgr->get_string(INPUTMANAGER_LOGICAL_MAPPINGS);
    if (!logical_mappings_str.empty()) {
        size_t mapping_count = logical_mappings_str.size() / sizeof(LogicalKeyMapping);
        static_config_.logical_key_mappings.clear();
        static_config_.logical_key_mappings.resize(mapping_count);
        std::memcpy(static_config_.logical_key_mappings.data(),
                     logical_mappings_str.data(),
                     logical_mappings_str.size());
    }
    
    return true;
}

// UI专用接口实现
void InputManager::get_all_device_status(TouchDeviceStatus data[8], int& device_count) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    device_count = config->device_count;
    
    for (int i = 0; i < device_count && i < 8; i++) {
        // 复制设备映射配置
        data[i].device = config->device_mappings[i];
        
        // 获取当前触摸状态
        data[i].touch_states = previous_touch_states_[i].physical_addr.mask;
        
        // 设置连接状态（假设所有配置的设备都已连接）
        data[i].is_connected = true;
        
        // 生成设备名称（16位地址转换为HEX）
        uint16_t device_addr = data[i].device.device_addr;
        char hex_name[8];
        snprintf(hex_name, sizeof(hex_name), "%04X", device_addr);
        data[i].device_name = std::string(hex_name);
    }
}

// [配置读取函数] - 返回当前配置的副本
InputManager_PrivateConfig inputmanager_get_config_copy() {
    return static_config_;
}

// [配置写入函数] - 将配置写入ConfigManager并保存
bool inputmanager_write_config_to_manager(const InputManager_PrivateConfig& config) {
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    // 写入工作模式
    config_mgr->set_uint8(INPUTMANAGER_WORK_MODE, static_cast<uint8_t>(config.work_mode));
    
    // 写入触摸键盘启用状态
    config_mgr->set_bool(INPUTMANAGER_TOUCH_KEYBOARD_ENABLED, config.touch_keyboard_enabled);
    
    // 写入触摸键盘模式
    config_mgr->set_uint8(INPUTMANAGER_TOUCH_KEYBOARD_MODE, static_cast<uint8_t>(config.touch_keyboard_mode));
    
    // 写入设备数量
    config_mgr->set_uint8(INPUTMANAGER_DEVICE_COUNT, config.device_count);
    
    // 写入GTX312L设备映射数据
    if (config.device_count > 0) {
        size_t devices_size = sizeof(GTX312L_DeviceMapping) * config.device_count;
        std::string devices_data(devices_size, '\0');
        std::memcpy(&devices_data[0], config.device_mappings, devices_size);
        config_mgr->set_string(INPUTMANAGER_GTX312L_DEVICES, devices_data);
    }
    
    // 写入物理键盘映射数据
    if (!config.physical_keyboard_mappings.empty()) {
        size_t keyboards_size = sizeof(PhysicalKeyboardMapping) * config.physical_keyboard_mappings.size();
        std::string keyboards_data(keyboards_size, '\0');
        std::memcpy(&keyboards_data[0], 
                   config.physical_keyboard_mappings.data(), 
                   keyboards_size);
        config_mgr->set_string(INPUTMANAGER_PHYSICAL_KEYBOARDS, keyboards_data);
    }
    
    // 写入逻辑按键映射数据
    if (!config.logical_key_mappings.empty()) {
        size_t mappings_size = sizeof(LogicalKeyMapping) * config.logical_key_mappings.size();
        std::string mappings_data(mappings_size, '\0');
        std::memcpy(&mappings_data[0], 
                   config.logical_key_mappings.data(), 
                   mappings_size);
        config_mgr->set_string(INPUTMANAGER_LOGICAL_MAPPINGS, mappings_data);
    }
    
    // 保存所有配置并更新静态配置
    config_mgr->save_config();
    static_config_ = config;
    
    return true;
}

// 绑定处理主函数
void InputManager::processBinding() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 检查超时
    if (current_time - binding_start_time_ > binding_timeout_ms_) {
        if (binding_callback_) {
            binding_callback_(false, "Binding timeout");
        }
        cancelBinding();
        return;
    }
    
    // 根据绑定状态处理
    switch (binding_state_) {
        case BindingState::SERIAL_BINDING_INIT:
        case BindingState::SERIAL_BINDING_WAIT_TOUCH:
        case BindingState::SERIAL_BINDING_PROCESSING:
        case BindingState::SERIAL_BINDING_COMPLETE:
            processSerialBinding();
            break;
            
        case BindingState::HID_BINDING_INIT:
        case BindingState::HID_BINDING_WAIT_TOUCH:
        case BindingState::HID_BINDING_SET_COORDS:
        case BindingState::HID_BINDING_COMPLETE:
            processHIDBinding();
            break;
            
        case BindingState::AUTO_SERIAL_BINDING_INIT:
        case BindingState::AUTO_SERIAL_BINDING_SCAN:
        case BindingState::AUTO_SERIAL_BINDING_WAIT:
        case BindingState::AUTO_SERIAL_BINDING_COMPLETE:
            processAutoSerialBinding();
            break;
            
        default:
            break;
    }
}

// 获取Serial绑定区域的辅助函数
static Mai2_TouchArea getSerialBindingArea(uint8_t index) {
    // Mai2_TouchArea枚举是连续的，从MAI2_AREA_A1=1开始到MAI2_AREA_E8=34
    if (index < 34) {
        return static_cast<Mai2_TouchArea>(MAI2_AREA_A1 + index);
    }
    return MAI2_NO_USED;
}

// Serial绑定处理
void InputManager::processSerialBinding() {
    switch (binding_state_) {
        case BindingState::SERIAL_BINDING_INIT:
            // 发送当前要绑定区域的mai2消息
            if (current_binding_index_ < 34) {
                Mai2_TouchArea current_area = getSerialBindingArea(current_binding_index_);
                sendMai2TouchMessage(current_area);
                binding_state_ = BindingState::SERIAL_BINDING_WAIT_TOUCH;
                
                // 通知UI显示当前绑定区域
                if (binding_callback_) {
                    char message[64];
                    snprintf(message, sizeof(message), "Please touch %s area", getMai2AreaName(current_area));
                    binding_callback_(true, message);
                }
                
                // 通过UI管理器显示绑定状态
                 if (ui_manager_) {
                     char message[64];
                     snprintf(message, sizeof(message), "绑定区域: %s (%d/34)", getMai2AreaName(current_area), current_binding_index_ + 1);
                 }
            } else {
                // 所有区域绑定完成
                binding_state_ = BindingState::SERIAL_BINDING_COMPLETE;
            }
            break;
            
        case BindingState::SERIAL_BINDING_WAIT_TOUCH:
            // 检测触摸输入
            for (size_t dev_idx = 0; dev_idx < gtx312l_devices_.size(); dev_idx++) {
                uint16_t current_state = current_touch_states_[dev_idx].physical_addr.mask;
                uint16_t previous_state = previous_touch_states_[dev_idx].physical_addr.mask;
                uint16_t new_touches = current_state & ~previous_state;
                
                if (new_touches != 0) {
                    // 找到第一个新触摸的通道
                    for (uint8_t ch = 0; ch < 12; ch++) {
                        if (new_touches & (1 << ch)) {
                            // 绑定当前区域到这个通道
                            Mai2_TouchArea current_area = getSerialBindingArea(current_binding_index_);
                            uint16_t device_addr = gtx312l_devices_[dev_idx]->get_physical_device_address().get_device_mask();
                            setSerialMapping(device_addr, ch, current_area);
                            
                            // 禁用这个通道以避免重复触发
                            GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
                            if (mapping) {
                                mapping->CH_available.value &= ~(1 << ch);  // 清除对应位
                                gtx312l_devices_[dev_idx]->set_channel_enable(ch, false);
                            }
                            
                            // 进入下一个区域绑定
                            current_binding_index_++;
                            binding_state_ = BindingState::SERIAL_BINDING_INIT;
                            return;
                        }
                    }
                }
            }
            break;
            
        case BindingState::SERIAL_BINDING_COMPLETE:
            // 保存配置并完成绑定
            {
                InputManager_PrivateConfig* config = inputmanager_get_config_holder();
                inputmanager_write_config_to_manager(*config);
                
                if (binding_callback_) {
                    binding_callback_(true, "Serial binding completed successfully");
                }
                
                // 恢复原始通道状态
                restoreChannelStates();
                
                // 重置绑定状态
                binding_active_ = false;
                binding_callback_ = nullptr;
                binding_state_ = BindingState::IDLE;
            }
            break;
            
        default:
            break;
    }
}

// 自动Serial绑定处理
void InputManager::processAutoSerialBinding() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    switch (binding_state_) {
        case BindingState::AUTO_SERIAL_BINDING_INIT:
            // 开始扫描所有触摸输入
            binding_state_ = BindingState::AUTO_SERIAL_BINDING_SCAN;
            
            if (binding_callback_) {
                binding_callback_(true, "开始扫描触摸输入，请按顺序触摸所有区域");
            }
            
            // 通过UI管理器显示扫描状态
            if (ui_manager_) {
                ui_manager_->show_binding_status("扫描模式：请按顺序触摸所有区域", false);
                ui_manager_->update_guided_binding_progress(0, "开始扫描触摸输入");
            }
            break;
            
        case BindingState::AUTO_SERIAL_BINDING_SCAN:
            // 检测触摸输入并自动分配
            {
                bool touch_detected = false;
                
                // 检查所有设备的触摸状态
                for (size_t dev_idx = 0; dev_idx < gtx312l_devices_.size(); dev_idx++) {
                    uint16_t current_state = current_touch_states_[dev_idx].physical_addr.mask;
                    uint16_t previous_state = previous_touch_states_[dev_idx].physical_addr.mask;
                    uint16_t new_touches = current_state & ~previous_state;
                    
                    if (new_touches != 0) {
                        // 找到第一个新触摸的通道
                        for (uint8_t ch = 0; ch < 12; ch++) {
                            if (new_touches & (1 << ch)) {
                                // 获取设备地址
                                uint16_t device_addr = gtx312l_devices_[dev_idx]->get_physical_device_address().get_device_mask();
                                
                                // 检查这个通道是否已经被绑定
                                Mai2_TouchArea existing_area = getSerialMapping(device_addr, ch);
                                if (existing_area == MAI2_NO_USED && current_binding_index_ < 34) {
                                    // 自动分配下一个可用的Mai2区域
                                    Mai2_TouchArea target_area = getSerialBindingArea(current_binding_index_);
                                    setSerialMapping(device_addr, ch, target_area);
                                    
                                    current_binding_index_++;
                                    touch_detected = true;
                                    
                                    // 更新UI显示
                                    if (ui_manager_) {
                                        char message[128];
                                        snprintf(message, sizeof(message), "检测到触摸：设备%04X 通道%d -> %s", 
                                                device_addr, ch, getMai2AreaName(target_area));
                                        ui_manager_->show_binding_status(message, false);
                                        ui_manager_->update_guided_binding_progress(current_binding_index_, "自动绑定进行中");
                                    }
                                    
                                    if (binding_callback_) {
                                        char message[128];
                                        snprintf(message, sizeof(message), "绑定成功：%s (%d/34)", 
                                                getMai2AreaName(target_area), current_binding_index_);
                                        binding_callback_(true, message);
                                    }
                                    
                                    // 如果所有区域都已绑定，进入等待确认状态
                                    if (current_binding_index_ >= 34) {
                                        binding_state_ = BindingState::AUTO_SERIAL_BINDING_WAIT;
                                        
                                        if (ui_manager_) {
                                            ui_manager_->show_binding_status("自动绑定完成，请确认", true);
                                        }
                                        
                                        if (binding_callback_) {
                                            binding_callback_(true, "自动绑定完成，请确认保存");
                                        }
                                    }
                                    
                                    break;
                                }
                            }
                        }
                        
                        if (touch_detected) break;
                    }
                }
            }
            break;
            
        case BindingState::AUTO_SERIAL_BINDING_WAIT:
            // 等待用户确认或超时
            if (current_time - binding_start_time_ > binding_timeout_ms_) {
                // 超时自动保存
                binding_state_ = BindingState::AUTO_SERIAL_BINDING_COMPLETE;
            }
            // 用户可以通过UI确认或取消
            break;
            
        case BindingState::AUTO_SERIAL_BINDING_COMPLETE:
            // 保存配置并完成绑定
            {
                InputManager_PrivateConfig* config = inputmanager_get_config_holder();
                inputmanager_write_config_to_manager(*config);
                
                if (binding_callback_) {
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
            break;
            
        default:
            break;
    }
}

// HID绑定处理
void InputManager::processHIDBinding() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    switch (binding_state_) {
        case BindingState::HID_BINDING_INIT:
            binding_state_ = BindingState::HID_BINDING_WAIT_TOUCH;
            if (binding_callback_) {
                binding_callback_(false, "开始HID绑定，请按下要绑定的区域");
            }
            
            // 通过UI管理器显示HID绑定状态
             if (ui_manager_) {
                 ui_manager_->show_binding_status("等待触摸输入...", false);
             }
            break;
            
        case BindingState::HID_BINDING_WAIT_TOUCH:
            // 检查超时
            if (current_time - binding_start_time_ > binding_timeout_ms_) {
                if (binding_callback_) {
                    binding_callback_(false, "HID绑定超时");
                }
                cancelBinding();
                return;
            }
            
            // 检测触摸输入
            for (size_t dev_idx = 0; dev_idx < gtx312l_devices_.size(); dev_idx++) {
                uint16_t current_state = current_touch_states_[dev_idx].physical_addr.mask;
                uint16_t previous_state = previous_touch_states_[dev_idx].physical_addr.mask;
                uint16_t new_touches = current_state & ~previous_state;
                
                if (new_touches != 0) {
                    // 找到第一个新触摸的通道
                    for (uint8_t ch = 0; ch < 12; ch++) {
                        if (new_touches & (1 << ch)) {
                            hid_binding_device_addr_ = gtx312l_devices_[dev_idx]->get_physical_device_address().get_device_mask();
                            hid_binding_channel_ = ch;
                            hid_binding_x_ = 0.5f;  // 默认中心坐标
                            hid_binding_y_ = 0.5f;
                            binding_state_ = BindingState::HID_BINDING_SET_COORDS;
                            
                            if (binding_callback_) {
                                char message[128];
                                snprintf(message, sizeof(message), "检测到触摸点位，设备:0x%04X 通道:%d，请在屏幕上设置X Y坐标", 
                                        hid_binding_device_addr_, ch);
                                binding_callback_(false, message);
                            }
                            
                            // 通过UI管理器显示坐标设置界面
                             if (ui_manager_) {
                                 char message[64];
                                 snprintf(message, sizeof(message), "设置坐标 - 设备:0x%04X 通道:%d", hid_binding_device_addr_, ch);
                                 ui_manager_->show_binding_status(message, false);
                             }
                            return;
                        }
                    }
                }
            }
            break;
            
        case BindingState::HID_BINDING_SET_COORDS:
            // 实时发送HID触摸数据，坐标跟随当前设置
            if (hid_) {
                // 构造HID触摸点
                HID_TouchPoint touch_point;
                touch_point.press = 1;
                touch_point.id = 1;
                touch_point.x = (uint16_t)(hid_binding_x_ * 32767);
                touch_point.y = (uint16_t)(hid_binding_y_ * 32767);

                hid_->send_touch_report(touch_point);
            }
            
            // 检查是否有新的坐标设置或确认绑定的信号
            // 这里需要UI层调用setHIDCoordinates方法来更新坐标
            // 或者调用confirmHIDBinding来确认绑定
            break;
            
        case BindingState::HID_BINDING_COMPLETE:
            // 保存绑定结果
            setHIDMapping(hid_binding_device_addr_, hid_binding_channel_, hid_binding_x_, hid_binding_y_);
            
            // 保存配置并完成绑定
            {
                InputManager_PrivateConfig* config = inputmanager_get_config_holder();
                inputmanager_write_config_to_manager(*config);
                
                if (binding_callback_) {
                    char message[128];
                    snprintf(message, sizeof(message), "HID绑定完成，坐标:(%.2f,%.2f)", hid_binding_x_, hid_binding_y_);
                    binding_callback_(true, message);
                }
                
                // 恢复原始通道状态
                restoreChannelStates();
                
                // 重置绑定状态
                binding_active_ = false;
                binding_callback_ = nullptr;
                binding_state_ = BindingState::IDLE;
            }
            break;
            
        default:
            break;
    }
}

// 备份通道状态
void InputManager::backupChannelStates() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    for (uint8_t i = 0; i < config->device_count && i < 8; i++) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            original_channels_backup_[i][ch] = (config->device_mappings[i].CH_available.value & (1 << ch)) ? 1 : 0;
        }
    }
}

// 恢复通道状态
void InputManager::restoreChannelStates() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    for (uint8_t i = 0; i < config->device_count && i < 8; i++) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            if (original_channels_backup_[i][ch] != 0) {
                config->device_mappings[i].CH_available.value |= (1 << ch);  // 设置位
            } else {
                config->device_mappings[i].CH_available.value &= ~(1 << ch); // 清除位
            }
            
            // 更新硬件设备状态
            if (i < gtx312l_devices_.size()) {
                gtx312l_devices_[i]->set_channel_enable(ch, original_channels_backup_[i][ch]);
            }
        }
    }
}

// 发送Mai2触摸消息
void InputManager::sendMai2TouchMessage(Mai2_TouchArea area) {
    if (mai2_serial_) {
        // 创建触摸状态，只设置指定区域为触摸状态
        Mai2Serial_TouchState touch_state;
        touch_state.parts.state1 = 0;
        touch_state.parts.state2 = 0;
        
        // 设置对应区域的状态
        if (area >= 1 && area <= 34) {
            uint8_t bit_index = area - 1;
            if (bit_index < 32) {
                touch_state.parts.state1 |= (1UL << bit_index);
            } else {
                bit_index -= 32;
                touch_state.parts.state2 |= (1 << bit_index);
            }
        }
        
        // 发送触摸状态
         mai2_serial_->send_touch_state(touch_state.parts.state1, touch_state.parts.state2);
    }
}

// 获取Mai2区域名称
const char* InputManager::getMai2AreaName(Mai2_TouchArea area) {
    if (area >= MAI2_NO_USED && area <= MAI2_AREA_E8) {
        return mai2_area_names[area];
    }
    return "UNKNOWN";
}

// GPIO键盘处理方法实现
void InputManager::updateGPIOStates() {
    // 高性能GPIO状态读取：使用位运算批量处理
    
    // 批量读取MCU GPIO状态 - 避免循环，使用硬件寄存器直接读取
    // RP2040 GPIO状态寄存器：SIO_BASE + SIO_GPIO_IN_OFFSET
    mcu_gpio_states_ = sio_hw->gpio_in & 0x3FFFFFFF; // 30位GPIO掩码
    
    // 读取MCP23S17 GPIO状态
    if (mcp23s17_available_ && mcp23s17_) {
        mcp23s17_->read_all_gpio(mcp_gpio_states_);
    }
}

void InputManager::processGPIOKeyboard() {
    // 高性能GPIO处理：零内存分配，使用类成员缓存变量
    static KeyboardBitmap prev_keyboard_state; // 跟踪上一次的按键状态
    static KeyboardBitmap current_keyboard_state;
    // 零内存分配：使用类成员缓存变量计算所有变化位图和反转位图
    gpio_mcu_changed_ = mcu_gpio_states_ ^ mcu_gpio_previous_states_;
    gpio_mcu_inverted_ = ~mcu_gpio_states_; // 低电平有效
    gpio_mcp_changed_a_ = mcp_gpio_states_.port_a ^ mcp_gpio_previous_states_.port_a;
    gpio_mcp_changed_b_ = mcp_gpio_states_.port_b ^ mcp_gpio_previous_states_.port_b;
    gpio_mcp_inverted_a_ = ~mcp_gpio_states_.port_a; // 低电平有效
    gpio_mcp_inverted_b_ = ~mcp_gpio_states_.port_b;
    
    // 快速跳过：如果没有GPIO变化则直接返回
    if (!gpio_mcu_changed_ && !gpio_mcp_changed_a_ && !gpio_mcp_changed_b_) {
        return;
    }
    
    current_keyboard_state.clear();
    
    // 零内存分配：使用类成员缓存变量获取所有指针和计数
    gpio_mappings_cache_ = config_->physical_keyboard_mappings.data();
    gpio_mapping_count_cache_ = config_->physical_keyboard_mappings.size();
    gpio_logical_mappings_cache_ = config_->logical_key_mappings.data();
    gpio_logical_count_cache_ = config_->logical_key_mappings.size();
    
    for (size_t i = 0; i < gpio_mapping_count_cache_; ++i) {
        gpio_mapping_ptr_cache_ = &gpio_mappings_cache_[i];
        gpio_pin_cache_ = gpio_mapping_ptr_cache_->gpio;
        gpio_pin_num_cache_ = get_gpio_pin_number(gpio_pin_cache_);
        
        // 使用位运算快速判断GPIO类型和状态
        if ((gpio_pin_cache_ & 0xC0) == 0x00) { // MCU GPIO
            gpio_current_state_cache_ = (gpio_mcu_inverted_ >> gpio_pin_num_cache_) & 1;
        } else { // MCP GPIO
            if (gpio_pin_num_cache_ <= 8) { // PORTA
                gpio_bit_pos_cache_ = gpio_pin_num_cache_ - 1;
                gpio_current_state_cache_ = (gpio_mcp_inverted_a_ >> gpio_bit_pos_cache_) & 1;
            } else { // PORTB
                gpio_bit_pos_cache_ = gpio_pin_num_cache_ - 9;
                gpio_current_state_cache_ = (gpio_mcp_inverted_b_ >> gpio_bit_pos_cache_) & 1;
            }
        }
        
        // 普通键盘处理：设置当前状态
        if (gpio_current_state_cache_) {
            if (gpio_mapping_ptr_cache_->default_key != HID_KeyCode::KEY_NONE) {
                current_keyboard_state.setKey(gpio_mapping_ptr_cache_->default_key, true);
            }
            // 内联逻辑键处理避免函数调用，使用类成员缓存变量
            for (size_t j = 0; j < gpio_logical_count_cache_; ++j) {
                if (gpio_logical_mappings_cache_[j].gpio_id == gpio_pin_cache_) {
                    // 使用类成员缓存指针和位运算展开循环
                    gpio_keys_ptr_cache_ = gpio_logical_mappings_cache_[j].keys;
                    if (gpio_keys_ptr_cache_[0] != HID_KeyCode::KEY_NONE) current_keyboard_state.setKey(gpio_keys_ptr_cache_[0], true);
                    if (gpio_keys_ptr_cache_[1] != HID_KeyCode::KEY_NONE) current_keyboard_state.setKey(gpio_keys_ptr_cache_[1], true);
                    if (gpio_keys_ptr_cache_[2] != HID_KeyCode::KEY_NONE) current_keyboard_state.setKey(gpio_keys_ptr_cache_[2], true);
                    break;
                }
            }
        }
    }
    
    // 比较当前状态与上一次状态，发送按键变化事件
    if (hid_) {
        for (uint8_t i = 0; i < SUPPORTED_KEYS_COUNT; i++) {
            HID_KeyCode key = supported_keys[i];
            bool current_pressed = current_keyboard_state.getKey(key);
            bool prev_pressed = prev_keyboard_state.getKey(key);
            
            if (current_pressed != prev_pressed) {
                if (current_pressed) {
                    hid_->press_key(key);
                } else {
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
uint8_t InputManager::getTouchSampleRate(uint16_t device_addr) {
    // 返回实际测量的采样频率
    return static_cast<uint8_t>(current_sample_rate_);
}

void InputManager::incrementSampleCounter() {
    sample_counter_++;
    
    // 检查是否需要重置计数器（每秒一次）
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_reset_time_ >= 1000) {
        current_sample_rate_ = sample_counter_;
        sample_counter_ = 0;
        last_reset_time_ = current_time;
    }
}

void InputManager::resetSampleCounter() {
    sample_counter_ = 0;
    current_sample_rate_ = 0;
    last_reset_time_ = to_ms_since_boot(get_absolute_time());
}

// 获取HID键盘回报速率
uint8_t InputManager::getHIDReportRate() {
    if (hid_ && hid_->is_initialized()) {
        // 返回实际测试的HID报告速率 (Hz)
        return hid_->get_report_rate();
    }
    return 0; // HID未初始化
}

// 获取物理键盘映射列表
const std::vector<PhysicalKeyboardMapping>& InputManager::getPhysicalKeyboards() const {
    return config_->physical_keyboard_mappings;
}

// 清除所有逻辑按键映射
void InputManager::clearAllLogicalKeyMappings() {
    config_->logical_key_mappings.clear();
    inputmanager_write_config_to_manager(*config_);
}

// 获取逻辑按键映射列表
const std::vector<LogicalKeyMapping>& InputManager::getLogicalKeyMappings() const {
    return config_->logical_key_mappings;
}