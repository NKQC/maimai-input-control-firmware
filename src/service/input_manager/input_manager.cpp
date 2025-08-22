#include "input_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include <cstring>
#include <algorithm>

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
    , ui_manager_(nullptr)
    , mcp23s17_(nullptr)
    , config_(inputmanager_get_config_holder())
    , mcp23s17_available_(false)
    , mcu_gpio_states_(0)
    , mcu_gpio_previous_states_(0)
    , gpio_keyboard_bitmap_()
    , touch_bitmap_cache_() {
    
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
    ui_manager_ = config.ui_manager;
    mcp23s17_ = config.mcp23s17;
    mcp23s17_available_ = (mcp23s17_ != nullptr);
    
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
bool InputManager::registerGTX312L(GTX312L* device) {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    
    if (!device || config->device_count >= 8) {
        return false;
    }
    
    // 获取设备地址
    uint16_t device_addr = device->get_physical_device_address().get_device_mask();
    
    // 检查是否已经注册
    for (const auto& registered_device : gtx312l_devices_) {
        if (registered_device->get_physical_device_address().get_device_mask() == device_addr) {
            return false; // 已经注册
        }
    }
    
    // 添加到设备列表
    gtx312l_devices_.push_back(device);
    
    // 初始化设备映射
    config->device_mappings[config->device_count].device_addr = device_addr;
    // 默认灵敏度已在构造函数中设置
    
    config->device_count++;
    return true;
}

// 注销GTX312L设备
void InputManager::unregisterGTX312L(GTX312L* device) {
    if (!device) return;
    
    uint16_t device_addr = device->get_physical_device_address().get_device_mask();
    
    // 从设备列表中移除
    auto it = std::find(gtx312l_devices_.begin(), gtx312l_devices_.end(), device);
    if (it != gtx312l_devices_.end()) {
        gtx312l_devices_.erase(it);
        
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

// 获取工作模式


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
    } else {
        // 根据工作模式处理触摸数据
        InputWorkMode work_mode = getWorkMode();
        if (work_mode == InputWorkMode::SERIAL_MODE) {
            processSerialMode();
        }
    }
}

// CPU1核心循环 - 键盘处理和HID发送
void InputManager::loop1() {
    // 1. 从共享内存获取loop0传递的触摸键盘bitmap（CPU0写入，CPU1只读）
    touch_bitmap_cache_.bitmap = shared_keyboard_data_.touch_keyboard_bitmap.bitmap;
    
    // 2. 采样GPIO键盘数据到独立bitmap（CPU1内部处理，无跨核竞态）
    gpio_keyboard_bitmap_.clear();
    updateGPIOStates();
    processGPIOKeyboard();
    
    // 3. 合并触摸和GPIO键盘bitmap（CPU1内部操作，线程安全）
    gpio_keyboard_bitmap_.bitmap |= touch_bitmap_cache_.bitmap;
    
    // 4. 根据最终bitmap发送键盘状态
    for (const auto& key : supported_keys) {
        if (gpio_keyboard_bitmap_.getKey(key)) {
            hid_->press_key(key);
            continue;
        }
        hid_->release_key(key);
    }
    
    // 5. 在HID模式下发送触摸数据
    if (getWorkMode() == InputWorkMode::HID_MODE) {
        sendHIDTouchData();
    }
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
        
        // 清除UI绑定状态
        if (ui_manager_) {
            ui_manager_->clear_binding_status();
        }
        
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
    
    // 查找设备
    GTX312L* device = nullptr;
    for (auto* dev : gtx312l_devices_) {
        if (dev->get_physical_device_address().get_device_mask() == auto_adjust_context_.device_addr) {
            device = dev;
            break;
        }
    }
    
    if (!device) {
        // 设备未找到，结束状态机
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
    }
}

// 设置HID映射
void InputManager::setHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y) {
    if (channel >= 12) return;
    
    GTX312L_DeviceMapping* mapping = findDeviceMapping(device_addr);
    if (mapping) {
        mapping->hid_area[channel] = TouchAxis(x, y);
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

// 启用所有通道
void InputManager::enableAllChannels() {
    for (auto* device : gtx312l_devices_) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            device->set_channel_enable(ch, true);
        }
    }
}

// 仅启用已映射的通道
void InputManager::enableMappedChannels() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    InputWorkMode work_mode = getWorkMode();
    
    for (int i = 0; i < config->device_count; i++) {
        auto* device = gtx312l_devices_[i];
        auto& mapping = config->device_mappings[i];
        
        for (uint8_t ch = 0; ch < 12; ch++) {
            bool enabled = false;
            
            if (work_mode == InputWorkMode::SERIAL_MODE) {
                enabled = (mapping.serial_area[ch] != MAI2_NO_USED);
            } else if (work_mode == InputWorkMode::HID_MODE) {
                enabled = (mapping.hid_area[ch].x != 0.0f || mapping.hid_area[ch].y != 0.0f);
            }
            
            device->set_channel_enable(ch, enabled);
        }
    }
}

// 更新触摸状态
inline void InputManager::updateTouchStates() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    millis_t current_time = to_ms_since_boot(get_absolute_time());
    
    for (int i = 0; i < config->device_count; i++) {
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

// 处理Serial模式
inline void InputManager::processSerialMode() {
    
    Mai2Serial_TouchState touch_state;
    touch_state.parts.state1 = 0;
    touch_state.parts.state2 = 0;
    
    // 使用静态触摸键盘bitmap，不清空，直接覆盖更新
    static KeyboardBitmap touch_keyboard_bitmap;
    
    // 遍历所有设备和通道，基于设备地址掩码进行映射
    for (int i = 0; i < config_->device_count; i++) {
        uint16_t current = current_touch_states_[i].physical_addr.mask;
        uint16_t previous = previous_touch_states_[i].physical_addr.mask;
        
        // 即使没有变化也要发送，这与HID不同
        
        // 根据设备地址掩码查找对应的映射配置
        uint16_t device_addr = current_touch_states_[i].physical_addr.mask & 0xF000; // 获取设备地址
        
        // 查找匹配的设备映射
        GTX312L_DeviceMapping* mapping = nullptr;
        for (int j = 0; j < config_->device_count; j++) {
            if ((config_->device_mappings[j].device_addr & 0xF000) == device_addr) {
                mapping = &config_->device_mappings[j];
                break;
            }
        }
        
        if (!mapping) continue; // 未找到匹配的映射
        
        for (uint8_t ch = 0; ch < 12; ch++) {
            // 处理所有通道，不跳过无变化的通道
            bool touched = (current & (1 << ch)) != 0;
            
            // 处理Serial触摸映射
            Mai2_TouchArea area = mapping->serial_area[ch];
            if (area != MAI2_NO_USED) {
                // 设置对应区域的状态
                if (area >= 1 && area <= 34) {
                    uint8_t bit_index = area - 1;
                    if (bit_index < 32) {
                        if (touched) {
                            touch_state.parts.state1 |= (1UL << bit_index);
                        } else {
                            touch_state.parts.state1 &= ~(1UL << bit_index);
                        }
                    } else {
                        bit_index -= 32;
                        if (touched) {
                            touch_state.parts.state2 |= (1 << bit_index);
                        } else {
                            touch_state.parts.state2 &= ~(1 << bit_index);
                        }
                    }
                }
            }
            
            // 处理触摸键盘映射
            HID_KeyCode key = mapping->keyboard_keys[ch];
            if (key != HID_KeyCode::KEY_NONE) {
                if (touched) {
                    touch_keyboard_bitmap.setKey(key, true);
                } else {
                    touch_keyboard_bitmap.setKey(key, false);
                }
            }
        }
    }
    
    // 将触摸键盘bitmap写入共享内存传递给loop1
    shared_keyboard_data_.touch_keyboard_bitmap.bitmap = touch_keyboard_bitmap.bitmap;
    
    // Serial模式下：只发送Serial数据，HID触摸发送直接失效
    if (mai2_serial_) {
        mai2_serial_->send_touch_state(touch_state.parts.state1, touch_state.parts.state2);
    }
    
    // HID键盘数据处理移到loop1中进行
}

// 处理HID模式
// 统一键盘处理函数实现
inline void InputManager::collectKeysFromTouch() {
    // 从触摸映射收集按键状态到键盘触发表
    for (int device_index = 0; device_index < 8; device_index++) {
        const GTX312L_SampleResult& current_state = current_touch_states_[device_index];
        const GTX312L_SampleResult& previous_state = previous_touch_states_[device_index];
        
        // 查找对应的设备映射
        GTX312L_DeviceMapping* device_mapping = nullptr;
        for (int i = 0; i < config_->device_count; i++) {
            if (config_->device_mappings[i].device_addr == current_state.physical_addr.get_device_mask()) {
                device_mapping = &config_->device_mappings[i];
                break;
            }
        }
        
        if (device_mapping) {
            // 遍历所有通道
            for (int channel = 0; channel < 12; channel++) {
                bool current_touched = (current_state.physical_addr.mask & (1 << channel)) != 0;
                bool previous_touched = (previous_state.physical_addr.mask & (1 << channel)) != 0;
                
                // 检查通道状态是否发生变化
                if (current_touched != previous_touched) {
                    HID_KeyCode key = device_mapping->keyboard_keys[channel];
                    if (key != HID_KeyCode::KEY_NONE) {
                        shared_keyboard_data_.touch_keyboard_bitmap.setKey(key, current_touched);
                    }
                }
            }
        }
    }
}

inline void InputManager::collectKeysFromGPIO() {
    // 从GPIO物理键盘收集按键状态到键盘触发表
    for (const auto& mapping : config_->physical_keyboard_mappings) {
        bool current_pressed = false;
        uint8_t gpio_pin = mapping.gpio;
        
        // 检查GPIO状态
        if (gpio_pin < 64) {  // MCU GPIO
            current_pressed = (mcu_gpio_states_ & (1ULL << gpio_pin)) != 0;
        } else if (gpio_pin >= 64 && gpio_pin < 80) {  // MCP23S17 GPIO
            int mcp_pin = gpio_pin - 64;
            // 组合port_a和port_b为16位状态
            uint16_t full_state = (static_cast<uint16_t>(mcp_gpio_states_.port_b) << 8) | mcp_gpio_states_.port_a;
            current_pressed = (full_state & (1 << mcp_pin)) != 0;
        }
        
        // 简单的按键状态判断（假设高电平有效）
        bool key_active = current_pressed;
        
        // 设置默认按键状态
        if (mapping.default_key != HID_KeyCode::KEY_NONE) {
            gpio_keyboard_bitmap_.setKey(mapping.default_key, key_active);
        }
        
        // 设置逻辑映射按键状态
        for (const auto& logical_mapping : config_->logical_key_mappings) {
            if (logical_mapping.gpio_id == gpio_pin) {
                for (int i = 0; i < logical_mapping.key_count && i < 3; i++) {
                    if (logical_mapping.keys[i] != HID_KeyCode::KEY_NONE) {
                        gpio_keyboard_bitmap_.setKey(logical_mapping.keys[i], key_active);
                    }
                }
            }
        }
    }
}

// 发送HID触摸数据
inline void InputManager::sendHIDTouchData() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    
    HID_TouchReport touch_report = {};
    
    uint8_t point_count = 0;
    
    // 遍历所有设备和通道，基于设备地址掩码进行映射
    for (int i = 0; i < config->device_count && point_count < 10; i++) {
        uint16_t current = current_touch_states_[i].physical_addr.mask;
        
        // 根据设备地址掩码查找对应的映射配置
        uint16_t device_addr = current_touch_states_[i].physical_addr.mask & 0xF000; // 获取设备地址
        
        // 查找匹配的设备映射
        GTX312L_DeviceMapping* mapping = nullptr;
        for (int j = 0; j < config->device_count; j++) {
            if ((config->device_mappings[j].device_addr & 0xF000) == device_addr) {
                mapping = &config->device_mappings[j];
                break;
            }
        }
        
        if (!mapping) continue; // 未找到匹配的映射
        
        for (uint8_t ch = 0; ch < 12 && point_count < 10; ch++) {
            if (!(current & (1 << ch))) continue; // 该通道未触摸
            
            TouchAxis axis = mapping->hid_area[ch];
            if (axis.x == 0.0f && axis.y == 0.0f) continue; // 未映射
            
            // 添加触摸点
            touch_report.contacts[point_count].x = (uint16_t)(axis.x * 65535.0f);
            touch_report.contacts[point_count].y = (uint16_t)(axis.y * 65535.0f);
            touch_report.contacts[point_count].tip_switch = 1;
            touch_report.contacts[point_count].contact_id = point_count;
            point_count++;
        }
    }
    
    touch_report.contact_count = point_count;
    
    // 发送HID报告
    if (hid_) {
        hid_->send_touch_report(touch_report);
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

// 纯公开函数实现
InputManager_PrivateConfig* inputmanager_get_config_holder() {
    return &static_config_;
}

bool inputmanager_load_config_from_manager() {
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    // 从ConfigManager加载配置项到静态配置
    bool success = true;
    
    // 加载工作模式
    uint8_t mode_value = 0;
    if (config_mgr->get_uint8(INPUTMANAGER_WORK_MODE, mode_value)) {
        static_config_.work_mode = static_cast<InputWorkMode>(mode_value);
    }
    
    // 加载触摸键盘映射配置
    uint8_t touch_enabled = 0;
    if (config_mgr->get_uint8("input_manager_touch_keyboard_enabled", touch_enabled)) {
        static_config_.touch_keyboard_enabled = (touch_enabled != 0);
    }
    
    uint8_t touch_mode = 0;
    if (config_mgr->get_uint8("input_manager_touch_keyboard_mode", touch_mode)) {
        static_config_.touch_keyboard_mode = static_cast<TouchKeyboardMode>(touch_mode);
    }
    
    // 加载设备映射
    std::vector<uint8_t> config_data;
    if (config_mgr->get_binary(INPUTMANAGER_GTX312L_DEVICES, config_data)) {
        if (config_data.size() == sizeof(InputManager_PrivateConfig)) {
            static_config_ = *reinterpret_cast<const InputManager_PrivateConfig*>(config_data.data());
        } else {
            success = false;
        }
    }
    
    return success;
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

InputManager_PrivateConfig inputmanager_get_config_copy() {
    return static_config_;
}

bool inputmanager_write_config_to_manager(const InputManager_PrivateConfig& config) {
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    // 将配置写入ConfigManager
    bool success = true;
    
    success &= config_mgr->set_uint8(INPUTMANAGER_WORK_MODE, static_cast<uint8_t>(config.work_mode));
    
    // 保存触摸键盘映射配置
    success &= config_mgr->set_uint8("input_manager_touch_keyboard_enabled", config.touch_keyboard_enabled ? 1 : 0);
    success &= config_mgr->set_uint8("input_manager_touch_keyboard_mode", static_cast<uint8_t>(config.touch_keyboard_mode));
    
    std::vector<uint8_t> config_data(sizeof(InputManager_PrivateConfig));
    std::memcpy(config_data.data(), &config, sizeof(InputManager_PrivateConfig));
    success &= config_mgr->set_binary(INPUTMANAGER_GTX312L_DEVICES, config_data);
    
    if (success) {
        success &= config_mgr->save_all_configs();
        // 更新静态配置
        static_config_ = config;
    }
    
    return success;
}

// Serial绑定顺序定义


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
                     ui_manager_->show_binding_status(message, false);
                     ui_manager_->update_binding_progress(current_binding_index_ + 1, "Serial绑定进行中");
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
                                mapping->CH_available[ch] = 0;
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
                
                // 清除UI绑定状态
                if (ui_manager_) {
                    ui_manager_->clear_binding_status();
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
                
                // 清除UI绑定状态
                if (ui_manager_) {
                    ui_manager_->clear_binding_status();
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
                // 构造HID触摸报告
                HID_TouchReport touch_report;
                touch_report.contact_count = 1;
                touch_report.contacts[0].contact_id = 1;
                touch_report.contacts[0].tip_switch = 1;
                touch_report.contacts[0].x = (uint16_t)(hid_binding_x_ * 32767);
                touch_report.contacts[0].y = (uint16_t)(hid_binding_y_ * 32767);
                
                // 发送HID报告
                hid_->send_touch_report(touch_report);
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
                
                // 清除UI绑定状态
                if (ui_manager_) {
                    ui_manager_->clear_binding_status();
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
            original_channels_backup_[i][ch] = config->device_mappings[i].CH_available[ch];
        }
    }
}

// 恢复通道状态
void InputManager::restoreChannelStates() {
    InputManager_PrivateConfig* config = inputmanager_get_config_holder();
    for (uint8_t i = 0; i < config->device_count && i < 8; i++) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            config->device_mappings[i].CH_available[ch] = original_channels_backup_[i][ch];
            
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
    // 保存上一次状态
    mcu_gpio_previous_states_ = mcu_gpio_states_;
    mcp_gpio_previous_states_ = mcp_gpio_states_;
    
    // 读取MCU GPIO状态 - 读取所有GPIO引脚状态
    mcu_gpio_states_ = 0;
    for (uint8_t pin = 0; pin < 30; pin++) {
        if (gpio_get(pin)) {
            mcu_gpio_states_ |= (1ULL << pin);
        }
    }
    
    // 读取MCP23S17 GPIO状态
    if (mcp23s17_available_ && mcp23s17_) {
        mcp23s17_->read_all_gpio(mcp_gpio_states_);
    }
}

void InputManager::processGPIOKeyboard() {
    // 处理物理键盘映射，将状态映射到gpio_keyboard_bitmap_
    for (const auto& mapping : config_->physical_keyboard_mappings) {
        uint8_t gpio_pin = mapping.gpio;
        bool current_state = false;
        
        if (is_mcu_gpio(gpio_pin)) {
            uint8_t pin_num = get_gpio_pin_number(gpio_pin);
            current_state = !(mcu_gpio_states_ & (1ULL << pin_num)); // 低电平有效
        } else if (is_mcp_gpio(gpio_pin)) {
            uint8_t pin_num = get_gpio_pin_number(gpio_pin);
            if (pin_num <= 8) { // PORTA
                current_state = !(mcp_gpio_states_.port_a & (1 << (pin_num - 1)));
            } else { // PORTB
                current_state = !(mcp_gpio_states_.port_b & (1 << (pin_num - 9)));
            }
        }
        
        // 将GPIO键盘状态映射到gpio_keyboard_bitmap_（CPU1内部处理，避免跨核竞态）
        if (current_state) {
            // 按键按下 - 设置默认按键和逻辑映射按键到bitmap
            if (mapping.default_key != HID_KeyCode::KEY_NONE) {
                gpio_keyboard_bitmap_.setKey(mapping.default_key, true);
            }
            setLogicalKeysInBitmap(gpio_pin, true, gpio_keyboard_bitmap_);
        }
    }
}

bool InputManager::readMCUGPIO(uint8_t pin, bool& value) {
    if (pin >= 30) { // RP2040有30个GPIO引脚
        return false;
    }
    
    value = gpio_get(pin);
    return true;
}

bool InputManager::readMCPGPIO(uint8_t pin, bool& value) {
    if (!mcp23s17_available_ || pin >= 16) { // MCP23S17有16个GPIO引脚
        return false;
    }
    
    // 将pin转换为port和pin_num
    MCP23S17_Port port;
    uint8_t pin_num;
    
    if (pin < 8) {
        port = MCP23S17_PORT_A;
        pin_num = pin;
    } else {
        port = MCP23S17_PORT_B;
        pin_num = pin - 8;
    }
    
    return mcp23s17_->read_pin(port, pin_num, value);
}

void InputManager::setLogicalKeysInBitmap(uint8_t gpio_pin, bool pressed, KeyboardBitmap& bitmap) {
    // 查找该GPIO的逻辑按键映射
    for (const auto& mapping : config_->logical_key_mappings) {
        if (mapping.gpio_id == gpio_pin) {
            for (int i = 0; i < 3; i++) {
                if (mapping.keys[i] != HID_KeyCode::KEY_NONE) {
                    bitmap.setKey(mapping.keys[i], pressed);
                }
            }
            break;
        }
    }
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
    if (hid_) {
        HID_Config config;
        if (hid_->get_config(config)) {
            // 返回回报间隔的倒数作为回报速率 (Hz)
            if (config.report_interval_ms > 0) {
                return 1000 / config.report_interval_ms;
            }
        }
    }
    return 0; // HID未初始化或配置无效
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