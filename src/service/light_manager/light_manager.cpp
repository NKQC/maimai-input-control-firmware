#include "light_manager.h"
#include "../config_manager/config_manager.h"
#include "../../hal/pio/hal_pio.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <cstring>
#include <algorithm>
#include <cstdio>

// 前向声明 - 避免循环依赖
class UIManager;

// 静态实例
LightManager* LightManager::instance_ = nullptr;

// 虚拟EEPROM数据 (模拟BD15070_4.h中的dummyEEPRom)
static uint8_t virtual_eeprom_[256] = {0};

// ============================================================================
// 配置管理函数实现
// ============================================================================

// [默认配置注册函数] 遵循服务层规则3 - 注册默认配置到ConfigManager，包含范围检查
void lightmanager_register_default_configs(config_map_t& default_map) {
    // 注册LightManager默认配置，包含合理的范围限制
    default_map[LIGHTMANAGER_ENABLE] = ConfigValue(true);  // 默认启用灯光
    default_map[LIGHTMANAGER_UART_DEVICE] = ConfigValue(std::string("uart1"));  // 默认UART设备
    default_map[LIGHTMANAGER_BAUD_RATE] = ConfigValue((uint32_t)115200, (uint32_t)9600, (uint32_t)1000000);  // 波特率范围: 9600-1000000
    default_map[LIGHTMANAGER_NODE_ID] = ConfigValue((uint8_t)1, (uint8_t)1, (uint8_t)255);  // 节点ID范围: 1-255
    default_map[LIGHTMANAGER_NEOPIXEL_COUNT] = ConfigValue((uint16_t)128, (uint16_t)1, (uint16_t)1024);  // LED数量范围: 1-1024
    default_map[LIGHTMANAGER_NEOPIXEL_PIN] = ConfigValue((uint8_t)16, (uint8_t)0, (uint8_t)29);  // 引脚范围: 0-29 (RP2040)
    default_map[LIGHTMANAGER_REGION_MAPPINGS] = ConfigValue(std::string(""));  // 默认区域映射
}

// [配置保管函数] 遵循服务层规则3 - 保存静态私有配置变量并返回指针
LightManager_Config* lightmanager_get_config_holder() {
    static LightManager_Config config;
    return &config;
}

// [配置加载函数] 遵循服务层规则3 - 从ConfigManager获取配置存入指针
bool lightmanager_load_config_from_manager(LightManager_Config* config) {
    if (!config || !config->config_manager) {
        return false;
    }
    
    // 这个函数设置服务指针，实际配置数据通过lightmanager_get_config_copy获取
    // 遵循架构：服务本身完全不保存配置，完全由外部公共公开函数处理
    return true;
}

// [配置读取函数] 遵循服务层规则3 - 复制配置副本并返回，使用预处理键
LightManager_PrivateConfig lightmanager_get_config_copy() {
    LightManager_PrivateConfig private_config;  // 每次创建新副本，不使用static
    
    // 从ConfigManager获取配置，使用预处理键，遵循服务层规则2
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (config_mgr && config_mgr->has_key(LIGHTMANAGER_ENABLE)) {
        private_config.enable = config_mgr->get_bool(LIGHTMANAGER_ENABLE);
        private_config.uart_device = config_mgr->get_string(LIGHTMANAGER_UART_DEVICE);
        private_config.baud_rate = config_mgr->get_uint32(LIGHTMANAGER_BAUD_RATE);
        private_config.node_id = config_mgr->get_uint8(LIGHTMANAGER_NODE_ID);
        private_config.neopixel_count = config_mgr->get_uint16(LIGHTMANAGER_NEOPIXEL_COUNT);
        private_config.neopixel_pin = config_mgr->get_uint8(LIGHTMANAGER_NEOPIXEL_PIN);
    }
    
    return private_config;  // 返回配置副本
}

// [配置写入函数] 遵循服务层规则3 - 将参数传回ConfigManager，使用预处理键
bool lightmanager_write_config_to_manager(const LightManager_PrivateConfig& config) {
    // 写入配置到ConfigManager，使用预处理键，遵循服务层规则2
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (config_mgr) {
        config_mgr->set_bool(LIGHTMANAGER_ENABLE, config.enable);
        config_mgr->set_string(LIGHTMANAGER_UART_DEVICE, config.uart_device);
        config_mgr->set_uint32(LIGHTMANAGER_BAUD_RATE, config.baud_rate);
        config_mgr->set_uint8(LIGHTMANAGER_NODE_ID, config.node_id);
        config_mgr->set_uint16(LIGHTMANAGER_NEOPIXEL_COUNT, config.neopixel_count);
        config_mgr->set_uint8(LIGHTMANAGER_NEOPIXEL_PIN, config.neopixel_pin);
        
        // 保存配置到Flash，ConfigManager内置数据类型天然检查边界
        return config_mgr->save_config();
    }
    return false;
}

// ============================================================================
// LightManager类实现
// ============================================================================

// 遵循服务层规则3: 服务本身完全不保存配置
LightManager::LightManager() 
    : initialized_(false)
    , debug_enabled_(false)
    , uart_hal_(nullptr)
    , neopixel_(nullptr)
    , rx_buffer_pos_(0)
    , escape_next_(false)
    , command_callback_(nullptr) {
    
    // 初始化接收缓冲区
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    
    // 初始化渐变状态
    fade_state_.active = false;
    fade_state_.start_time = 0;
    fade_state_.end_time = 0;
    fade_state_.start_led = 0;
    fade_state_.end_led = 0;
    fade_state_.start_color = NeoPixel_Color(0, 0, 0);
    fade_state_.end_color = NeoPixel_Color(0, 0, 0);
}

LightManager::~LightManager() {
    deinit();
}

LightManager* LightManager::getInstance() {
    if (!instance_) {
        instance_ = new LightManager();
    }
    return instance_;
}

bool LightManager::init() {
    if (initialized_) {
        return true;
    }
    
    log_debug("Initializing LightManager...");
    
    // 加载配置 - 遵循服务层规则3: 外部获取配置应当使用[配置读取函数]
    LightManager_PrivateConfig config = lightmanager_get_config_copy();
    if (!config.enable) {
        log_debug("LightManager disabled in configuration");
        return false;
    }
    
    // node_id 现在通过配置函数获取，不再存储在类内部
    
    // 初始化UART设备
    // 注意：这里需要HAL_UART的实际实现
    // uart_hal_ = HAL_UART::getInstance(config.uart_device.c_str());
    // if (uart_hal_) {
    //     if (!uart_hal_->init(config.baud_rate)) {
    //         log_error("Failed to initialize UART device");
    //         return false;
    //     }
    // } else {
    //     log_error("Failed to get UART device instance");
    //     return false;
    // }
    
    // 初始化NeoPixel设备
    // 注意：这里需要HAL_PIO的实际实现
    // HAL_PIO* pio_hal = HAL_PIO::getInstance();
    // if (pio_hal) {
    //     neopixel_ = new NeoPixel(pio_hal, config.neopixel_pin, config.neopixel_count);
    //     if (neopixel_) {
    //         if (!neopixel_->init()) {
    //             log_error("Failed to initialize NeoPixel");
    //             delete neopixel_;
    //             neopixel_ = nullptr;
    //             return false;
    //         }
    //     } else {
    //         log_error("Failed to create NeoPixel instance");
    //         return false;
    //     }
    // } else {
    //     log_error("Failed to get PIO HAL instance");
    //     return false;
    // }
    
    // 加载区域映射配置
    initialized_ = true; // 需要先设置为true才能调用load_region_mappings
    if (!load_region_mappings()) {
        log_error("Failed to load region mappings, using defaults");
        reset_region_mappings();
    }
    
    log_debug("LightManager initialized successfully");
    return true;
}

void LightManager::deinit() {
    if (!initialized_) {
        return;
    }
    
    log_debug("Deinitializing LightManager...");
    
    // 清理NeoPixel
    if (neopixel_) {
        neopixel_->clear_all();
        neopixel_->show();
        neopixel_->deinit();
        delete neopixel_;
        neopixel_ = nullptr;
    }
    
    // 清理UART
    if (uart_hal_) {
        uart_hal_->deinit();
        uart_hal_ = nullptr;
    }
    
    // 清理区域映射
    region_mappings_.clear();
    
    // 重置状态
    rx_buffer_pos_ = 0;
    escape_next_ = false;
    fade_state_.active = false;
    
    initialized_ = false;
    log_debug("LightManager deinitialized");
}

bool LightManager::is_ready() const {
    return initialized_ && uart_hal_ && neopixel_;
}

// ============================================================================
// 区域映射管理
// ============================================================================

bool LightManager::add_region_mapping(const LightRegionMapping& mapping) {
    if (!initialized_) {
        log_error("add_region_mapping: LightManager not initialized");
        return false;
    }
    
    // 验证映射参数
    if (mapping.name.empty()) {
        log_error("add_region_mapping: empty region name");
        return false;
    }
    
    if (mapping.mai2light_start_index > mapping.mai2light_end_index) {
        log_error("add_region_mapping: invalid range (" + std::to_string(mapping.mai2light_start_index) + " > " + std::to_string(mapping.mai2light_end_index) + ")");
        return false;
    }
    
    if (mapping.mai2light_end_index >= 32) {
        log_error("add_region_mapping: end index out of range (" + std::to_string(mapping.mai2light_end_index) + " >= 32)");
        return false;
    }
    
    // 检查是否已存在同名映射
    auto it = std::find_if(region_mappings_.begin(), region_mappings_.end(),
        [&mapping](const LightRegionMapping& existing) {
            return existing.name == mapping.name;
        });
    
    if (it != region_mappings_.end()) {
        log_error("Region mapping already exists: " + mapping.name);
        return false;
    }
    
    region_mappings_.push_back(mapping);
    log_debug("Added region mapping: " + mapping.name + " (range: " + std::to_string(mapping.mai2light_start_index) + "-" + std::to_string(mapping.mai2light_end_index) + ")");
    return true;
}

bool LightManager::remove_region_mapping(const std::string& name) {
    if (!initialized_) {
        return false;
    }
    
    auto it = std::find_if(region_mappings_.begin(), region_mappings_.end(),
        [&name](const LightRegionMapping& mapping) {
            return mapping.name == name;
        });
    
    if (it != region_mappings_.end()) {
        region_mappings_.erase(it);
        log_debug("Removed region mapping: " + name);
        return true;
    }
    
    log_error("Region mapping not found: " + name);
    return false;
}

bool LightManager::get_region_mapping(const std::string& name, LightRegionMapping& mapping) const {
    auto it = std::find_if(region_mappings_.begin(), region_mappings_.end(),
        [&name](const LightRegionMapping& existing) {
            return existing.name == name;
        });
    
    if (it != region_mappings_.end()) {
        mapping = *it;
        return true;
    }
    
    return false;
}

std::vector<std::string> LightManager::get_region_names() const {
    std::vector<std::string> names;
    for (const auto& mapping : region_mappings_) {
        names.push_back(mapping.name);
    }
    return names;
}

bool LightManager::enable_region_mapping(const std::string& name, bool enabled) {
    auto it = std::find_if(region_mappings_.begin(), region_mappings_.end(),
        [&name](LightRegionMapping& mapping) {
            return mapping.name == name;
        });
    
    if (it != region_mappings_.end()) {
        it->enabled = enabled;
        log_debug("Region mapping " + name + (enabled ? " enabled" : " disabled"));
        return true;
    }
    
    return false;
}

bool LightManager::save_region_mappings() {
    if (!initialized_) {
        return false;
    }
    
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    // 序列化区域映射数据
    std::vector<uint8_t> mapping_data;
    
    // 写入映射数量
    uint16_t mapping_count = static_cast<uint16_t>(region_mappings_.size());
    mapping_data.push_back(mapping_count & 0xFF);
    mapping_data.push_back((mapping_count >> 8) & 0xFF);
    
    // 写入每个映射
    for (const auto& mapping : region_mappings_) {
        // 名称长度和名称
        uint8_t name_len = static_cast<uint8_t>(std::min(mapping.name.length(), size_t(255)));
        mapping_data.push_back(name_len);
        for (size_t i = 0; i < name_len; ++i) {
            mapping_data.push_back(static_cast<uint8_t>(mapping.name[i]));
        }
        
        // 映射参数
        mapping_data.push_back(mapping.mai2light_start_index);
        mapping_data.push_back(mapping.mai2light_end_index);
        
        // Neopixel bitmap (4字节)
        mapping_data.push_back(mapping.neopixel_bitmap & 0xFF);
        mapping_data.push_back((mapping.neopixel_bitmap >> 8) & 0xFF);
        mapping_data.push_back((mapping.neopixel_bitmap >> 16) & 0xFF);
        mapping_data.push_back((mapping.neopixel_bitmap >> 24) & 0xFF);
        
        // 启用状态
        mapping_data.push_back(mapping.enabled ? 1 : 0);
    }
    
    // 保存到ConfigManager
    config_mgr->set_string("LIGHTMANAGER_REGION_MAPPINGS", std::string(mapping_data.begin(), mapping_data.end()));
    return config_mgr->save_config();
}

bool LightManager::load_region_mappings() {
    if (!initialized_) {
        return false;
    }
    
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (!config_mgr) {
        return false;
    }
    
    std::vector<uint8_t> mapping_data;
    std::string mapping_str = config_mgr->get_string("LIGHTMANAGER_REGION_MAPPINGS");
    if (mapping_str.empty()) {
        // 如果没有保存的映射数据，使用默认映射
        reset_region_mappings();
        return true;
    }
    
    // 转换字符串为字节数组
    mapping_data.assign(mapping_str.begin(), mapping_str.end());
    
    if (mapping_data.size() < 2) {
        log_error("Invalid region mapping data size");
        return false;
    }
    
    // 清空现有映射
    region_mappings_.clear();
    
    // 读取映射数量
    uint16_t mapping_count = mapping_data[0] | (mapping_data[1] << 8);
    size_t offset = 2;
    
    for (uint16_t i = 0; i < mapping_count && offset < mapping_data.size(); ++i) {
        LightRegionMapping mapping;
        
        // 读取名称
        if (offset >= mapping_data.size()) break;
        uint8_t name_len = mapping_data[offset++];
        
        if (offset + name_len > mapping_data.size()) break;
        mapping.name.reserve(name_len);
        for (uint8_t j = 0; j < name_len; ++j) {
            mapping.name += static_cast<char>(mapping_data[offset++]);
        }
        
        // 读取映射参数
        if (offset + 6 > mapping_data.size()) break;
        mapping.mai2light_start_index = mapping_data[offset++];
        mapping.mai2light_end_index = mapping_data[offset++];
        
        // 读取Neopixel bitmap
        mapping.neopixel_bitmap = mapping_data[offset] |
                                 (mapping_data[offset + 1] << 8) |
                                 (mapping_data[offset + 2] << 16) |
                                 (mapping_data[offset + 3] << 24);
        offset += 4;
        
        // 读取启用状态
        mapping.enabled = (mapping_data[offset++] != 0);
        
        region_mappings_.push_back(mapping);
    }
    
    log_debug("Loaded " + std::to_string(region_mappings_.size()) + " region mappings");
    return true;
}

bool LightManager::reset_region_mappings() {
    if (!initialized_) {
        return false;
    }
    
    // 清空现有映射
    region_mappings_.clear();
    
    // 添加按钮区域映射
    LightRegionMapping button_mapping;
    button_mapping.name = "button_area";
    button_mapping.mai2light_start_index = 0;
    button_mapping.mai2light_end_index = 10;
    button_mapping.neopixel_bitmap = 0x000007FF; // 前11个LED (0-10)
    button_mapping.enabled = true;
    region_mappings_.push_back(button_mapping);
    
    // 添加框体灯映射
    LightRegionMapping body_mapping;
    body_mapping.name = "body_lights";
    body_mapping.mai2light_start_index = 8;
    body_mapping.mai2light_end_index = 10;
    body_mapping.neopixel_bitmap = 0x00000700; // LED 8, 9, 10
    body_mapping.enabled = true;
    region_mappings_.push_back(body_mapping);
    
    // 添加全局映射（用于测试）
    LightRegionMapping global_mapping;
    global_mapping.name = "global";
    global_mapping.mai2light_start_index = 0;
    global_mapping.mai2light_end_index = 31;
    global_mapping.neopixel_bitmap = 0xFFFFFFFF; // 映射到前32个Neopixel
    global_mapping.enabled = false; // 默认禁用
    region_mappings_.push_back(global_mapping);
    
    log_debug("Reset to default region mappings");
    return save_region_mappings();
}

// ============================================================================
// 手动触发映射接口
// ============================================================================

bool LightManager::trigger_region_mapping(const std::string& region_name, uint8_t r, uint8_t g, uint8_t b) {
    if (!is_ready()) {
        return false;
    }
    
    LightRegionMapping mapping;
    if (!get_region_mapping(region_name, mapping) || !mapping.enabled) {
        return false;
    }
    
    // 根据bitmap设置对应的NeoPixel
    bitmap32_t bitmap = mapping.neopixel_bitmap;
    for (uint8_t i = 0; i < 32; i++) {
        if (bitmap & (1U << i)) {
            if (neopixel_) {
                neopixel_->set_pixel(i, r, g, b);
            }
        }
    }
    
    if (neopixel_) {
        neopixel_->show();
    }
    
    log_debug("Triggered region mapping: " + region_name);
    return true;
}

// ============================================================================
// Loop接口 - 处理Mai2Light回调
// ============================================================================

void LightManager::loop() {
    if (!is_ready()) {
        return;
    }
    
    // 处理接收到的数据
    process_received_data();
    
    // 更新渐变效果
    update_fade_effects();
}

// ============================================================================
// 回调函数设置
// ============================================================================

void LightManager::set_command_callback(LightManager_CommandCallback callback) {
    command_callback_ = callback;
}

// ============================================================================
// 调试功能
// ============================================================================

void LightManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
    
    if (enabled) {
        log_debug("LightManager debug output enabled");
        log_debug(get_debug_info());
    } else {
        log_debug("LightManager debug output disabled");
    }
}

std::string LightManager::get_debug_info() const {
    std::string info = "=== LightManager Debug Info ===\n";
    
    // 基本状态
    info += "Initialized: " + std::string(initialized_ ? "Yes" : "No") + "\n";
    // 通过配置函数获取node_id，遵循服务层规则3
    LightManager_PrivateConfig config = lightmanager_get_config_copy();
    info += "Node ID: " + std::to_string(config.node_id) + "\n";
    
    // 配置信息
    info += "Enabled: " + std::string(config.enable ? "Yes" : "No") + "\n";
    info += "Baud Rate: " + std::to_string(config.baud_rate) + "\n";
    
    // NeoPixel状态
    if (neopixel_) {
        info += "NeoPixel: Connected (" + std::to_string(config.neopixel_count) + " LEDs on pin " + std::to_string(config.neopixel_pin) + ")\n";
    } else {
        info += "NeoPixel: Not connected\n";
    }
    
    // UART状态
    if (uart_hal_) {
        info += "UART: Connected\n";
    } else {
        info += "UART: Not connected\n";
    }
    
    // 区域映射信息
    info += "Region Mappings: " + std::to_string(region_mappings_.size()) + " total\n";
    for (const auto& mapping : region_mappings_) {
        info += "  - " + mapping.name + ": [" + std::to_string(mapping.mai2light_start_index) + "-" + 
                std::to_string(mapping.mai2light_end_index) + "] -> 0x" + 
                std::to_string(mapping.neopixel_bitmap) + (mapping.enabled ? " (enabled)" : " (disabled)") + "\n";
    }
    
    // 通信状态
    info += "RX Buffer Position: " + std::to_string(rx_buffer_pos_) + "\n";
    info += "Escape Next: " + std::string(escape_next_ ? "Yes" : "No") + "\n";
    
    // 渐变状态
    info += "Fade Active: " + std::string(fade_state_.active ? "Yes" : "No") + "\n";
    if (fade_state_.active) {
        info += "  Range: [" + std::to_string(fade_state_.start_led) + "-" + std::to_string(fade_state_.end_led) + "]\n";
        info += "  Target RGB: (" + std::to_string(fade_state_.end_color.r) + "," + 
                std::to_string(fade_state_.end_color.g) + "," + std::to_string(fade_state_.end_color.b) + ")\n";
        info += "  Start Time: " + std::to_string(fade_state_.start_time) + "\n";
        info += "  End Time: " + std::to_string(fade_state_.end_time) + "\n";
    }
    
    // 调试状态
    info += "Debug Output: " + std::string(debug_enabled_ ? "Enabled" : "Disabled") + "\n";
    info += "Command Callback: " + std::string(command_callback_ ? "Set" : "Not set") + "\n";
    
    return info;
}

// ============================================================================
// 内部方法 - 数据处理
// ============================================================================

void LightManager::process_received_data() {
    if (!uart_hal_) {
        return;
    }
    
    // 从UART读取数据
    uint8_t buffer[256];
    size_t bytes_read = uart_hal_->read_from_rx_buffer(buffer, sizeof(buffer));
    
    for (size_t i = 0; i < bytes_read; i++) {
        uint8_t byte = buffer[i];
            // 处理同步字节
            if (byte == BD15070_SYNC) {
                rx_buffer_pos_ = 0;
                escape_next_ = false;
                continue;
            }
            
            // 处理转义字节
            if (byte == BD15070_MARKER) {
                escape_next_ = true;
                continue;
            }
            
            // 处理转义
            if (escape_next_) {
                byte++;
                escape_next_ = false;
            }
            
        // 存储数据
        if (rx_buffer_pos_ < sizeof(rx_buffer_)) {
            rx_buffer_[rx_buffer_pos_++] = byte;
            
            // 检查是否接收完整数据包
            if (rx_buffer_pos_ >= 4) { // 至少需要dstNodeID, srcNodeID, length, command
                uint8_t expected_length = rx_buffer_[2] + 4; // length + header (4 bytes)
                
                if (rx_buffer_pos_ >= expected_length) {
                    // 验证校验和
                    uint8_t checksum = calculate_checksum(rx_buffer_, expected_length - 1);
                    if (checksum == rx_buffer_[expected_length - 1]) {
                        // 解析数据包
                        BD15070_PacketReq packet;
                        if (parse_packet(rx_buffer_, expected_length, packet)) {
                            handle_command(packet);
                        }
                    } else {
                        log_error("Checksum error in received packet");
                    }
                    
                    // 移除已处理的数据包
                    memmove(rx_buffer_, rx_buffer_ + expected_length, rx_buffer_pos_ - expected_length);
                    rx_buffer_pos_ -= expected_length;
                }
            }
        } else {
            // 缓冲区溢出，重置
            rx_buffer_pos_ = 0;
            escape_next_ = false;
            log_error("RX buffer overflow");
        }
    }
}

bool LightManager::parse_packet(const uint8_t* buffer, uint8_t length, BD15070_PacketReq& packet) {
    if (!buffer) {
        log_error("parse_packet: null buffer pointer");
        return false;
    }
    
    if (length < 4) {
        log_error("parse_packet: packet too short (" + std::to_string(length) + " bytes)");
        return false;
    }
    
    packet.dstNodeID = buffer[0];
    packet.srcNodeID = buffer[1];
    packet.length = buffer[2];
    packet.command = buffer[3];
    
    // 验证数据长度
    uint8_t data_length = packet.length;
    if (data_length > sizeof(packet.data)) {
        log_error("parse_packet: data length too large (" + std::to_string(data_length) + " > " + std::to_string(sizeof(packet.data)) + ")");
        return false;
    }
    
    // 验证总包长度
    if (length < (4 + data_length)) {
        log_error("parse_packet: insufficient data (expected " + std::to_string(4 + data_length) + ", got " + std::to_string(length) + ")");
        return false;
    }
    
    // 复制数据部分
    if (data_length > 0) {
        memcpy(&packet.data, buffer + 4, data_length);
    }
    
    log_debug("parse_packet: successfully parsed packet (cmd=" + std::to_string(packet.command) + ", len=" + std::to_string(data_length) + ")");
    return true;
}

void LightManager::handle_command(const BD15070_PacketReq& packet) {
    // 检查节点ID
    // 通过配置函数获取node_id，遵循服务层规则3
    LightManager_PrivateConfig config = lightmanager_get_config_copy();
    if (packet.dstNodeID != config.node_id && packet.dstNodeID != 0xFF) {
        log_debug("Ignoring packet for node " + std::to_string(packet.dstNodeID) + " (our node: " + std::to_string(config.node_id) + ")");
        return; // 不是发给我们的数据包
    }
    
    log_debug("Processing command " + std::to_string(packet.command) + " from node " + std::to_string(packet.srcNodeID));
    
    // 处理命令
    switch (static_cast<BD15070_Command>(packet.command)) {
        case SetLedGs8Bit:
            handle_set_led_gs8bit(packet);
            break;
            
        case SetLedGs8BitMulti:
            handle_set_led_gs8bit_multi(packet);
            break;
            
        case SetLedGs8BitMultiFade:
            handle_set_led_gs8bit_multi_fade(packet);
            break;
            
        case SetLedFet:
            handle_set_led_fet(packet);
            break;
            
        case SetLedGsUpdate:
            handle_set_led_gs_update(packet);
            break;
            
        case GetBoardInfo:
            handle_get_board_info(packet);
            break;
            
        case GetBoardStatus:
            handle_get_board_status(packet);
            break;
            
        case GetFirmSum:
            handle_get_firm_sum(packet);
            break;
            
        case GetProtocolVersion:
            handle_get_protocol_version(packet);
            break;
            
        case SetEEPRom:
        case GetEEPRom:
            handle_eeprom_commands(packet);
            break;
            
        case SetEnableResponse:
        case SetDisableResponse:
            // 这些命令不需要特殊处理
            log_debug("Received response control command: " + std::to_string(packet.command));
            send_ack(static_cast<BD15070_Command>(packet.command));
            break;
            
        default:
            log_error("Unknown command: 0x" + std::to_string(packet.command));
            send_ack(static_cast<BD15070_Command>(packet.command), AckStatus_Invalid, AckReport_CommandUnknown);
            break;
    }
    
    // 通知回调
    if (command_callback_) {
        command_callback_(static_cast<BD15070_Command>(packet.command), packet);
    }
}

void LightManager::send_ack(BD15070_Command command, BD15070_AckStatus status, 
                           BD15070_AckReport report, const uint8_t* data, uint8_t data_length) {
    if (!uart_hal_) {
        return;
    }
    
    BD15070_PacketAck ack;
    ack.dstNodeID = 0; // 发送给主机
    // 通过配置函数获取node_id，遵循服务层规则3
    LightManager_PrivateConfig config = lightmanager_get_config_copy();
    ack.srcNodeID = config.node_id;
    ack.length = 3 + data_length; // status + command + report + data
    ack.status = status;
    ack.command = static_cast<uint8_t>(command);
    ack.report = report;
    
    if (data && data_length > 0) {
        memcpy(&ack.data, data, std::min(data_length, (uint8_t)sizeof(ack.data)));
    }
    
    // 构建发送缓冲区
    uint8_t tx_buffer[64];
    uint8_t pos = 0;
    
    // 同步字节
    tx_buffer[pos++] = BD15070_SYNC;
    
    // 数据包内容
    uint8_t* packet_data = reinterpret_cast<uint8_t*>(&ack);
    uint8_t packet_length = 6 + data_length; // 固定头部6字节 + 数据
    
    for (uint8_t i = 0; i < packet_length; i++) {
        uint8_t byte = packet_data[i];
        if (byte == BD15070_SYNC || byte == BD15070_MARKER) {
            tx_buffer[pos++] = BD15070_MARKER;
            tx_buffer[pos++] = byte - 1;
        } else {
            tx_buffer[pos++] = byte;
        }
    }
    
    // 校验和
    uint8_t checksum = calculate_checksum(packet_data, packet_length);
    if (checksum == BD15070_SYNC || checksum == BD15070_MARKER) {
        tx_buffer[pos++] = BD15070_MARKER;
        tx_buffer[pos++] = checksum - 1;
    } else {
        tx_buffer[pos++] = checksum;
    }
    
    // 发送数据
    uart_hal_->write_to_tx_buffer(tx_buffer, pos);
    uart_hal_->trigger_tx_dma();
}

// ============================================================================
// 命令处理函数实现
// ============================================================================

void LightManager::handle_set_led_gs8bit(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for SetLedGs8Bit command");
        send_ack(SetLedGs8Bit, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    if (packet.length < 4) {
        log_error("SetLedGs8Bit: Invalid data length " + std::to_string(packet.length) + ", expected at least 4");
        send_ack(SetLedGs8Bit, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    uint8_t index = packet.data.led_gs8bit.index;
    uint8_t r = packet.data.led_gs8bit.color[0];
    uint8_t g = packet.data.led_gs8bit.color[1];
    uint8_t b = packet.data.led_gs8bit.color[2];
    
    log_debug("SetLedGs8Bit: index=" + std::to_string(index) + 
              ", RGB=(" + std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
    
    map_mai2light_to_neopixel(index, r, g, b);
    send_ack(SetLedGs8Bit);
}

void LightManager::handle_set_led_gs8bit_multi(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for SetLedGs8BitMulti command");
        send_ack(SetLedGs8BitMulti, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    if (packet.length < 5) {
        log_error("SetLedGs8BitMulti: Invalid data length " + std::to_string(packet.length) + ", expected at least 5");
        send_ack(SetLedGs8BitMulti, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    uint8_t start = packet.data.led_multi.start;
    uint8_t end = packet.data.led_multi.end;
    uint8_t r = packet.data.led_multi.Multi_color[0];
    uint8_t g = packet.data.led_multi.Multi_color[1];
    uint8_t b = packet.data.led_multi.Multi_color[2];
    
    // 处理特殊值 0x20 (SetLedDataAllOff)
    if (end == 0x20) {
        end = 11; // NUM_LEDS
    }
    
    if (start > end) {
        log_error("SetLedGs8BitMulti: Invalid range (" + std::to_string(start) + " > " + std::to_string(end) + ")");
        send_ack(SetLedGs8BitMulti, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    log_debug("SetLedGs8BitMulti: range=[" + std::to_string(start) + "-" + std::to_string(end) + "], RGB=(" + 
              std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
    
    map_range_to_neopixel(start, end - 1, r, g, b);
    fade_state_.active = false; // 停止渐变
    send_ack(SetLedGs8BitMulti);
}

void LightManager::handle_set_led_gs8bit_multi_fade(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for SetLedGs8BitMultiFade command");
        send_ack(SetLedGs8BitMultiFade, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    if (packet.length < 6) {
        log_error("SetLedGs8BitMultiFade: Invalid data length " + std::to_string(packet.length) + ", expected at least 6");
        send_ack(SetLedGs8BitMultiFade, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    uint8_t start = packet.data.led_multi.start;
    uint8_t end = packet.data.led_multi.end;
    uint8_t r = packet.data.led_multi.Multi_color[0];
    uint8_t g = packet.data.led_multi.Multi_color[1];
    uint8_t b = packet.data.led_multi.Multi_color[2];
    uint8_t speed = packet.data.led_multi.speed;
    
    if (start > end) {
        log_error("SetLedGs8BitMultiFade: Invalid range (" + std::to_string(start) + " > " + std::to_string(end) + ")");
        send_ack(SetLedGs8BitMultiFade, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    if (speed == 0) {
        log_error("SetLedGs8BitMultiFade: Invalid speed (0)");
        send_ack(SetLedGs8BitMultiFade, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    log_debug("SetLedGs8BitMultiFade: range=[" + std::to_string(start) + "-" + std::to_string(end) + "], RGB=(" + 
              std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + "), speed=" + std::to_string(speed));
    
    // 设置渐变效果
    fade_state_.active = true;
    fade_state_.start_time = 0; // 需要实际的时间函数
    fade_state_.end_time = fade_state_.start_time + (4095 / speed * 8);
    fade_state_.start_led = start;
    fade_state_.end_led = end - 1;
    fade_state_.end_color = NeoPixel_Color(r, g, b);
    
    // 获取当前颜色作为起始颜色
    if (neopixel_ && start < 32) {
        fade_state_.start_color = neopixel_->get_pixel(start);
    } else {
        fade_state_.start_color = NeoPixel_Color(0, 0, 0);
    }
    
    send_ack(SetLedGs8BitMultiFade);
}

void LightManager::handle_set_led_fet(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for SetLedFet command");
        send_ack(SetLedFet, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    if (packet.length < 3) {
        log_error("SetLedFet: Invalid data length " + std::to_string(packet.length) + ", expected at least 3");
        send_ack(SetLedFet, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    uint8_t body_led = packet.data.led_fet.BodyLed;
    uint8_t ext_led = packet.data.led_fet.ExtLed;
    uint8_t side_led = packet.data.led_fet.SideLed;
    
    log_debug("SetLedFet: BodyLed=" + std::to_string(body_led) + ", ExtLed=" + std::to_string(ext_led) + ", SideLed=" + std::to_string(side_led));
    
    // 框体灯处理 - 只有白色，值代表亮度
    if (neopixel_) {
        // LED 8: BodyLed
        uint8_t body_brightness = (body_led * 255) / 255;
        neopixel_->set_pixel(8, body_brightness, body_brightness, body_brightness);
        
        // LED 9: ExtLed (same as BodyLed)
        uint8_t ext_brightness = (ext_led * 255) / 255;
        neopixel_->set_pixel(9, ext_brightness, ext_brightness, ext_brightness);
        
        // LED 10: SideLed (00 or FF)
        uint8_t side_brightness = (side_led * 255) / 255;
        neopixel_->set_pixel(10, side_brightness, side_brightness, side_brightness);
        
        // 立即刷新
        neopixel_->show();
    } else {
        log_error("SetLedFet: NeoPixel instance is null");
        send_ack(SetLedFet, AckStatus_Invalid, AckReport_ParamError);
        return;
    }
    
    send_ack(SetLedFet);
}

void LightManager::handle_set_led_gs_update(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for SetLedGsUpdate command");
        send_ack(SetLedGsUpdate, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    log_debug("SetLedGsUpdate: Refreshing all LEDs");
    
    // 立即刷新所有LED
    if (neopixel_) {
        neopixel_->show();
        log_debug("SetLedGsUpdate: LED refresh completed");
    } else {
        log_error("SetLedGsUpdate: NeoPixel instance is null");
        send_ack(SetLedGsUpdate, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    send_ack(SetLedGsUpdate);
}

void LightManager::handle_get_board_info(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for GetBoardInfo command");
        send_ack(GetBoardInfo, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    log_debug("GetBoardInfo: Sending board information");
    uint8_t board_info[10];
    memcpy(board_info, "15070-04\xFF", 9);
    board_info[9] = 144; // firmRevision
    
    send_ack(GetBoardInfo, AckStatus_Ok, AckReport_Ok, board_info, 10);
}

void LightManager::handle_get_board_status(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for GetBoardStatus command");
        send_ack(GetBoardStatus, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    log_debug("GetBoardStatus: Sending board status");
    uint8_t status_data[4] = {0, 1, 0, 0}; // timeoutStat, timeoutSec, pwmIo, fetTimeout
    send_ack(GetBoardStatus, AckStatus_Ok, AckReport_Ok, status_data, 4);
}

void LightManager::handle_get_firm_sum(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for GetFirmSum command");
        send_ack(GetFirmSum, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    log_debug("GetFirmSum: Sending firmware checksum");
    uint8_t sum_data[2] = {0, 0}; // sum_upper, sum_lower
    send_ack(GetFirmSum, AckStatus_Ok, AckReport_Ok, sum_data, 2);
}

void LightManager::handle_get_protocol_version(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for GetProtocolVersion command");
        send_ack(GetProtocolVersion, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    log_debug("GetProtocolVersion: Sending protocol version");
    uint8_t version_data[3] = {1, 1, 1}; // appliMode, major, minor
    send_ack(GetProtocolVersion, AckStatus_Ok, AckReport_Ok, version_data, 3);
}

void LightManager::handle_eeprom_commands(const BD15070_PacketReq& packet) {
    if (!initialized_) {
        log_error("LightManager not initialized for EEPROM command");
        BD15070_Command cmd = static_cast<BD15070_Command>(packet.command);
        send_ack(cmd, AckStatus_Invalid, AckReport_Invalid);
        return;
    }
    
    if (packet.command == SetEEPRom) {
        uint8_t address = packet.data.eeprom_set.Set_adress;
        uint8_t data = packet.data.eeprom_set.writeData;
        
        log_debug("SetEEPRom: address=" + std::to_string(address) + ", data=" + std::to_string(data));
        
        if (address < sizeof(virtual_eeprom_)) {
            virtual_eeprom_[address] = data;
            send_ack(SetEEPRom);
        } else {
            log_error("SetEEPRom: address out of range (" + std::to_string(address) + " >= " + std::to_string(sizeof(virtual_eeprom_)) + ")");
            send_ack(SetEEPRom, AckStatus_Invalid, AckReport_ParamError);
        }
    } else if (packet.command == GetEEPRom) {
        uint8_t address = packet.data.Get_adress;
        
        log_debug("GetEEPRom: address=" + std::to_string(address));
        
        if (address < sizeof(virtual_eeprom_)) {
            uint8_t eep_data = virtual_eeprom_[address];
            send_ack(GetEEPRom, AckStatus_Ok, AckReport_Ok, &eep_data, 1);
        } else {
            log_error("GetEEPRom: address out of range (" + std::to_string(address) + " >= " + std::to_string(sizeof(virtual_eeprom_)) + ")");
            send_ack(GetEEPRom, AckStatus_Invalid, AckReport_ParamError);
        }
    }
}

// ============================================================================
// 映射和转换函数
// ============================================================================

void LightManager::map_mai2light_to_neopixel(uint8_t mai2light_index, uint8_t r, uint8_t g, uint8_t b) {
    if (!is_ready()) {
        log_error("map_mai2light_to_neopixel: LightManager not ready");
        return;
    }
    
    if (mai2light_index >= 32) {
        log_error("map_mai2light_to_neopixel: index out of range (" + std::to_string(mai2light_index) + " >= 32)");
        return;
    }
    
    bool mapped = false;
    
    // 查找包含此索引的区域映射
    for (const auto& mapping : region_mappings_) {
        if (!mapping.enabled) {
            continue;
        }
        
        if (mai2light_index >= mapping.mai2light_start_index && 
            mai2light_index <= mapping.mai2light_end_index) {
            
            // 根据bitmap设置对应的NeoPixel
            bitmap32_t bitmap = mapping.neopixel_bitmap;
            for (uint8_t i = 0; i < 32; i++) {
                if (bitmap & (1U << i)) {
                    if (neopixel_) {
                        neopixel_->set_pixel(i, r, g, b);
                        mapped = true;
                    } else {
                        log_error("map_mai2light_to_neopixel: neopixel instance is null");
                        return;
                    }
                }
            }
            
            log_debug("Mapped Mai2Light[" + std::to_string(mai2light_index) + "] to region '" + mapping.name + "' with color RGB(" + std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
            break;
        }
    }
    
    if (!mapped) {
        log_debug("No mapping found for Mai2Light index " + std::to_string(mai2light_index));
    }
}

void LightManager::map_range_to_neopixel(uint8_t start_index, uint8_t end_index, uint8_t r, uint8_t g, uint8_t b) {
    if (!is_ready()) {
        log_error("map_range_to_neopixel: LightManager not ready");
        return;
    }
    
    if (start_index > end_index) {
        log_error("map_range_to_neopixel: invalid range (" + std::to_string(start_index) + " > " + std::to_string(end_index) + ")");
        return;
    }
    
    if (end_index >= 32) {
        log_error("map_range_to_neopixel: end index out of range (" + std::to_string(end_index) + " >= 32)");
        return;
    }
    
    log_debug("Mapping range [" + std::to_string(start_index) + "-" + std::to_string(end_index) + "] with color RGB(" + std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
    
    // 为范围内的每个索引设置颜色
    for (uint8_t i = start_index; i <= end_index; i++) {
        map_mai2light_to_neopixel(i, r, g, b);
    }
}

bitmap32_t LightManager::get_neopixel_bitmap_for_mai2light_range(uint8_t start_index, uint8_t end_index) const {
    bitmap32_t combined_bitmap = 0;
    
    for (const auto& mapping : region_mappings_) {
        if (!mapping.enabled) {
            continue;
        }
        
        // 检查范围是否有重叠
        if (!(end_index < mapping.mai2light_start_index || start_index > mapping.mai2light_end_index)) {
            combined_bitmap |= mapping.neopixel_bitmap;
        }
    }
    
    return combined_bitmap;
}

// ============================================================================
// 渐变效果处理
// ============================================================================

void LightManager::update_fade_effects() {
    if (!fade_state_.active || !neopixel_) {
        return;
    }
    
    // 注意：这里需要实际的时间函数
    uint32_t current_time = 0;
    
    if (current_time >= fade_state_.end_time) {
        // 渐变完成
        for (uint8_t i = fade_state_.start_led; i <= fade_state_.end_led; i++) {
            map_mai2light_to_neopixel(i, fade_state_.end_color.r, 
                                     fade_state_.end_color.g, fade_state_.end_color.b);
        }
        fade_state_.active = false;
        neopixel_->show();
    } else {
        // 计算渐变进度
        uint32_t elapsed = current_time - fade_state_.start_time;
        uint32_t duration = fade_state_.end_time - fade_state_.start_time;
        uint8_t progress = (elapsed * 255) / duration;
        
        // 计算当前颜色
        NeoPixel_Color current_color = NeoPixel::blend_colors(
            fade_state_.start_color, fade_state_.end_color, progress);
        
        // 设置颜色
        for (uint8_t i = fade_state_.start_led; i <= fade_state_.end_led; i++) {
            map_mai2light_to_neopixel(i, current_color.r, current_color.g, current_color.b);
        }
        neopixel_->show();
    }
}

// ============================================================================
// 工具函数
// ============================================================================

uint8_t LightManager::calculate_checksum(const uint8_t* data, uint8_t length) const {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

void LightManager::log_debug(const std::string& message) const {
    if (debug_enabled_) {
        USB_SerialLogs::global_log(USB_LogLevel::DEBUG, message, "LightManager");
    }
}

void LightManager::log_error(const std::string& message) const {
    USB_SerialLogs::global_log(USB_LogLevel::ERROR, message, "LightManager");
}
