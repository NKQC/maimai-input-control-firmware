#include "light_manager.h"
#include "../config_manager/config_manager.h"
#include "../../hal/pio/hal_pio.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include "pico/time.h"  // 用于time_us_32()函数

// 前向声明 - 避免循环依赖
class UIManager;

// 静态实例
LightManager* LightManager::instance_ = nullptr;

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
LightManager_PrivateConfig* lightmanager_get_config_holder() {
    static LightManager_PrivateConfig config;
    return &config;
}



// [配置读取函数] 遵循服务层规则3 - 从config_holder地址读取配置并返回副本
LightManager_PrivateConfig lightmanager_get_config_copy() {
    LightManager_PrivateConfig* holder = lightmanager_get_config_holder();
    if (holder) {
        return *holder;
    }
    // 如果获取失败，返回默认配置
    return LightManager_PrivateConfig{};
}

// [配置保存函数] 遵循服务层规则3 - 将config_holder地址的配置保存到ConfigManager
bool lightmanager_save_config_to_manager(const LightManager_PrivateConfig& config) {
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

// [配置写入函数] 遵循服务层规则3 - 将配置写入到config_holder地址中
bool lightmanager_write_config_to_manager(const LightManager_PrivateConfig& config) {
    LightManager_PrivateConfig* holder = lightmanager_get_config_holder();
    if (!holder) return false;
    
    // 将配置写入到config_holder地址
    *holder = config;
    
    return true;
}

// ============================================================================
// LightManager类实现
// ============================================================================

// 遵循服务层规则3: 服务本身完全不保存配置
LightManager::LightManager() 
    : initialized_(false)
    , debug_enabled_(false)
    , config_holder_(lightmanager_get_config_holder())
    , mai2light_(nullptr)
    , neopixel_(nullptr)
    {
    
    // 初始化区域bitmap数组
    for (int i = 0; i < REGION_COUNT; i++) {
        region_bitmaps_[i] = RegionBitmap();
    }
    
    // 初始化时间片调度器
    scheduler_ = TimeSliceScheduler();
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

bool LightManager::init(const InitConfig& init_config) {
    if (initialized_) {
        return true;
    }
    
    log_debug("Initializing LightManager...");
    
    // 验证传入的实例指针
    if (!init_config.mai2light || !init_config.neopixel) {
        log_error("Invalid mai2light or neopixel instance");
        return false;
    }
    
    // 保存Mai2Light和NeoPixel指针
    mai2light_ = init_config.mai2light;
    neopixel_ = init_config.neopixel;
    
    // 直接从config_holder获取配置
    if (!config_holder_ || !config_holder_->enable) {
        log_debug("LightManager disabled in configuration");
        return false;
    }
    
    // 初始化区域bitmap为默认值
    for (int i = 0; i < REGION_COUNT; i++) {
        region_bitmaps_[i].neopixel_bitmap = (1 << i); // 每个区域对应一个LED
    }
    
    log_debug("LightManager initialized successfully");
    return true;
}

void LightManager::deinit() {
    if (!initialized_) {
        return;
    }
    
    log_debug("Deinitializing LightManager...");
    
    initialized_ = false;
    log_debug("LightManager deinitialized");
}

bool LightManager::is_ready() const {
    return initialized_ && neopixel_;
}

// ============================================================================
// 基础灯光控制接口
// ============================================================================

void LightManager::set_region_color(uint8_t region_id, uint8_t r, uint8_t g, uint8_t b) {
    if (!is_ready()) {
        log_error("LightManager not ready");
        return;
    }
    
    uint8_t index = get_region_index(region_id);
    if (index >= REGION_COUNT) {
        log_error("Invalid region ID: " + std::to_string(region_id));
        return;
    }
    
    // 设置区域颜色
    region_bitmaps_[index].r = r;
    region_bitmaps_[index].g = g;
    region_bitmaps_[index].b = b;
    region_bitmaps_[index].enabled = true;
    
    log_debug("Set region " + std::to_string(region_id) + " color: RGB(" + 
              std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
}

void LightManager::set_single_led(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (!is_ready() || !neopixel_) {
        log_error("LightManager or NeoPixel not ready");
        return;
    }
    
    // 直接设置单个LED
    neopixel_->set_pixel(led_index, r, g, b);
    neopixel_->show();
    
    log_debug("Set LED " + std::to_string(led_index) + " color: RGB(" + 
              std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
}

void LightManager::clear_all_leds() {
    if (!is_ready() || !neopixel_) {
        log_error("LightManager or NeoPixel not ready");
        return;
    }
    
    // 清空所有区域
    for (int i = 0; i < REGION_COUNT; i++) {
        region_bitmaps_[i].r = 0;
        region_bitmaps_[i].g = 0;
        region_bitmaps_[i].b = 0;
        region_bitmaps_[i].enabled = false;
    }
    
    // 清空所有LED
    neopixel_->clear_all();
    neopixel_->show();
    
    log_debug("Cleared all LEDs");
}

// ============================================================================
// 区域映射管理 (简化版)
// ============================================================================

void LightManager::set_region_bitmap(uint8_t region_id, bitmap16_t bitmap) {
    if (!is_ready()) {
        log_error("LightManager not ready");
        return;
    }
    
    uint8_t index = get_region_index(region_id);
    if (index >= REGION_COUNT) {
        log_error("Invalid region ID: " + std::to_string(region_id));
        return;
    }
    
    region_bitmaps_[index].neopixel_bitmap = bitmap;
    log_debug("Set region " + std::to_string(region_id) + " bitmap: 0x" + 
              std::to_string(bitmap));
}

bitmap16_t LightManager::get_region_bitmap(uint8_t region_id) const {
    uint8_t index = get_region_index(region_id);
    if (index >= REGION_COUNT) {
        log_error("Invalid region ID: " + std::to_string(region_id));
        return 0;
    }
    
    return region_bitmaps_[index].neopixel_bitmap;
}

// ============================================================================
// 时间片调度处理
// ============================================================================

void LightManager::process_time_slice() {
    if (!is_ready() || !neopixel_) {
        return;
    }
    
    // 检查是否需要开始新的时间片
    if (!scheduler_.processing_active) {
        reset_time_slice();
        scheduler_.processing_active = true;
    }
    
    // 在时间片内处理区域到LED的映射
    while (!is_time_slice_expired() && scheduler_.current_region < REGION_COUNT) {
        apply_region_to_leds(scheduler_.current_region + 1);  // 转换为1-11区域ID
        scheduler_.current_region++;
    }
    
    // 如果处理完所有区域或时间片到期
    if (scheduler_.current_region >= REGION_COUNT || is_time_slice_expired()) {
        if (scheduler_.current_region >= REGION_COUNT) {
            // 所有区域处理完成，更新显示
            neopixel_->show();
        }
        scheduler_.processing_active = false;
    }
}

bool LightManager::is_time_slice_expired() const {
    uint32_t current_time = time_us_32();  // 获取当前微秒时间
    return (current_time - scheduler_.slice_start_time) >= scheduler_.slice_duration_us;
}

void LightManager::reset_time_slice() {
    scheduler_.slice_start_time = time_us_32();
    scheduler_.current_region = 0;
    scheduler_.current_led = 0;
}

// ============================================================================
// 工具函数
// ============================================================================

void LightManager::apply_region_to_leds(uint8_t region_id) {
    uint8_t index = get_region_index(region_id);
    if (index >= REGION_COUNT || !region_bitmaps_[index].enabled) {
        return;
    }
    
    const RegionBitmap& region = region_bitmaps_[index];
    bitmap16_t bitmap = region.neopixel_bitmap;
    
    // 遍历bitmap中设置的位，应用颜色到对应的LED
    for (uint8_t bit = 0; bit < 16; bit++) {
        if (bitmap & (1 << bit)) {
            neopixel_->set_pixel(bit, region.r, region.g, region.b);
        }
    }
}

uint8_t LightManager::get_region_index(uint8_t region_id) const {
    if (region_id >= 1 && region_id <= 11) {
        return region_id - 1;  // 转换1-11到0-10索引
    }
    return 255;  // 无效索引
}

// 移除了add_region_mapping方法，使用新的bitmap接口

// 移除了remove_region_mapping方法，使用新的bitmap接口

// 移除了get_region_mapping方法，使用新的bitmap接口

std::vector<std::string> LightManager::get_region_names() const {
    std::vector<std::string> names;
    for (uint8_t i = 1; i <= REGION_COUNT; i++) {
        names.push_back("Region " + std::to_string(i));
    }
    return names;
}

// ============================================================================
// 数据同步方法
// ============================================================================

// 从mai2light同步LED数据到区域
bool LightManager::sync_mai2light_to_regions() {
    if (!is_ready()) {
        return false;
    }
    
    // 获取mai2light的所有LED状态
    const Mai2Light_LEDStatus* led_status_array = mai2light_->get_led_status_array();
    if (!led_status_array) {
        return false;
    }
    
    // 直接将mai2light的LED数据映射到对应的neopixel
    for (uint8_t mai2light_idx = 0; mai2light_idx < 16 && mai2light_idx < MAI2LIGHT_NUM_LEDS; mai2light_idx++) {
        const Mai2Light_LEDStatus& led_status = led_status_array[mai2light_idx];
        if (led_status.enabled) {
            // 直接映射到对应的neopixel LED
            map_mai2light_to_neopixel(mai2light_idx, 
                                        led_status.color.r, 
                                        led_status.color.g, 
                                        led_status.color.b);
            }
        }
    
    return true;
}

// 将区域数据应用到neopixel
bool LightManager::apply_regions_to_neopixel() {
    if (!is_ready() || !neopixel_) {
        return false;
    }
    
    // 刷新neopixel显示
    return neopixel_->show();
}

// ============================================================================
// Mai2Light配置管理
// ============================================================================

// 更新mai2light配置
bool LightManager::update_mai2light_config(const Mai2Light_Config& config) {
    if (!is_ready() || !mai2light_) {
        return false;
    }
    
    // 设置mai2light配置
    if (!mai2light_->set_config(config)) {
        log_error("Failed to update mai2light configuration");
        return false;
    }
    
    log_debug("Mai2Light configuration updated successfully");
    return true;
}

// 获取mai2light配置
Mai2Light_Config LightManager::get_mai2light_config() const {
    if (!is_ready() || !mai2light_) {
        return Mai2Light_Config();
    }
    
    Mai2Light_Config config;
    mai2light_->get_config(config);
    return config;
}

// ============================================================================
// Loop接口 - 处理Mai2Light回调
// ============================================================================

void LightManager::loop() {
    if (!is_ready()) {
        return;
    }
    
    // 代为执行mai2light模块的loop
    if (mai2light_) {
        mai2light_->task();
    }
    
    // 时间片调度处理
    process_time_slice();
    
    // 更新neopixel动画
    if (neopixel_) {
        neopixel_->task();
    }
}

// ============================================================================
// 回调函数设置
// ============================================================================

// 协议回调函数已移除 - set_command_callback函数不再使用

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
    
    // 直接从config_holder获取配置，遵循服务层规则3
    if (config_holder_) {
        info += "Node ID: " + std::to_string(config_holder_->node_id) + "\n";
        
        // 配置信息
        info += "Enabled: " + std::string(config_holder_->enable ? "Yes" : "No") + "\n";
        info += "Baud Rate: " + std::to_string(config_holder_->baud_rate) + "\n";
        
        // NeoPixel状态
        if (neopixel_) {
            info += "NeoPixel: Connected (" + std::to_string(config_holder_->neopixel_count) + " LEDs on pin " + std::to_string(config_holder_->neopixel_pin) + ")\n";
        } else {
            info += "NeoPixel: Not connected\n";
        }
    } else {
        info += "Config: Not available\n";
    }
    
    // 区域bitmap信息
    info += "Region Bitmaps:\n";
    for (int i = 0; i < REGION_COUNT; i++) {
        info += "  Region " + std::to_string(i + 1) + ": bitmap=0x" + std::to_string(region_bitmaps_[i].neopixel_bitmap) + 
                ", RGB=(" + std::to_string(region_bitmaps_[i].r) + "," + std::to_string(region_bitmaps_[i].g) + "," + std::to_string(region_bitmaps_[i].b) + ")" +
                (region_bitmaps_[i].enabled ? " (enabled)" : " (disabled)") + "\n";
    }
    
    // 通信状态
    // BD15070协议状态信息已移除
    
    // 时间片调度信息
    info += "Time Slice Scheduler:\n";
    info += "  Current Region: " + std::to_string(scheduler_.current_region) + "\n";
    info += "  Current LED: " + std::to_string(scheduler_.current_led) + "\n";
    info += "  Slice Duration: " + std::to_string(scheduler_.slice_duration_us) + "us\n";
    
    // 调试状态
    info += "Debug Output: " + std::string(debug_enabled_ ? "Enabled" : "Disabled") + "\n";
    // 协议回调信息已移除
    
    return info;
}

// ============================================================================
// 内部方法 - 数据处理
// ============================================================================



// 指令解析功能已移除 - handle_command函数不再使用
// LightManager现在只负责LED控制，不再处理BD15070协议指令



// ============================================================================
// 命令处理函数实现
// ============================================================================

// 指令解析功能已移除 - handle_set_led_gs8bit函数不再使用

// 指令解析功能已移除 - handle_set_led_gs8bit_multi函数不再使用

// 指令解析功能已移除 - handle_set_led_gs8bit_multi_fade函数不再使用

// 指令解析功能已移除 - handle_set_led_fet函数不再使用

// 指令解析功能已移除 - handle_set_led_gs_update函数不再使用

// 指令解析功能已移除 - handle_get_board_info函数不再使用

// 指令解析功能已移除 - handle_get_board_status函数不再使用

// 指令解析功能已移除 - handle_get_firm_sum函数不再使用

// 指令解析功能已移除 - handle_get_protocol_version函数不再使用

// 指令解析功能已移除 - handle_eeprom_commands函数不再使用

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
    
    // 直接映射到对应的NeoPixel LED
    if (neopixel_ && mai2light_index < 16) {
        neopixel_->set_pixel(mai2light_index, r, g, b);
        log_debug("Mapped Mai2Light[" + std::to_string(mai2light_index) + "] to NeoPixel[" + std::to_string(mai2light_index) + "] with color RGB(" + std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
    } else {
        log_debug("Mai2Light index " + std::to_string(mai2light_index) + " out of range or neopixel not available");
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

// 移除了get_neopixel_bitmap_for_mai2light_range方法，使用新的bitmap接口

// ============================================================================
// 渐变效果处理
// ============================================================================

// 移除了update_fade_effects方法，不再使用渐变效果

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

// 保存区域映射到配置管理器
void LightManager::save_region_mappings() {
    if (!config_holder_) {
        log_error("Cannot save region mappings: config holder not available");
        return;
    }
    
    // 将当前的区域bitmap保存到配置中
    for (int i = 0; i < REGION_COUNT; i++) {
        config_holder_->region_bitmaps[i] = region_bitmaps_[i].neopixel_bitmap;
        config_holder_->region_enabled[i] = region_bitmaps_[i].enabled;
        config_holder_->region_colors[i][0] = region_bitmaps_[i].r;
        config_holder_->region_colors[i][1] = region_bitmaps_[i].g;
        config_holder_->region_colors[i][2] = region_bitmaps_[i].b;
    }
    
    // 保存配置到ConfigManager
    lightmanager_save_config_to_manager(*config_holder_);
    log_debug("Region mappings saved to configuration");
}

// 从配置管理器加载区域映射
void LightManager::load_region_mappings() {
    if (!config_holder_) {
        log_error("Cannot load region mappings: config holder not available");
        return;
    }
    
    // 从配置中加载区域bitmap
    for (int i = 0; i < REGION_COUNT; i++) {
        region_bitmaps_[i].neopixel_bitmap = config_holder_->region_bitmaps[i];
        region_bitmaps_[i].enabled = config_holder_->region_enabled[i];
        region_bitmaps_[i].r = config_holder_->region_colors[i][0];
        region_bitmaps_[i].g = config_holder_->region_colors[i][1];
        region_bitmaps_[i].b = config_holder_->region_colors[i][2];
    }
    
    log_debug("Region mappings loaded from configuration");
}
