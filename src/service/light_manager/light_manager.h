#ifndef LIGHT_MANAGER_H
#define LIGHT_MANAGER_H

#include "../../protocol/mai2light/mai2light.h"
#include "../../protocol/neopixel/neopixel.h"
#include <stdint.h>
#include <string>
#include <vector>
#include <functional>
#include "../config_manager/config_types.h"

// 前向声明
class ConfigManager;
class UIManager;

// LightManager配置键定义 - 遵循服务层规则2: {服务名}_{子模块}_{键}
#define LIGHTMANAGER_ENABLE "LIGHTMANAGER_ENABLE"
#define LIGHTMANAGER_UART_DEVICE "LIGHTMANAGER_UART_DEVICE"
#define LIGHTMANAGER_BAUD_RATE "LIGHTMANAGER_BAUD_RATE"
#define LIGHTMANAGER_NODE_ID "LIGHTMANAGER_NODE_ID"
#define LIGHTMANAGER_NEOPIXEL_COUNT "LIGHTMANAGER_NEOPIXEL_COUNT"
#define LIGHTMANAGER_NEOPIXEL_PIN "LIGHTMANAGER_NEOPIXEL_PIN"
#define LIGHTMANAGER_REGION_MAPPINGS "LIGHTMANAGER_REGION_MAPPINGS"

// 数据类型定义
typedef uint16_t bitmap16_t;  // 16位bitmap，支持最多16个灯

// 区域枚举定义 (1-11对应原始灯编号)
enum LightRegion : uint8_t {
    REGION_1 = 1,
    REGION_2 = 2,
    REGION_3 = 3,
    REGION_4 = 4,
    REGION_5 = 5,
    REGION_6 = 6,
    REGION_7 = 7,
    REGION_8 = 8,
    REGION_9 = 9,
    REGION_10 = 10,
    REGION_11 = 11,
    REGION_COUNT = 11
};

// 简化的区域映射结构
struct RegionBitmap {
    bitmap16_t neopixel_bitmap;  // 16位bitmap，表示该区域映射的neopixel位置
    uint8_t r, g, b;             // 当前颜色值
    bool enabled;                // 是否启用
    
    RegionBitmap() : neopixel_bitmap(0), r(0), g(0), b(0), enabled(false) {}
};

// 时间片调度结构
struct TimeSliceScheduler {
    uint32_t slice_start_time;   // 时间片开始时间 (微秒)
    uint32_t slice_duration_us;  // 时间片持续时间 (微秒，默认100us = 0.1ms)
    uint8_t current_region;      // 当前处理的区域
    uint8_t current_led;         // 当前处理的LED
    bool processing_active;      // 是否正在处理
    
    TimeSliceScheduler() : slice_start_time(0), slice_duration_us(100), 
                          current_region(0), current_led(0), processing_active(false) {}
};

// 灯光管理器私有配置
// 遵循服务层规则3: 私有配置结构体，不存储在Class内部
struct LightManager_PrivateConfig {
    bool enable;                         // 启用灯光管理器
    std::string uart_device;             // UART设备名称
    uint32_t baud_rate;                  // 波特率
    uint8_t node_id;                     // 节点ID
    uint16_t neopixel_count;             // Neopixel数量
    uint8_t neopixel_pin;                // Neopixel引脚
    
    // 区域映射持久化数据
    bitmap16_t region_bitmaps[REGION_COUNT];     // 区域bitmap映射
    bool region_enabled[REGION_COUNT];           // 区域启用状态
    uint8_t region_colors[REGION_COUNT][3];      // 区域颜色 [R,G,B]

    LightManager_PrivateConfig()
        : enable(true)
        , uart_device("uart1")
        , baud_rate(115200)
        , node_id(1)
        , neopixel_count(128)
        , neopixel_pin(16) {
        // 初始化区域映射数据
        for (int i = 0; i < REGION_COUNT; i++) {
            region_bitmaps[i] = 0;
            region_enabled[i] = false;
            region_colors[i][0] = 0; // R
            region_colors[i][1] = 0; // G
            region_colors[i][2] = 0; // B
        }
    }
};

// 灯光管理器配置 (仅包含服务指针)
// 遵循服务层规则3: 服务依赖配置结构体
struct LightManager_Config {
    ConfigManager* config_manager;
    UIManager* ui_manager;

    LightManager_Config()
        : config_manager(nullptr)
        , ui_manager(nullptr) {}
};

// 配置管理函数声明
// 遵循服务层规则3: 配置管理函数 - 完全的单数据存储和及时响应配置更变
void lightmanager_register_default_configs(config_map_t& default_map);     // [默认配置注册函数] 注册默认配置到ConfigManager
LightManager_PrivateConfig* lightmanager_get_config_holder();              // [配置保管函数] 保存静态私有配置变量并返回指针
bool lightmanager_write_config_to_manager(const LightManager_PrivateConfig& config); // [配置写入函数] 将配置写入到config_holder地址中
LightManager_PrivateConfig lightmanager_get_config_copy();                 // [配置读取函数] 从config_holder地址读取配置并返回副本
bool lightmanager_save_config_to_manager(const LightManager_PrivateConfig& config); // [配置保存函数] 将config_holder地址的配置保存到ConfigManager
bool lightmanager_load_config_from_manager();                              // [配置加载函数] 从ConfigManager加载配置到config_holder并应用

// 协议回调函数已移除 - LightManager不再处理协议指令

// 灯光管理器类 (单例)
class LightManager {
public:
    // 单例模式
    static LightManager* getInstance();
    
    // 析构函数
    ~LightManager();

    // 初始化配置结构体
    struct InitConfig {
        Mai2Light* mai2light;           // Mai2Light实例指针
        NeoPixel* neopixel;             // NeoPixel实例指针
        
        InitConfig() : mai2light(nullptr), neopixel(nullptr) {}
        InitConfig(Mai2Light* m2l, NeoPixel* np) : mai2light(m2l), neopixel(np) {}
    };
    
    // 初始化和释放
    bool init(const InitConfig& init_config);
    void deinit();
    bool is_ready() const;
    
    // 基础灯光控制接口
    void set_region_color(uint8_t region_id, uint8_t r, uint8_t g, uint8_t b);  // 设置区域颜色
    void set_single_led(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);    // 设置单个LED
    void clear_all_leds();                                                      // 清空所有LED
    
    // 区域映射管理 (简化版)
    void set_region_bitmap(uint8_t region_id, bitmap16_t bitmap);              // 设置区域bitmap映射
    bitmap16_t get_region_bitmap(uint8_t region_id) const;                     // 获取区域bitmap映射
    std::vector<std::string> get_region_names() const;                         // 获取区域名称列表
    void save_region_mappings();                                               // 保存区域映射
    void load_region_mappings();                                               // 加载区域映射
    
    // 数据同步方法
    bool sync_mai2light_to_regions();                                          // 从mai2light同步LED数据到区域
    bool apply_regions_to_neopixel();                                          // 将区域数据应用到neopixel
    
    // Mai2Light配置管理
    bool update_mai2light_config(const Mai2Light_Config& config);              // 更新mai2light配置
    Mai2Light_Config get_mai2light_config() const;                             // 获取mai2light配置
    
    // Loop接口 - 处理Mai2Light回调
    void task();
    
    // 协议回调函数已移除
    
    // 调试功能
    void enable_debug_output(bool enabled);
    std::string get_debug_info() const;
    
private:
    // 私有构造函数 (单例模式)
    LightManager();
    LightManager(const LightManager&) = delete;
    LightManager& operator=(const LightManager&) = delete;
    
    // 单例实例
    static LightManager* instance_;
    
    // 成员变量
    bool initialized_;
    bool debug_enabled_;
    
    // 配置相关
    LightManager_PrivateConfig* config_holder_;
    
    // 模块实例指针
    Mai2Light* mai2light_;
    NeoPixel* neopixel_;
    
    // 区域bitmap数组 (11个区域)
    RegionBitmap region_bitmaps_[REGION_COUNT];
    
    // 时间片调度器
    TimeSliceScheduler scheduler_;
    
    // 协议回调函数已移除
    
    // 指令解析功能已移除 - 所有handle_*函数不再使用
    
    // 时间片调度处理
    void process_time_slice();
    bool is_time_slice_expired() const;
    void reset_time_slice();
    
    // 映射和转换函数
    void map_mai2light_to_neopixel(uint8_t mai2light_index, uint8_t r, uint8_t g, uint8_t b);
    void map_range_to_neopixel(uint8_t start_index, uint8_t end_index, uint8_t r, uint8_t g, uint8_t b);
    void apply_region_to_leds(uint8_t region_id);
    uint8_t get_region_index(uint8_t region_id) const;  // 将1-11转换为0-10索引
    
    // 工具函数
    uint8_t calculate_checksum(const uint8_t* data, uint8_t length) const;
    inline void log_debug(const std::string& message) const;
    inline void log_error(const std::string& message) const;
};

#endif // LIGHT_MANAGER_H