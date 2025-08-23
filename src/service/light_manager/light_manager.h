#ifndef LIGHT_MANAGER_H
#define LIGHT_MANAGER_H

#include "../../protocol/mai2light/mai2light.h"
#include "../../protocol/neopixel/neopixel.h"
#include "../../hal/uart/hal_uart.h"
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

// 协议常量定义 (基于BD15070_4.h)
#define BD15070_SYNC 0xE0
#define BD15070_MARKER 0xD0

// 协议命令定义
enum BD15070_Command : uint8_t {
    SetLedGs8Bit = 0x31,
    SetLedGs8BitMulti = 0x32,
    SetLedGs8BitMultiFade = 0x33,
    SetLedFet = 0x39,
    SetDcUpdate = 0x3B,
    SetLedGsUpdate = 0x3C,
    SetDc = 0x3F,
    SetEEPRom = 0x7B,
    GetEEPRom = 0x7C,
    SetEnableResponse = 0x7D,
    SetDisableResponse = 0x7E,
    GetBoardInfo = 0xF0,
    GetBoardStatus = 0xF1,
    GetFirmSum = 0xF2,
    GetProtocolVersion = 0xF3
};

// 应答状态定义
enum BD15070_AckStatus : uint8_t {
    AckStatus_Ok = 0x01,
    AckStatus_SumError = 0x02,
    AckStatus_ParityError = 0x03,
    AckStatus_FramingError = 0x04,
    AckStatus_OverRunError = 0x05,
    AckStatus_RecvBfOverFlow = 0x06,
    AckStatus_Invalid = 0xFF
};

// 应答报告定义
enum BD15070_AckReport : uint8_t {
    AckReport_Ok = 0x01,
    AckReport_Busy = 0x02,
    AckReport_CommandUnknown = 0x03,
    AckReport_ParamError = 0x04,
    AckReport_Invalid = 0xFF
};

// 数据类型定义
typedef uint32_t bitmap32_t;

// 请求数据包结构 (基于BD15070_4.h PacketReq)
struct BD15070_PacketReq {
    uint8_t dstNodeID;
    uint8_t srcNodeID;
    uint8_t length;
    uint8_t command;
    
    union {
        uint8_t timeout;
        struct {
            uint8_t index;
            uint8_t color[3];
        } led_gs8bit;
        struct {
            uint8_t start;
            uint8_t end;
            uint8_t skip;
            uint8_t Multi_color[3];
            uint8_t speed;
        } led_multi;
        struct {
            uint8_t BodyLed;
            uint8_t ExtLed;
            uint8_t SideLed;
        } led_fet;
        struct {
            uint8_t Set_adress;
            uint8_t writeData;
        } eeprom_set;
        uint8_t Get_adress;
        uint8_t Direct_color[11][3];
    } data;
};

// 应答数据包结构 (基于BD15070_4.h PacketAck)
struct BD15070_PacketAck {
    uint8_t dstNodeID;
    uint8_t srcNodeID;
    uint8_t length;
    uint8_t status;
    uint8_t command;
    uint8_t report;
    
    union {
        uint8_t eepData;
        struct {
            uint8_t boardNo[9];
            uint8_t firmRevision;
        } board_info;
        struct {
            uint8_t timeoutStat;
            uint8_t timeoutSec;
            uint8_t pwmIo;
            uint8_t fetTimeout;
        } board_status;
        struct {
            uint8_t sum_upper;
            uint8_t sum_lower;
        } firm_sum;
        struct {
            uint8_t appliMode;
            uint8_t major;
            uint8_t minor;
        } protocol_version;
    } data;
};

// 灯光区域映射结构
struct LightRegionMapping {
    std::string name;                    // 区域名称
    uint8_t mai2light_start_index;       // Mai2Light起始索引
    uint8_t mai2light_end_index;         // Mai2Light结束索引
    bitmap32_t neopixel_bitmap;          // Neopixel位置bitmap (最多32个位置)
    bool enabled;                        // 是否启用映射
    
    LightRegionMapping() : name(), mai2light_start_index(0), 
                          mai2light_end_index(0), neopixel_bitmap(0), enabled(true) {}
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

    LightManager_PrivateConfig()
        : enable(true)
        , uart_device("uart1")
        , baud_rate(115200)
        , node_id(1)
        , neopixel_count(128)
        , neopixel_pin(16) {}
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
LightManager_Config* lightmanager_get_config_holder();                     // [配置保管函数] 保存静态私有配置变量并返回指针
bool lightmanager_load_config_from_manager(LightManager_Config* config);   // [配置加载函数] 从ConfigManager获取配置存入指针
LightManager_PrivateConfig lightmanager_get_config_copy();                 // [配置读取函数] 复制配置副本并返回
bool lightmanager_write_config_to_manager(const LightManager_PrivateConfig& config); // [配置写入函数] 将参数传回ConfigManager

// Mai2Light命令回调函数类型
typedef std::function<void(BD15070_Command command, const BD15070_PacketReq& packet)> LightManager_CommandCallback;

// 灯光管理器类 (单例)
class LightManager {
public:
    // 单例模式
    static LightManager* getInstance();
    
    // 析构函数
    ~LightManager();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 区域映射管理
    bool add_region_mapping(const LightRegionMapping& mapping);
    bool remove_region_mapping(const std::string& name);
    bool get_region_mapping(const std::string& name, LightRegionMapping& mapping) const;
    std::vector<std::string> get_region_names() const;
    bool enable_region_mapping(const std::string& name, bool enabled);
    
    // 区域映射配置管理
    bool save_region_mappings();
    bool load_region_mappings();
    bool reset_region_mappings();
    
    // 手动触发映射接口
    bool trigger_region_mapping(const std::string& region_name, uint8_t r, uint8_t g, uint8_t b);
    
    // Loop接口 - 处理Mai2Light回调
    void loop();
    
    // 设置回调函数
    void set_command_callback(LightManager_CommandCallback callback);
    
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
    
    // 协议相关 - 遵循服务层规则3: 服务本身完全不保存配置
    HAL_UART* uart_hal_;
    NeoPixel* neopixel_;
    // node_id_ 已移除 - 通过lightmanager_get_config_copy()获取
    
    // 接收缓冲区
    uint8_t rx_buffer_[64];
    uint8_t rx_buffer_pos_;
    bool escape_next_;
    
    // 区域映射
    std::vector<LightRegionMapping> region_mappings_;
    
    // 渐变效果状态
    struct FadeState {
        bool active;
        uint32_t start_time;
        uint32_t end_time;
        uint8_t start_led;
        uint8_t end_led;
        NeoPixel_Color start_color;
        NeoPixel_Color end_color;
    } fade_state_;
    
    // 回调函数
    LightManager_CommandCallback command_callback_;
    
    // 内部方法
    void process_received_data();
    bool parse_packet(const uint8_t* buffer, uint8_t length, BD15070_PacketReq& packet);
    void handle_command(const BD15070_PacketReq& packet);
    void send_ack(BD15070_Command command, BD15070_AckStatus status = AckStatus_Ok, 
                  BD15070_AckReport report = AckReport_Ok, const uint8_t* data = nullptr, uint8_t data_length = 0);
    
    // 命令处理函数
    void handle_set_led_gs8bit(const BD15070_PacketReq& packet);
    void handle_set_led_gs8bit_multi(const BD15070_PacketReq& packet);
    void handle_set_led_gs8bit_multi_fade(const BD15070_PacketReq& packet);
    void handle_set_led_fet(const BD15070_PacketReq& packet);
    void handle_set_led_gs_update(const BD15070_PacketReq& packet);
    void handle_get_board_info(const BD15070_PacketReq& packet);
    void handle_get_board_status(const BD15070_PacketReq& packet);
    void handle_get_firm_sum(const BD15070_PacketReq& packet);
    void handle_get_protocol_version(const BD15070_PacketReq& packet);
    void handle_eeprom_commands(const BD15070_PacketReq& packet);
    
    // 映射和转换函数
    void map_mai2light_to_neopixel(uint8_t mai2light_index, uint8_t r, uint8_t g, uint8_t b);
    void map_range_to_neopixel(uint8_t start_index, uint8_t end_index, uint8_t r, uint8_t g, uint8_t b);
    bitmap32_t get_neopixel_bitmap_for_mai2light_range(uint8_t start_index, uint8_t end_index) const;
    
    // 渐变效果处理
    void update_fade_effects();
    
    // 工具函数
    uint8_t calculate_checksum(const uint8_t* data, uint8_t length) const;
    void log_debug(const std::string& message) const;
    void log_error(const std::string& message) const;
};

#endif // LIGHT_MANAGER_H