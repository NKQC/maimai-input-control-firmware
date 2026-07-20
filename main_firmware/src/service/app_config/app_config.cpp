#include "app_config.h"
#include "../config_manager/config_manager.h"
#include <cstdio>

void app_config_register_schema() {
    // Register schema via ConfigManager::register_init_function
    // This function is called during ConfigManager::initialize() to populate defaults
    
    ConfigManager::register_init_function([](config_map_t& config_map) {
        // ===== comm.* (9 keys) =====
        config_map["comm.sample_delay_ms"]       = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(100));
        config_map["comm.send_only_on_change"]   = ConfigValue(false);
        config_map["comm.aggregation_delay_ms"]  = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(100));
        config_map["comm.extra_send"]            = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(10));
        config_map["comm.rate_limit_en"]         = ConfigValue(false);
        config_map["comm.rate_limit_hz"]         = ConfigValue(uint16_t(120), uint16_t(1), uint16_t(1000));
        config_map["comm.keyboard_map_en"]       = ConfigValue(false);
        config_map["comm.serial_baud"]           = ConfigValue(uint32_t(115200));
        config_map["comm.light_baud"]            = ConfigValue(uint32_t(115200));
        // 触控串口/键盘输出延迟线: 100us 时间片, 0..1000 片 = 0..100ms, UI 可设。
        // 触控延迟仅作用于串口上报; 触控->键盘映射走原始触控, 不加键盘延迟。
        config_map["comm.touch_delay_100us"]     = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(1000));
        config_map["comm.keyboard_delay_100us"]  = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(1000));
        
        // ===== mode.work (1 key) =====
        config_map["mode.work"]                  = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(1));
        
        // ===== led.* (3 keys) =====
        config_map["led.enable"]                 = ConfigValue(true);
        config_map["led.node_id"]                = ConfigValue(uint8_t(1));
        config_map["led.count"]                  = ConfigValue(uint16_t(128), uint16_t(1), uint16_t(1000));
        
        // ===== bind.map00..bind.map33 (34 keys) =====
        for (int i = 0; i < 34; i++) {
            char key_buf[32];
            snprintf(key_buf, sizeof(key_buf), "bind.map%02d", i);
            // 默认恒等绑定:逻辑分区 i → 物理通道 i(0..33)，保持开箱可用(等价旧 ZONE_CHANNEL_MAP)，
            // 用户可经绑定页/指触改到实际接线。值语义=物理通道索引(0..35)，0xFFFFFFFF=未映射。
            config_map[key_buf] = ConfigValue(uint32_t(i));
        }
    });
}
