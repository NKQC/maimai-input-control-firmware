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

        // ===== kbd.key00..key11 (12 keys) 物理键盘 GPIO1-12 → HID 键码 =====
        // 值 = HID usage(键盘/键区页, 0=不映射)。默认 GPIO1-12 → 1..9,0,Enter,Esc。
        // 用 HID_KeyCode 原始值: KEY_1=0x1E..KEY_9=0x26, KEY_0=0x27, ENTER=0x28, ESC=0x29。
        {
            const uint8_t kbd_defaults[12] = {
                0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
                0x24, 0x25, 0x26, 0x27, 0x28, 0x29
            };
            for (int i = 0; i < 12; i++) {
                char key_buf[16];
                snprintf(key_buf, sizeof(key_buf), "kbd.key%02d", i);
                config_map[key_buf] = ConfigValue(uint8_t(kbd_defaults[i]), uint8_t(0), uint8_t(255));
            }
        }

        // ===== kbd.zone00..zone33 (34) 触控→键盘映射: 逻辑分区 → HID 键码(0=不映射) =====
        // 由 comm.keyboard_map_en 总开关启用; 默认全 0(不映射), 用户经 UI 逐区设定。
        for (int i = 0; i < 34; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.zone%02d", i);
            config_map[key_buf] = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(255));
        }

        // ===== 组合键修饰位 (bit0=LCtrl bit1=LShift bit2=LAlt bit3=LGui) =====
        // kbd.km00..11: 物理键 GPIO1-12 的修饰键; kbd.zm00..33: 触控分区的修饰键。默认 0(无修饰)。
        for (int i = 0; i < 12; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.km%02d", i);
            config_map[key_buf] = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(15));
        }
        for (int i = 0; i < 34; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.zm%02d", i);
            config_map[key_buf] = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(15));
        }
        
        // ===== led.* (8 keys) =====
        config_map["led.enable"]                 = ConfigValue(true);
        config_map["led.node_id"]                = ConfigValue(uint8_t(1));
        config_map["led.count"]                  = ConfigValue(uint16_t(128), uint16_t(1), uint16_t(1000));
        config_map["led.status_brightness"]      = ConfigValue(uint8_t(128), uint8_t(0), uint8_t(255));
        // 颜色为 bit mask：0=关闭, 1=红(Red), 2=绿(Green), 3=黄(Yellow),
        // 4=蓝(Blue), 5=紫(Magenta), 6=青(Cyan), 7=白(White)。
        // ConfigValue 当前没有 enum/options 元数据，使用受限 U8 以保持上位机 schema 兼容。
        config_map["led.color_connected"]        = ConfigValue(uint8_t(2), uint8_t(0), uint8_t(7));
        config_map["led.color_flash_error"]      = ConfigValue(uint8_t(1), uint8_t(0), uint8_t(7));
        config_map["led.color_link_error"]       = ConfigValue(uint8_t(4), uint8_t(0), uint8_t(7));
        config_map["led.color_healthy"]          = ConfigValue(uint8_t(2), uint8_t(0), uint8_t(7));
        
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
