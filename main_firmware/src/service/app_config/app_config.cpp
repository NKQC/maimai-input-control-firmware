#include "app_config.h"
#include "../config_manager/config_manager.h"
#include <cstdio>

void app_config_register_schema() {
    // Register schema via ConfigManager::register_init_function
    // This function is called during ConfigManager::initialize() to populate defaults
    
    ConfigManager::register_init_function([](config_map_t& config_map) {
        // ===== comm.* (13 keys) =====
        config_map["comm.sample_delay_ms"]       = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(100));
        config_map["comm.send_only_on_change"]   = ConfigValue(false);
        config_map["comm.aggregation_delay_ms"]  = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(100));
        config_map["comm.extra_send"]            = ConfigValue(uint8_t(0), uint8_t(0), uint8_t(10));
        config_map["comm.rate_limit_en"]         = ConfigValue(false);
        config_map["comm.rate_limit_hz"]         = ConfigValue(uint16_t(120), uint16_t(1), uint16_t(1000));
        config_map["comm.keyboard_map_en"]       = ConfigValue(false);
        // 仅当 comm.keyboard_map_en 为真时有意义；为真时触控映射仅在 mai2serial 实际发送触控数据期间生效。
        config_map["comm.keyboard_map_serial_only"] = ConfigValue(false);
        config_map["comm.serial_baud"]           = ConfigValue(uint32_t(115200));
        config_map["comm.light_baud"]            = ConfigValue(uint32_t(115200));
        config_map["comm.serial_reset_calibrate"] = ConfigValue(false);
        config_map["comm.serial_reset_baseline"]  = ConfigValue(true);
        // 触控串口/键盘输出延迟线: 100us 时间片, 0..1000 片 = 0..100ms, UI 可设。
        // 触控延迟仅作用于串口上报; 触控->键盘映射走原始触控, 不加键盘延迟。
        config_map["comm.touch_delay_100us"]     = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(1000));
        config_map["comm.keyboard_delay_100us"]  = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(1000));
        
        // ===== calib.* (1 key) 校准偏好 =====
        // calib.pref: 频率自适应的"灵敏度档位" 1..7(默认 4=居中)。随 AUTO_TUNE 请求下发,
        // 档位越高 → PSoC 在"校准刚好通过的临界最高频率"基础上往低频多让 2 个 snsClk 分频/档,
        // 充电更充分产生近场探测效应(更灵敏); 档位 1 = 临界频率本身(最不灵敏, 余量最小)。
        config_map["calib.pref"]                 = ConfigValue(uint8_t(4), uint8_t(1), uint8_t(7));

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

        // ===== 每键长按参数(毫秒, 0=禁用) =====
        // kbd.hdNN/kbd.mhNN: 物理键 GPIO1-12; kbd.zhdNN/kbd.zmhNN: 触控分区。
        // hd/zhd = 按下需持续该时长才真正输出 HID(0=立即输出, 原行为);
        // mh/zmh = 已输出后最长保持该时长即自动 release(0=不自动抬起), 抬起后需松开才能再触发。
        for (int i = 0; i < 12; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.hd%02d", i);
            config_map[key_buf] = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(65535));
            snprintf(key_buf, sizeof(key_buf), "kbd.mh%02d", i);
            config_map[key_buf] = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(65535));
        }
        for (int i = 0; i < 34; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.zhd%02d", i);
            config_map[key_buf] = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(65535));
            snprintf(key_buf, sizeof(key_buf), "kbd.zmh%02d", i);
            config_map[key_buf] = ConfigValue(uint16_t(0), uint16_t(0), uint16_t(65535));
        }

        // ===== 物理键每键触发极性 + 独立防抖 =====
        // kbd.plNN: 0=低电平触发 / 1=高电平触发 / 2=AUTO(默认)。
        //   AUTO = 只看**启动时**的电平并把它当作该键的"抬起"电平: 启动为高 → 低电平触发,
        //   启动为低 → 高电平触发; 启动后不再重采样。既有硬件(1K 外部上拉直连键)开机空闲为高,
        //   AUTO 会解析成低电平触发, 与旧固件的全局 active-low 行为一致, 故默认 AUTO 不会反转行为。
        //   ★围栏上限必须是 2★: ConfigValue 构造会 clamp_value(), 上限仍写 1 的话默认值 2 会被
        //   静默夹成 1(全部键变成高电平触发), 表现为开机后按键全程常按, 极难自查。
        // kbd.dbNN: 该键防抖窗(微秒), 0=不去抖, 上限 10000。默认 3000 = 改造前的全局 DEBOUNCE_US。
        //   逐键独立(固件侧各自记录稳定起点), 一个抖动键不再重置其它键的防抖窗。
        for (int i = 0; i < 12; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.pl%02d", i);
            config_map[key_buf] = ConfigValue(uint8_t(2), uint8_t(0), uint8_t(2));
            snprintf(key_buf, sizeof(key_buf), "kbd.db%02d", i);
            config_map[key_buf] = ConfigValue(uint16_t(3000), uint16_t(0), uint16_t(10000));
        }

        // ===== kbd.cbA/B/K/T 00..15: 触控组合映射(16 条) =====
        // 一条 = "zone_mask 内的分区全部同时按下" → "keycode[0..3] 全部同时输出"。
        // ★为什么打包成 4 个 uint32 而不是逐字段一个 KV★: 逐字段要新增 144 项, 明显放大配置 JSON
        // 与 CRC 计算开销; 打包后仅 64 项。位域布局与 keyboard.cpp 的 _load_combo/_store_combo 严格镜像:
        //   cbA = zone_mask[31:0]
        //   cbB = bit0..1: zone_mask[33:32]; bit8..15: 修饰位(bit0=LCtrl bit1=LShift bit2=LAlt bit3=LGui)
        //   cbK = key0 | key1<<8 | key2<<16 | key3<<24 (HID usage, 0=空位)
        //   cbT = delay_ms | max_hold_ms<<16 (毫秒, 0=禁用该项)
        // 默认全 0 = 组合表为空; 此时固件回落到 kbd.zoneNN 的 per-zone 判定, 存量配置不受影响。
        for (int i = 0; i < 16; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "kbd.cbA%02d", i);
            config_map[key_buf] = ConfigValue(uint32_t(0), uint32_t(0), uint32_t(0xFFFFFFFF));
            snprintf(key_buf, sizeof(key_buf), "kbd.cbB%02d", i);
            config_map[key_buf] = ConfigValue(uint32_t(0), uint32_t(0), uint32_t(0x0000FF03));
            snprintf(key_buf, sizeof(key_buf), "kbd.cbK%02d", i);
            config_map[key_buf] = ConfigValue(uint32_t(0), uint32_t(0), uint32_t(0xFFFFFFFF));
            snprintf(key_buf, sizeof(key_buf), "kbd.cbT%02d", i);
            config_map[key_buf] = ConfigValue(uint32_t(0), uint32_t(0), uint32_t(0xFFFFFFFF));
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

        // ===== WS2812 灯链 (3 + 11 keys) =====
        // 两路物理链: 通道 0=GPIO13, 通道 1=GPIO14。默认 64 珠/链 = 常见 8 键×8 珠布线,
        // 且 60Hz 下单链 64 珠推流 ≈1.9ms, core0 负担可接受。
        config_map["led.ws_count0"]              = ConfigValue(uint16_t(64), uint16_t(1), uint16_t(1000));
        config_map["led.ws_count1"]              = ConfigValue(uint16_t(64), uint16_t(1), uint16_t(1000));
        // WS2812 亮度独立于板载状态灯亮度(led.status_brightness 已被 main.cpp 心跳灯占用),
        // 否则调灯链亮度会连带改状态灯。
        config_map["led.ws_brightness"]          = ConfigValue(uint8_t(128), uint8_t(0), uint8_t(255));
        // led.map00..map10: 虚拟 LED 单元(0..7 按键灯, 8/9/10 = Body/Ext/Side)→ 物理段。
        // 打包 u32: bit31..28=channel(0/1, 0xF=未映射), bit27..12=start, bit11..0=count。
        // 结构性约束(越界/段重叠)无法用标量 range 表达, 由 LED_SET_REGION 整批校验; 默认全未映射。
        for (int i = 0; i < 11; i++) {
            char key_buf[16];
            snprintf(key_buf, sizeof(key_buf), "led.map%02d", i);
            config_map[key_buf] = ConfigValue(uint32_t(0xF0000000u));
        }
        
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
