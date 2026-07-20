# Phase 3 设计 (打通固件级功能)

## 蓝本(v3.1)关键结构
- TouchKeyboardMapping: area_mask(u64 逻辑分区,AND) + hold_time_ms(延迟,0=立即) + key(单键) + trigger_once + has_triggered(状态机)。
- PhysicalKeyboardMapping: gpio + key + trigger_level(ACTIVE_HIGH/LOW/AUTO)。LogicalKeyMapping.keys[3] 每键最多3键。
- AreaChannelMappingConfig::serial_mappings[34]: 区(0-33)→通道(u32,单通道位图,0xFFFFFFFF=未映射)。
- BindingState: IDLE→PREPARE→WAIT_TOUCH→PROCESSING(指触绑定)。

## v4 逻辑分区编号(与上位机 zone_label 对齐, 0..33)
A1-A8(0-7), B1-B8(8-15), C1-C2(16-17), D1-D8(18-25), E1-E8(26-33)。

## 分阶段实施(每阶段独立编译验证)

### Stage 1: 自动连接 + 分区绑定生效 + BIND_* 交互
- **自动连接**(上位机 main.rs): 启动即 connect(0)(若有设备); timer 里若 Disconnected 且有设备则节流(~2s)重连。用户不必手动点连接。
- **bind.mapNN 语义重定义**(v4 单 PSoC 36 通道, 24位掩码不够): u32 低字节=物理通道索引(0..35), 0xFFFFFFFF=未映射。(替换 v3.1 的 devmask<<24|24位掩码。)
- **game_io 消费绑定**: game_io 缓存 34 个通道索引(bind.map00..33), 每~200ms 从 ConfigManager 刷新; map_channels_to_areas 改为: area_bits |= ((touch_mask>>ch[area])&1) << area。取代固定 ZONE_CHANNEL_MAP。
- **BIND_* 固件 handler**(host_cmd 注册, 现全 NAK):
  - BIND_GET_MAP(0x43): 返回 34×u8 通道索引(0xFF=未映射)。
  - BIND_SET_MAP(0x44): payload=zone(u8)+channel(u8) → 写 bind.mapZZ = channel(或0xFFFFFFFF)。
  - BIND_START(0x40): payload=zone(u8) → 进入 WAIT_TOUCH; 记录该 zone; 固件在触控 task 里检测首个新激活通道→写 bind.map[zone]=ch, 发 BIND_EVENT(zone,ch,status=1完成), 回 IDLE。
  - BIND_ABORT(0x41): 取消绑定态。
  - BIND_EVENT(0x45): 设备→host, payload=zone(u8)+channel(u8)+status(u8)。
- 绑定状态机放 game_io 或新 binding 小服务(读 psoc->touch_mask)。

### Stage 2: 触摸区域可视化绑定 UI 页
- 用 Slint 几何绘制 maimai 圆形 34 分区(A外圈/B/C中心/D/E), 或嵌入区域图 png。点分区→选中; 显示该区绑定的通道; "指触绑定"按钮→BIND_START(zone); 列表展示全部绑定。
- 分区几何: 参考 触摸区域映射.jpg (圆形, 8扇区×环)。优先 Slint 画近似(圆按角度分 A/D 外圈交替, B/E 中圈, C 中心)。

### Stage 3: 触控→键盘映射解释器
- 新 LittleFS blob KeymapConfig: 规则数组(≤16条), 每条 {area_mask u64, hold_delay_ms u16, max_press_ms u16(0=不超时), flags u8(bit0 immediate), key_count u8, keys[6] u8(HID keycode)}。
- 固件解释器(每触控周期跑, 输入=逻辑34分区 area_bits): 每条规则状态机:
  - cond = (area_bits & area_mask)==area_mask (全部所需分区激活)。
  - 未按下&cond&(!locked): 若 immediate 或 已保持 hold_delay → press keys, 记 press_ts。
  - 已按下: 若 !cond → release keys(组合中抬起一个即 cond 假→release, 之后 cond 再真=再入场触发); 若 max_press_ms>0 且 now-press_ts>max_press → release + locked=true(结算)。
  - locked: 需 cond 变假再变真解锁(下次达到条件才计算)。
- HID 上报: 复用 v4 hid.cpp press_key/release_key。需 work_mode=HID 或与 serial 并存(确认 hid task 是否被调度)。
- host_cmd: KEYMAP_GET/KEYMAP_SET(新命令码, 用 0x40 域空位如 0x46/0x47, 或 0x60 新域)传 blob。UI 规则编辑器。

### Stage 4: 物理键盘 GPIO1-12
- config.h 查 GPIO1-12 是否占用(hardware.txt: GPIO1-12=Keyboard 1K上拉+二极管→按下拉低=ACTIVE_LOW)。
- 新服务 phys_kbd: 扫描 12 GPIO+统一去抖(ms), 每键配置 {keys[3] HID, mode(常开NO/常闭NC)}。常开=未按开路(上拉高), 按下拉低; 常闭反之。
- 配置: KeymapConfig blob 内附 12 键 或 独立 blob。
- HID 上报复用 hid.cpp。

## 待定/风险
- HID 与 Serial 并存: v4 启动按 mode.work 选描述符, HID 模式才有 HID 接口。键盘映射需 HID 接口存在。可能需 work_mode=HID 或描述符同时含 HID(需查 hal_usb 描述符)。这是键盘映射能否输出的前提, Stage3 前必须确认。
- bind.mapNN 语义变更影响现有上位机 binding helper(make_binding/binding_channel_mask 假设 devmask<<24|24位), 需同步改 Rust 侧。

## 进度
- [x] Stage1 自动连接+绑定生效+BIND_*
- [x] Stage2 可视化绑定UI(几何绘制圆形分区图,详见 phase3-stage2.md)
- [ ] Stage3 键盘映射解释器
- [ ] Stage4 物理键盘
