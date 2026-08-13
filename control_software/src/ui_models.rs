//! UI 派生模型与配置编辑模型。
//!
//! 本模块集中维护配置行、通道/绑区/HID/频谱/LED 行模型，
//! 以及这些模型共用的设备状态派生规则；回调注册只负责调用它们。

use super::*;

pub(crate) fn expected_scan_period_us(ctrl: &AppController) -> i32 {
    const MOD_CLK_MHZ: u64 = 48;
    const CHANNELS: u64 = 36;
    // 每通道固定开销(µs): 真机实测拟合值(传感切换/IMO 稳定/IDAC/IsBusy 轮询/多频扫描等,
    // 与分辨率/分频无关)。使预期贴近实测, 从而分频/分辨率外的突变可显现为真实异常。
    const FIXED_OVERHEAD_US_PER_CH: u64 = 76;
    let res = ctrl.param(0, PARAM_RESOLUTION).unwrap_or(0) as u64;
    if res < 1 || res > 20 {
        return 0;
    }
    let conv_us = (1u64 << res) / MOD_CLK_MHZ; // 换能: 2^res / ModClk(µs), 与 snsClkDiv 无关
    (CHANNELS * (FIXED_OVERHEAD_US_PER_CH + conv_us)).min(i32::MAX as u64) as i32
}

/// 一次待写入的配置草稿。仅在"控制器已被主循环借出"时短暂排队, 下一拍原样执行。
/// ★用枚举而不是装 boxed 闭包★ 闭包会把 `&mut AppController` 的借用期粘进队列元素里,
/// 而这里要表达的恰恰是"把参数存下来, 稍后再借"。
pub(crate) enum PendingCfgWrite {
    Bool(String, bool),
    Number(String, f64),
    Hex(String, String),
    Enum(String, i32),
    Str(String, String),
}

impl PendingCfgWrite {
    pub(crate) fn apply(self, ctrl: &mut AppController) {
        match self {
            PendingCfgWrite::Bool(key, value) => {
                let _ = ctrl.set_config(ConfigEntry::new(key, CfgValue::Bool(value)));
            }
            PendingCfgWrite::Number(key, value) => {
                let _ = ctrl.set_config_number(&key, value);
            }
            PendingCfgWrite::Hex(key, value) => {
                let _ = ctrl.set_config_hex(&key, &value);
            }
            PendingCfgWrite::Enum(key, index) => {
                let _ = ctrl.set_config_enum(&key, index);
            }
            PendingCfgWrite::Str(key, value) => {
                let _ = ctrl.set_config(ConfigEntry::new(key, CfgValue::Str(value)));
            }
        }
    }
}

/// 能借到控制器就立即写, 借不到就排队(绝不丢弃, 也绝不 panic)。
pub(crate) fn _apply_or_queue_cfg(
    controller: &Rc<RefCell<AppController>>,
    pending: &Rc<RefCell<Vec<PendingCfgWrite>>>,
    write: PendingCfgWrite,
) {
    match controller.try_borrow_mut() {
        Ok(mut ctrl) => write.apply(&mut ctrl),
        Err(_) => pending.borrow_mut().push(write),
    }
}

pub(crate) fn build_config_rows(entries: &[ConfigEntry]) -> Vec<ConfigRow> {
    // ★显式排序★: 入参来自按 key 排序的 BTreeMap, 直接渲染就是"英文键名字典序", 与中文标签的
    // 阅读逻辑完全无关。先按组内显式次序排, 同次序再按 key 兜底(保证稳定、不随 HashMap 抖动)。
    // 组之间的顺序由各页面挂 ConfigGroupSection 的先后决定, 不在这里管。
    let mut ordered: Vec<&ConfigEntry> = entries.iter().collect();
    ordered.sort_by(|a, b| {
        config_display_order(&a.key)
            .cmp(&config_display_order(&b.key))
            .then_with(|| a.key.cmp(&b.key))
    });
    ordered
        .into_iter()
        // ★渲染归属集中在这里★ 每个 KV 只允许出现在一个页面: 有专用编辑器的 key 一律在此滤掉,
        // 其余按 parse_config_label 给出的 group 决定落在"通信系统"Tab 还是"协议"页的对应块。
        // bind.map* → 分区绑定页; kbd.* → 键盘页; calib.* → 触控全局调整页的偏好滑条;
        // led.map* → 协议页 11 单元映射可视化编辑器(裸 KV 是打包 u32, 暴露出来只会被误改);
        // led.ws_count0/1 与 led.ws_brightness → 协议页 mai2light 块已有专用 SpinBox。
        .filter(|entry| {
            let k = entry.key.as_str();
            // ★只滤有专用编辑器的那一个 calib 键★ 原先按 `calib.` 整个前缀滤, 于是后加的三个
            // 启动校准开关(calib.boot_*)被连带滤掉 —— 协议页"启动校准与延迟补正"只剩 1 行,
            // 实测直方图正是 `"启动校准与延迟补正": 1`。前缀级黑名单挡住的是未来的键, 不是设计意图。
            !k.starts_with("bind.")
                && !k.starts_with("kbd.")
                // hid.enNN/xNN/yNN(108 项) → HID 触控点位页的截图锚定编辑器。
                // 裸 KV 是屏幕归一坐标, 手改数字既无参照也看不出落在屏幕哪里; 更要紧的是
                // 108 行会把"通信系统"配置表整体淹掉(该表原本约 60 行)。
                && !k.starts_with("hid.")
                && k != "calib.pref"
                && !k.starts_with("led.map")
                && !matches!(k, "led.ws_count0" | "led.ws_count1" | "led.ws_brightness")
        })
        .map(|entry| {
            let (
                mut kind,
                type_code,
                bool_val,
                num_val,
                mut min_val,
                mut max_val,
                mut has_range,
                mut enum_index,
                str_val,
            ) = match &entry.value {
                CfgValue::Bool(v) => (0, 0, *v, 0.0, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::U8(v) => (1, 2, false, *v as f32, 0.0, 255.0, true, 0, "".to_string()),
                CfgValue::U16(v) => (
                    1,
                    3,
                    false,
                    *v as f32,
                    0.0,
                    65535.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::U32(v) => (
                    1,
                    4,
                    false,
                    *v as f32,
                    0.0,
                    4294967295.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::I8(v) => (
                    1,
                    1,
                    false,
                    *v as f32,
                    -128.0,
                    127.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::F32(v) => (1, 5, false, *v, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::Str(v) => (3, 6, false, 0.0, 0.0, 0.0, false, 0, v.clone()),
            };

            // ★围栏必须用设备自报的 range, 不许按类型猜★
            // 上面那张表只知道"这是个 U16", 于是给出 0..65535; 而固件 schema 对同一个 key 声明的
            // 合法区间可能窄得多(例: 触控延迟只收 0..1000)。界面围栏比固件宽 = 放行必被 NAK 的值,
            // 用户失焦夹取也夹不到正确阈值上。schema 带 range 就以它为准, 否则才回落类型上限。
            if let Some((lo, hi)) = &entry.range {
                if let (Some(lo), Some(hi)) = (lo.as_f32(), hi.as_f32()) {
                    if hi >= lo {
                        min_val = lo;
                        max_val = hi;
                        has_range = true;
                    }
                }
            }

            // schema 未携带颜色枚举选项；这四个 U8 配置使用专用 ComboBox，
            // 但数值仍原样沿用 set_config_number → CfgValue::U8 的既有链路。
            if let CfgValue::U8(value) = &entry.value {
                if matches!(
                    entry.key.as_str(),
                    "led.color_connected"
                        | "led.color_flash_error"
                        | "led.color_link_error"
                        | "led.color_healthy"
                ) {
                    kind = 4;
                    enum_index = (*value).min(7) as i32;
                }
                // mode.work 是 U8 0..1(Serial/HID)枚举, 用 kind=2 ComboBox 而非数字输入。
                if entry.key.as_str() == "mode.work" {
                    kind = 2;
                    enum_index = (*value).min(1) as i32;
                }
            }

            let (group, label, desc) = parse_config_label(&entry.key);

            // 通信系统配置一律十进制展示(hex 口径仅用于需寄存器处理的 CSD 内容, 见触控全局调整)。
            // hex_val 保留为空(不再走 hex 输入); range_hex 复用为十进制取值范围小字, 便于新人上手。
            let is_num_int = kind == 1 && matches!(type_code, 1 | 2 | 3 | 4);
            let hex_val = String::new();
            let range_hex = if is_num_int && has_range {
                let lo = (min_val as f64).round() as i64;
                let hi = (max_val as f64).round() as i64;
                format!("取值范围 {}–{}", lo, hi)
            } else {
                String::new()
            };

            ConfigRow {
                key: entry.key.clone().into(),
                label: label.into(),
                desc: desc.into(),
                group: group.into(),
                kind,
                type_code,
                bool_val,
                num_val,
                min_val,
                max_val,
                has_range,
                enum_index,
                str_val: str_val.into(),
                hex_val: hex_val.into(),
                range_hex: range_hex.into(),
            }
        })
        .collect()
}

/// 造一行参数编辑数据(标签 + 围栏 + 单选项)。
/// ★单通道精调与全通道批量面板共用本函数★: 围栏与单选项只有这一处派生, 不许在 .slint 里按
/// param_id 硬编码 —— 那会成为与两处固件必然漂移的第三份定义(围栏当年就是这么漂移出
/// "SNS_CLK_SOURCE 上界写死 6 而设备合法持有 128"那个 bug 的)。
/// `value` 允许为 -1: 批量面板用它表示"面板上尚无值", 界面显示"—"。
pub(crate) fn build_param_row(param_id: u8, value: i32) -> ParamRow {
    let fence = mai2control_ui::proto::param_fence(param_id);
    let choices = fence.ui_choices();
    let choice_labels: Vec<slint::SharedString> =
        choices.iter().map(|(t, _)| t.as_str().into()).collect();
    let choice_values: Vec<i32> = choices.iter().map(|(_, v)| *v as i32).collect();
    ParamRow {
        param_id: param_id as i32,
        label: param_display_name(param_id).into(),
        value,
        min: fence.ui_min() as i32,
        max: fence.ui_max() as i32,
        choices: slint::ModelRc::new(slint::VecModel::from(choice_labels)),
        choice_values: slint::ModelRc::new(slint::VecModel::from(choice_values)),
        // 无值(-1)时不指向任何一项, 界面显示"—"而不是假装选中第一项。
        choice_index: if value < 0 {
            -1
        } else {
            fence.ui_choice_index(value as u32)
        },
        // 元数据与围栏同源(见 proto::telemetry::ParamFence 的 unit/scope/help/impact):
        // 界面只显示这两个字符串, 不按 param_id 自己拼说明 —— 那就是第二份元数据。
        unit: fence.unit.into(),
        help: fence.help_text(param_id).into(),
    }
}

/// 组内显示次序。★为什么必须显式给★
/// `config_entries()` 来自按 key 排序的 BTreeMap ⇒ 界面顺序 = **英文 key 的字典序**。
/// 于是协议页会排成"聚合延迟 → 额外重发 → 触控映射… → 速率上限 → 采样延迟 → 仅变化时发送 →
/// 触控串口波特率…"这种与阅读逻辑毫无关系的顺序(用户看到的是中文标签, 字典序按的却是英文键名)。
/// 这里给每个 key 一个显式次序: 先"这条链路是什么"(波特率/节点), 再"怎么发"(延迟), 最后"善后行为"。
/// 未收录的 key 一律排到末尾(999)并按 key 兜底, 新增 KV 不会插到中间打乱既有顺序。
pub(crate) fn config_display_order(key: &str) -> u16 {
    match key {
        // —— mai2serial: 链路 → 节流 → 延迟/聚合 → 发送策略 → RSET 善后 ——
        "comm.serial_baud" => 10,
        "comm.rate_limit_en" => 20,
        "comm.rate_limit_hz" => 21,
        "comm.touch_delay_100us" => 30,
        "comm.aggregation_delay_ms" => 40,
        "comm.send_only_on_change" => 50,
        "comm.extra_send" => 51,
        "comm.keyboard_map_serial_only" => 60,
        "comm.serial_reset_baseline" => 70,
        "comm.serial_reset_calibrate" => 71,
        // —— PSoC 每启动代次的一次性校准 + 主机端延迟补正 ——
        "calib.boot_idac" => 10,
        "calib.boot_channel" => 20,
        "calib.boot_baseline" => 30,
        "comm.latency_correction_en" => 40,
        // —— mai2light: 链路 → 节点 → 灯珠总数 ——
        "comm.light_baud" => 10,
        "led.node_id" => 20,
        "led.count" => 30,
        // —— 键盘映射 ——
        "comm.keyboard_map_en" => 10,
        "comm.keyboard_delay_100us" => 20,
        // —— 状态指示灯: 总开关 → 亮度 → 四个颜色 ——
        "led.enable" => 10,
        "led.status_brightness" => 20,
        "led.color_connected" => 30,
        "led.color_healthy" => 31,
        "led.color_link_error" => 32,
        "led.color_flash_error" => 33,
        // —— 工作模式 ——
        "mode.work" => 10,
        _ => 999,
    }
}

/// 返回 (分组中文名, 配置项中文名, 小字说明)。未收录的 key 回退到英文 key 的可读形式。
pub(crate) fn parse_config_label(key: &str) -> (String, String, String) {
    let parts: Vec<&str> = key.split('.').collect();
    // ★group 就是渲染位置★ 各 Slint 端的 ConfigGroupSection 用 group_title 精确匹配取行,
    // 所以归属按语义逐 key 指定, 而不是按 key 的一级前缀 —— comm./led. 前缀下同时混着
    // "协议能力参数"(属协议页)与"键盘映射/状态指示灯"(属通信系统 Tab), 前缀分不开。
    let group = match key {
        // mai2serial: 串口、节流、延迟聚合与 {E}RSET 善后 → 协议页 mai2serial 块。
        "comm.serial_baud"
        | "comm.touch_delay_100us"
        | "comm.aggregation_delay_ms"
        | "comm.extra_send"
        | "comm.rate_limit_en"
        | "comm.rate_limit_hz"
        | "comm.send_only_on_change"
        | "comm.serial_reset_calibrate"
        | "comm.serial_reset_baseline"
        // 「触控映射仅协议启动时生效」的判定依据就是 mai2serial 的实际发送态,
        // 语义上属协议能力而非键盘映射本身, 故归到协议页 mai2serial 块。
        | "comm.keyboard_map_serial_only" => "mai2serial 协议参数",
        // PSoC 启动流水线与主机侧补正开关都在协议页集中持久化。
        "calib.boot_idac"
        | "calib.boot_channel"
        | "calib.boot_baseline"
        | "comm.latency_correction_en" => "启动校准与延迟补正",
        // mai2light: 灯板串口 + 节点号 + 灯珠总数 → 协议页 mai2light 块
        "comm.light_baud" | "led.node_id" | "led.count" => "mai2light 协议参数",
        // 触控 → 键盘映射: 虽在 comm. 前缀下, 语义与协议无关
        "comm.keyboard_map_en" | "comm.keyboard_delay_100us" => "键盘映射",
        // 板载状态指示灯(main.cpp 心跳灯消费), 与灯板协议无关
        "led.enable"
        | "led.status_brightness"
        | "led.color_connected"
        | "led.color_flash_error"
        | "led.color_link_error"
        | "led.color_healthy" => "状态指示灯",
        "mode.work" => "工作模式",
        // 未收录 key 的兜底: 按前缀落到通信系统 Tab 的"其他"组, 不会凭空消失。
        _ => match parts.first().copied().unwrap_or("") {
            "bind" => "绑区",
            _ => "其他",
        },
    }
    .to_string();

    let (label, desc): (&str, &str) = match key {
        "comm.touch_delay_100us" => ("触控延迟 (×100µs)", "触控串口上报延迟线, 0..100ms"),
        "comm.aggregation_delay_ms" => (
            "多数投票窗口 (ms)",
            "对延迟后的触控状态按位多数投票；平票沿用上次结果",
        ),
        "comm.extra_send" => (
            "改变后额外重发",
            "仅改变时发送启用后，每个成功状态边沿额外重发的次数",
        ),
        "comm.rate_limit_en" => (
            "启用发送节流",
            "限制触控帧最高发送频率；节流不会制造重复状态边沿",
        ),
        "comm.rate_limit_hz" => ("发送频率上限 (Hz)", "启用发送节流时的最高成功发送频率"),
        "comm.send_only_on_change" => (
            "仅状态改变时发送",
            "关闭时连续发送；开启时仅成功发送新状态及其额外重发",
        ),
        "comm.keyboard_map_en" => ("启用触摸→键盘", "把触摸分区映射为键盘按键输出"),
        "comm.keyboard_map_serial_only" => (
            "触控映射仅协议启动时生效",
            "需先开启「启用触摸→键盘」；开启后触摸→键盘映射只在 mai2serial 实际发送触控数据期间生效，停发时映射失效并释放已按下的键",
        ),
        "comm.serial_baud" => ("触控串口波特率", "游戏触控串口 (COM) 的波特率"),
        "comm.serial_reset_calibrate" => (
            "串口重启后自动 IDAC 校准",
            "收到 mai2serial 重启指令({E} RSET)后，自动执行一次全通道 IDAC 校准",
        ),
        "comm.serial_reset_baseline" => (
            "串口重启后自动基线复位",
            "收到 mai2serial 重启指令({E} RSET)后，自动执行一次全通道基线复位",
        ),
        "calib.boot_idac" => (
            "启动时 IDAC 校准",
            "每颗 PSoC 完成 provisioning 且采样可信后，本启动代次只执行一次全通道 IDAC 校准",
        ),
        "calib.boot_channel" => (
            "启动时通道自适应",
            "IDAC 阶段结束后异步执行一次全通道频率自适应，不在失败时循环重试",
        ),
        "calib.boot_baseline" => ("启动时基线复位", "启动校准流水线末尾执行一次全通道基线复位"),
        "comm.latency_correction_en" => (
            "启用延迟样本补正",
            "用 0x20 三段最近非零且新鲜的实测值补齐异步窗口缺段；不包含也不改变触控延迟线",
        ),
        "comm.light_baud" => ("灯板串口波特率", "灯板通信串口的波特率"),
        "comm.keyboard_delay_100us" => ("键盘延迟 (×100µs)", "触摸→键盘输出的附加延迟"),
        "mode.work" => ("工作模式", "Serial=游戏串口; HID=触摸屏+键盘"),
        "led.enable" => ("启用 LED", "关闭则运行时 LED 静默(启动序列不受影响)"),
        "led.node_id" => ("灯节点 ID", "灯串协议的节点编号"),
        "led.count" => ("灯珠数量", "WS2812 灯珠个数"),
        "led.status_brightness" => ("状态亮度", "状态指示灯亮度，0=熄灭，255=最亮"),
        "led.color_connected" => ("已连接颜色", "主机连接正常时的状态灯颜色"),
        "led.color_flash_error" => ("Flash 错误颜色", "配置或存储错误时的状态灯颜色"),
        "led.color_link_error" => ("链路错误颜色", "PSoC 链路异常时的状态灯颜色"),
        "led.color_healthy" => ("健康颜色", "传感器与系统正常时的状态灯颜色"),
        _ => ("", ""),
    };

    let label = if label.is_empty() {
        parts.get(1).copied().unwrap_or(key).replace('_', " ")
    } else {
        label.to_string()
    };
    (group, label, desc.to_string())
}

/// 把 Cp(fF) 缓存值换算成两位小数 pF 文本，语义与曲线页一致：
/// None=本会话尚未测量(无任何自动获取)，Some(0)=设备尚未给出结果(测量中)，
/// Some(CP_MEASURE_FAILED)=★唯一的失败判据★(MEASURE_CP 后设备回读 0x00FFFFFF)，其余=正常测量值。
///
/// ★不从别处推断 Cp 失败★: 非激活电极接法、railed 通道数、采样率这些都不是 Cp 失败的证据,
/// 历史上按它们猜出来的警告只会把用户引到错误的排查方向。
pub(crate) fn cp_display_text(cp: Option<u32>) -> String {
    match cp {
        None => "未测量".to_string(),
        Some(0) => "测量中…".to_string(),
        Some(CP_MEASURE_FAILED) => CP_FAILURE_TEXT.to_string(),
        Some(value) => format!("{:.2} pF", value as f64 / 1000.0),
    }
}

/// Cp 失败的**唯一**文案。单通道显示、批量汇总、全局调整页警告三处共用同一句,
/// 免得同一个故障在三处被描述成三种不同的原因。
pub(crate) const CP_FAILURE_TEXT: &str = "测量失败：可能短接或电容过大";

/// 收集 Cp 失败通道(判据只有 `CP_MEASURE_FAILED`), 返回形如 `CH0 CH5` 的列表文案。
/// None/0(测量中)/GND 接法/raw 满量程/frozen/采样率一律不参与 —— 它们不是 Cp 失败的证据。
pub(crate) fn cp_failed_list(ctrl: &AppController) -> String {
    (0..36u8)
        .filter(|&ch| ctrl.cp(ch) == Some(CP_MEASURE_FAILED))
        .map(|ch| format!("CH{}", ch))
        .collect::<Vec<_>>()
        .join(" ")
}

/// 绑区页 34 分区静态几何 + 实时绑定/触摸/Cp 状态，几何来自 DXF 生成的
/// [`touch_geometry::ZONE_GEOMETRY`]，坐标系固定为 SCREEN_W x SCREEN_H。
pub(crate) fn build_hid_point_cells(ctrl: &AppController) -> Vec<HidPointCell> {
    (0..HID_POINT_COUNT)
        .map(|channel| {
            let (x, y) = ctrl.hid_point_xy(channel);
            let enabled = ctrl.hid_point_enabled(channel);
            let touched = (ctrl.active_ch_mask() & (1u64 << channel)) != 0;
            HidPointCell {
                channel: channel as i32,
                label: format!("CH{}", channel).into(),
                enabled,
                x: x as i32,
                y: y as i32,
                coord_text: if enabled {
                    format!("{}, {}", x, y).into()
                } else {
                    "未启用".into()
                },
                touched,
            }
        })
        .collect()
}

/// 抓取主屏并转换为 Slint RGB8 图像。失败时保留上一帧，避免误把黑图作为新的锚定参照。
pub(crate) fn refresh_hid_screenshot(ui: &AppWindow) {
    match mai2control_ui::screen_capture::capture_primary_screen() {
        Ok(frame) => {
            let mut pixels =
                slint::SharedPixelBuffer::<slint::Rgb8Pixel>::new(frame.width, frame.height);
            let dst = pixels.make_mut_bytes();
            if dst.len() == frame.rgb.len() {
                dst.copy_from_slice(&frame.rgb);
                ui.set_hid_screenshot(slint::Image::from_rgb8(pixels));
                ui.set_hid_screenshot_width(frame.width as i32);
                ui.set_hid_screenshot_height(frame.height as i32);
            } else {
                log::warn!(
                    "HID 触控点位: 截图缓冲长度不一致({} != {})",
                    dst.len(),
                    frame.rgb.len()
                );
            }
        }
        Err(error) => log::warn!("HID 触控点位: 主屏截图失败: {}", error),
    }
}

pub(crate) fn build_zone_cells(ctrl: &AppController) -> Vec<ZoneCell> {
    let waiting_zone = ctrl
        .bind_progress()
        .and_then(|(zone, status)| (status == 0).then_some(zone));
    let mut cells = Vec::with_capacity(34);
    for i in 0..34usize {
        let label = zone_label(i);
        let ring = label.chars().next().unwrap_or('?').to_string();
        let ch = ctrl.binding_channel_of(i);
        let channel = if ch == 0xFF { -1 } else { ch as i32 };
        let touched = channel >= 0
            && ctrl
                .telem_latest(channel as u8)
                .and_then(|sample| sample.status)
                .map(|status| (status & 0x01) != 0)
                .unwrap_or(false);
        let geometry = &touch_geometry::ZONE_GEOMETRY[i];
        let cp_text = if channel >= 0 {
            cp_display_text(ctrl.cp(channel as u8))
        } else {
            "未绑定".to_string()
        };

        cells.push(ZoneCell {
            index: i as i32,
            label: label.into(),
            ring: ring.into(),
            channel,
            binding_text: if channel >= 0 {
                format!("CH{}", channel).into()
            } else {
                "未绑定".into()
            },
            cp_text: cp_text.into(),
            touched,
            binding_active: waiting_zone == Some(i as u8),
            path: geometry.path.into(),
            path_x: geometry.min_x,
            path_y: geometry.min_y,
            path_width: geometry.width,
            path_height: geometry.height,
            label_x: geometry.label_x,
            label_y: geometry.label_y,
        });
    }
    cells
}

/// 全通道网格的行模型。★排序与筛选都在这里定稿★: 次序取自
/// `AppController::channel_display_order`(逻辑序的唯一来源是绑定映射), Slint 只按数组下标摆格子。
/// 返回 `(行, 被筛掉的通道数)`。
/// 链路状态只复用 AppController 的现有活性证据：断开立刻未知，连接后无新鲜证据才过期。
pub(crate) fn channel_telemetry_state(ctrl: &AppController) -> i32 {
    if ctrl.state() != ConnState::Connected {
        2
    } else if ctrl.psoc_link_alive() {
        0
    } else {
        1
    }
}

pub(crate) fn build_channel_status(
    ctrl: &AppController,
    sort: i32,
    show_disabled: bool,
    telemetry_state: i32,
) -> (Vec<ChannelStatus>, i32) {
    let (order, hidden) = ctrl.channel_display_order(sort, show_disabled);
    let mut out = Vec::with_capacity(order.len());
    for (pos, ch) in order.into_iter().enumerate() {
        let (active, raw, diff) = match ctrl.telem_latest(ch) {
            Some(s) => (
                (s.status.unwrap_or(0) & 0x01) != 0,
                s.raw.unwrap_or(0) as i32,
                s.diff.unwrap_or(0) as i32,
            ),
            None => (false, 0, 0),
        };
        let bindings: Vec<String> = (0..34usize)
            .rev()
            .filter(|&zone| ctrl.binding_channel_of(zone) == ch)
            .map(zone_label)
            .collect();
        let binding_text = if bindings.is_empty() {
            "未绑定".to_string()
        } else {
            bindings.join(" ")
        };

        out.push(ChannelStatus {
            index: ch as i32,
            label: format!("CH{}", ch).into(),
            active,
            raw,
            diff,
            // 网格坐标按【展示次序】给出(不是物理号): 排序后按物理号摆格会与实际渲染次序错位。
            grid_col: (pos % 6) as i32,
            grid_row: (pos / 6) as i32,
            binding_text: binding_text.into(),
            cp_text: cp_display_text(ctrl.cp(ch)).into(),
            frozen: ctrl.channel_frozen(ch),
            enabled: ctrl.ch_enabled(ch),
            order: pos as i32,
            telemetry_state,
        });
    }
    (out, hidden as i32)
}

pub(crate) fn build_param_rows(params: &[(u8, u32)]) -> Vec<ParamRow> {
    params
        .iter()
        // 单通道精调展示全部由 CSD 模式接管的逐通道参数；AUTO 时 Slint 统一显示 AUTO 并锁定，
        // SEMI 时恢复数值与编辑，避免自动实时值被误当作用户的手动设置。
        .filter(|&&(param_id, _)| matches!(param_id, 0x01..=0x06 | 0x08..=0x0B))
        .map(|&(param_id, value)| {
            // 围栏随行下发: Slint 只消费数值, 不再自己按 param_id 派生阈值(见 types.slint::ParamRow)。
            build_param_row(param_id, value as i32)
        })
        .collect()
}

/// 响应噪声频谱的热图格子模型(SWEEP_GAINS × SWEEP_DIVS 格)。
///
/// ★归一化与配色必须在 Rust 侧算★ Slint 数不出"全体有效格的最小/最大值", 而热图的全部意义在于
/// 相对高低; 若把原始标准差丢给 .slint 让它自己上色, 那边只能写死一个绝对量程 —— 换个通道或换个
/// 分辨率就整片糊成一个颜色。
/// ★撞顶/贴底的格子不参与 min/max★: 那种点 RAW 贴在量程边界上, 标准差小得好看但 diff 已经失效,
/// 让它参与归一化会把整张图的基准拉歪。它们仍然画出来, 由 .slint 侧描红边区别标注。
pub(crate) fn build_spectrum_cells(ctrl: &AppController) -> Vec<SpectrumCell> {
    let cells = ctrl.noise_sweep_cells();
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for cell in cells {
        if !cell.valid || cell.railed {
            continue;
        }
        lo = lo.min(cell.std);
        hi = hi.max(cell.std);
    }
    let span = (hi - lo).max(1e-6);
    cells
        .iter()
        .enumerate()
        .map(|(i, cell)| {
            let norm = if cell.valid && hi >= lo {
                ((cell.std - lo) / span).clamp(0.0, 1.0)
            } else {
                0.0
            };
            SpectrumCell {
                gain: (i / SWEEP_DIVS) as i32,
                div: (i % SWEEP_DIVS + 1) as i32,
                std: cell.std,
                pp: cell.pp,
                norm,
                valid: cell.valid,
                railed: cell.railed,
                tint: _spectrum_tint(cell.valid, cell.railed, norm),
            }
        })
        .collect()
}

/// 一格的颜色: 未测=底色, 撞顶=暗红(其"低噪声"不可信), 其余按噪声由低到高 蓝→绿→红。
/// 分两段线性插值而不是简单 R/B 对调: 中间层次(绿)才让人看出"哪一片是可用区"。
pub(crate) fn _spectrum_tint(valid: bool, railed: bool, norm: f32) -> slint::Color {
    if !valid {
        return slint::Color::from_rgb_u8(0x1a, 0x1f, 0x24);
    }
    if railed {
        return slint::Color::from_rgb_u8(0x4a, 0x2a, 0x2a);
    }
    let t = norm.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.5 {
        let u = t / 0.5;
        (
            _lerp8(0x20, 0x30, u),
            _lerp8(0x60, 0xd0, u),
            _lerp8(0xd0, 0x50, u),
        )
    } else {
        let u = (t - 0.5) / 0.5;
        (
            _lerp8(0x30, 0xe0, u),
            _lerp8(0xd0, 0x40, u),
            _lerp8(0x50, 0x30, u),
        )
    };
    slint::Color::from_rgb_u8(r, g, b)
}

pub(crate) fn _lerp8(a: u8, b: u8, t: f32) -> u8 {
    (a as f32 + (b as f32 - a as f32) * t.clamp(0.0, 1.0)).round() as u8
}

/// 全局项(GPARAM_*)的数字输入围栏, 供触控全局调整页的 SpinBox 取上下界。
///
/// ★围栏唯一来源 = `proto::telemetry::global_fence`★(与 RP2040 `_handle_global_set`、
/// PSoC `cmd_set_global` 同源, 见该文件该节头注释)。slint 侧只消费数值, 不按 gparam_id
/// 自己派生阈值 —— 那会变成与两处固件必然漂移的第三份围栏。
/// GPARAM_INACTIVE_SNS(0x01) 取值离散({1,2,4}), 由 ComboBox 表达, 不需要数值上下界。
pub(crate) fn build_global_fences() -> GlobalFences {
    use mai2control_ui::proto::algo as ga;
    use mai2control_ui::proto::global_fence;
    let gain = global_fence(ga::GPARAM_IDAC_GAIN_INIT);
    let imin = global_fence(ga::GPARAM_IDAC_MIN);
    let tgt = global_fence(ga::GPARAM_RAW_TARGET);
    let f1 = global_fence(ga::GPARAM_MFS_DIV_F1);
    let f2 = global_fence(ga::GPARAM_MFS_DIV_F2);
    GlobalFences {
        idac_gain_min: gain.ui_min() as i32,
        idac_gain_max: gain.ui_max() as i32,
        idac_min_min: imin.ui_min() as i32,
        idac_min_max: imin.ui_max() as i32,
        raw_target_min: tgt.ui_min() as i32,
        raw_target_max: tgt.ui_max() as i32,
        mfs_f1_min: f1.ui_min() as i32,
        mfs_f1_max: f1.ui_max() as i32,
        mfs_f2_min: f2.ui_min() as i32,
        mfs_f2_max: f2.ui_max() as i32,
    }
}

/// per-channel 参数的中文显示名。单一真相源: 单通道精调的参数行与全通道页的批量应用面板
/// 共用本函数, 避免两处各写一份 match 而出现名称漂移。
pub(crate) fn param_display_name(param_id: u8) -> String {
    match param_id {
        0x01 => "手指阈值(PARAM_FINGER_TH)".to_string(),
        0x02 => "噪声阈值(PARAM_NOISE_TH)".to_string(),
        0x03 => "负阈值(PARAM_NEG_NOISE_TH)".to_string(),
        0x04 => "迟滞(PARAM_HYSTERESIS)".to_string(),
        0x05 => "按键消抖(PARAM_ON_DEBOUNCE)".to_string(),
        0x06 => "低基线复位(PARAM_LOW_BSLN_RST)".to_string(),
        0x07 => "分辨率(PARAM_RESOLUTION)".to_string(),
        0x08 => "传感时钟分频(PARAM_SNS_CLK_DIV)".to_string(),
        0x09 => "模态 IDAC(PARAM_IDAC_MOD)".to_string(),
        0x0A => "时钟源(PARAM_SNS_CLK_SOURCE)".to_string(),
        0x0B => "IDAC 增幅档(PARAM_IDAC_GAIN)".to_string(),
        _ => format!("参数 0x{:02X}", param_id),
    }
}

/// 主页"工作模式"一行文案。草稿优先(与通信系统 Tab 的 ComboBox 同源), 未回读到 mode.work
/// 就显示"未知" —— 拿默认值 0 冒充设备真值会让人以为设备在 Serial 模式。
/// 草稿与设备值不同时追加提示: 该项要整机重启重枚举才生效, 复用 draft_needs_reboot 判定。
pub(crate) fn work_mode_text(ctrl: &AppController) -> String {
    let base = match ctrl.work_mode() {
        Some(0) => "Serial (mai2serial + mai2light 双 CDC)",
        Some(1) => "HID (键盘 + 触摸屏)",
        Some(_) => "未知(设备返回了未定义值)",
        None => "未知",
    };
    if ctrl.draft_needs_reboot() {
        format!("{}（未保存，保存后重启生效）", base)
    } else {
        base.to_string()
    }
}

/// 读取数值型配置 KV(草稿优先, 由 config_get 保证); 未回读到则给默认值。
/// 各数值变体统一折成 u32, 免得每个调用点再 match 一遍 CfgValue。
pub(crate) fn cfg_u32_or(ctrl: &AppController, key: &str, default: u32) -> u32 {
    match ctrl.config_get(key).map(|e| e.value) {
        Some(CfgValue::U8(v)) => v as u32,
        Some(CfgValue::I8(v)) => v.max(0) as u32,
        Some(CfgValue::U16(v)) => v as u32,
        Some(CfgValue::U32(v)) => v,
        Some(CfgValue::F32(v)) => v.max(0.0) as u32,
        Some(CfgValue::Bool(v)) => u32::from(v),
        _ => default,
    }
}

/// 虚拟 LED 单元语义标签: 0..7 为按键灯(经灯板协议缓冲提交), 8/9/10 为白灯直刷。
pub(crate) fn led_unit_label(unit: usize) -> String {
    match unit {
        0..=7 => format!("{} 按键灯{}", unit, unit + 1),
        8 => "8 Body 白灯".to_string(),
        9 => "9 Ext 白灯".to_string(),
        10 => "10 Side 白灯".to_string(),
        _ => format!("{} ?", unit),
    }
}

/// 协议页 11 个虚拟 LED 单元行: 采样色(LED_GET 回报的当前有效色)+ 映射(草稿优先)。
pub(crate) fn build_led_unit_rows(ctrl: &AppController) -> Vec<LedUnitRow> {
    (0..LED_UNIT_COUNT)
        .map(|unit| {
            let rgb = ctrl.led_color(unit);
            let region = ctrl.led_region(unit);
            LedUnitRow {
                unit: unit as i32,
                label: led_unit_label(unit).into(),
                sample: slint::Color::from_rgb_u8(rgb[0], rgb[1], rgb[2]),
                rgb_text: format!("R{} G{} B{}", rgb[0], rgb[1], rgb[2]).into(),
                ch_choice: if region.ch > 1 {
                    0
                } else {
                    region.ch as i32 + 1
                },
                start: region.start as i32,
                count: region.count as i32,
            }
        })
        .collect()
}
