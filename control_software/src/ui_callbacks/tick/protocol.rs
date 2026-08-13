{
    // mai2serial 发送状态(version 门控): 未读取时显式未知，不能以 false/0 冒充设备真值。
    if ctrl.mai2_version() != last_mai2_version {
        last_mai2_version = ctrl.mai2_version();
        let send_en = ctrl.mai2_send_en();
        ui.set_mai2_send_en_known(send_en.is_some());
        ui.set_mai2_send_en(send_en.unwrap_or(false));
        let status = match ctrl.mai2_status() {
            Some(0) => "停",
            Some(1) => "就绪",
            Some(2) => "运行",
            _ => "未知",
        };
        ui.set_mai2_status(status.into());
        let baud = ctrl
            .mai2_baud()
            .map(|value| format!("{} bps", value))
            .unwrap_or_else(|| "未知".to_string());
        ui.set_mai2_baud(baud.into());
    }
    // mai2light 灯效运行态(version 门控): 回读/草稿编辑/应用结果任一变化才重建 11 行。
    if ctrl.led_version() != last_led_version {
        last_led_version = ctrl.led_version();
        ui.set_light_known(ctrl.led_known());
        ui.set_light_status(match ctrl.led_status() {
            Some(0) => "停",
            Some(1) => "就绪",
            Some(2) => "运行",
            _ => "未知",
        }.into());
        // 两链就绪位与故障分档来自同一状态字高位: 未回读时一律 false/"", UI 显示"未知"而非绿灯。
        ui.set_light_chain0_ready(ctrl.led_chain_ready(0).unwrap_or(false));
        ui.set_light_chain1_ready(ctrl.led_chain_ready(1).unwrap_or(false));
        ui.set_light_init_fault(match ctrl.led_init_fault() {
            Some(1) => "PIO1 初始化失败",
            Some(2) => "灯链0 建链失败(PIO 程序内存/状态机)",
            Some(3) => "灯链1 建链失败(PIO 程序内存/状态机)",
            _ => "",
        }.into());
        ui.set_light_resp_en(match ctrl.led_resp_enabled() {
            Some(true) => "已使能",
            Some(false) => "未使能",
            None => "未知",
        }.into());
        let baud = if ctrl.led_baud_valid() {
            format!("{} bps", ctrl.led_baud().unwrap_or_default())
        } else {
            "未知".to_string()
        };
        ui.set_light_baud(baud.into());
        let rx_frames = if ctrl.led_rx_frames_valid() {
            ctrl.led_rx_frames().to_string()
        } else {
            "未知".to_string()
        };
        ui.set_light_rx_frames(rx_frames.into());
        let sum_errors = if ctrl.led_sum_errors_valid() {
            ctrl.led_sum_errors().to_string()
        } else {
            "未知".to_string()
        };
        ui.set_light_sum_errors(sum_errors.into());
        // 就地更新常驻模型(实例在 tick/mod.rs 建好并绑定)。绝不换 ModelRc: 换实例会销毁
        // 行内的通道下拉与起始/数量数字框, 让正在输入的用户被动失焦(约 1s 一次回读即触发)。
        let unit_rows = build_led_unit_rows(&ctrl);
        while light_units_model.row_count() > unit_rows.len() {
            light_units_model.remove(light_units_model.row_count() - 1);
        }
        for (row, unit_row) in unit_rows.into_iter().enumerate() {
            if row < light_units_model.row_count() {
                if light_units_model.row_data(row).as_ref() != Some(&unit_row) {
                    light_units_model.set_row_data(row, unit_row);
                }
            } else {
                light_units_model.push(unit_row);
            }
        }
        ui.set_light_conflict(ctrl.led_region_conflict().unwrap_or_default().into());
        let expected_brightness = cfg_u32_or(&ctrl, "led.ws_brightness", 0).clamp(0, 255) as u8;
        let brightness_status = match ctrl.led_applied_brightness() {
            Some(actual) if actual == expected_brightness => {
                format!("亮度已由设备应用: {}", actual)
            }
            Some(actual) => format!(
                "亮度待应用: 配置 {}，设备实际 {}",
                expected_brightness, actual
            ),
            None => "亮度应用状态未知（旧固件或尚未回读）".to_string(),
        };
        let apply_status = ctrl.led_apply_status();
        let apply_status = if apply_status.is_empty() {
            brightness_status
        } else {
            format!("{} · {}", apply_status, brightness_status)
        };
        ui.set_light_apply_status(apply_status.into());
    }
    // 灯链长度/亮度取配置 KV(草稿优先), 与 calib_pref 同口径每帧回填。
    ui.set_light_ws_count0(cfg_u32_or(&ctrl, "led.ws_count0", 0).clamp(0, 1000) as i32);
    ui.set_light_ws_count1(cfg_u32_or(&ctrl, "led.ws_count1", 0).clamp(0, 1000) as i32);
    ui.set_light_brightness(cfg_u32_or(&ctrl, "led.ws_brightness", 0).clamp(0, 255) as i32);
}
