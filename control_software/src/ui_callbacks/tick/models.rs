{
    // 触控组合映射回填(version 门控)。
    let current_combo_version = ctrl.kbd_combo_version();
    if current_combo_version != last_combo_version {
        last_combo_version = current_combo_version;
        let table = ctrl.kbd_combos();
        let zone_names: Vec<String> = (0..34usize).map(zone_label).collect();
        let zones_text: Vec<slint::SharedString> = table.iter()
            .map(|c| {
                let names: Vec<&str> = (0..34usize)
                    .filter(|z| (c.zone_mask & (1u64 << z)) != 0)
                    .map(|z| zone_names[z].as_str())
                    .collect();
                slint::SharedString::from(names.join(" + "))
            })
            .collect();
        let keys_text: Vec<slint::SharedString> = table.iter()
            .map(|c| {
                let mut parts: Vec<String> = Vec::new();
                // 修饰位单独成段, 再列具体键 —— 与 kbd_display 的口径一致。
                for code in c.keycodes.iter().filter(|k| **k != 0) {
                    parts.push(kbd_display(*code, 0));
                }
                let base = if parts.is_empty() { "(仅修饰键)".to_string() } else { parts.join(" + ") };
                if c.modifiers != 0 {
                    slint::SharedString::from(format!("{} [{}]", base, kbd_display(0, c.modifiers)))
                } else {
                    slint::SharedString::from(base)
                }
            })
            .collect();
        let delays: Vec<i32> = table.iter().map(|c| c.delay_ms as i32).collect();
        let maxes: Vec<i32> = table.iter().map(|c| c.max_hold_ms as i32).collect();
        ui.set_combo_count(table.len() as i32);
        // 见 last_combo_zones 的说明: 内容没变就不换 model, 以保住行内键框的录制态。
        if zones_text != last_combo_zones {
            last_combo_zones = zones_text.clone();
            ui.set_combo_zones_text(slint::ModelRc::new(slint::VecModel::from(zones_text)));
        }
        ui.set_combo_keys_text(slint::ModelRc::new(slint::VecModel::from(keys_text)));
        ui.set_combo_delay(slint::ModelRc::new(slint::VecModel::from(delays)));
        ui.set_combo_max_hold(slint::ModelRc::new(slint::VecModel::from(maxes)));
        ui.set_combo_zone_used(slint::ModelRc::new(slint::VecModel::from(
            ctrl.kbd_combo_zone_used())));
        ui.set_combo_zone_picked(slint::ModelRc::new(slint::VecModel::from(
            ctrl.kbd_combo_pending_zones())));
        let (pk, pm) = ctrl.kbd_combo_pending_keys();
        let mut pending: Vec<String> = pk.iter().filter(|k| **k != 0)
            .map(|k| kbd_display(*k, 0)).collect();
        if pm != 0 {
            pending.push(format!("[{}]", kbd_display(0, pm)));
        }
        ui.set_combo_pending_keys_text(pending.join(" + ").into());
        // 设备支持性显式区分"旧固件不支持"与"支持但一条都没配", 不让前者被误读成配置丢了。
        let status = match ctrl.kbd_combo_supported() {
            Some(false) => "设备固件不支持组合映射(请更新固件)".to_string(),
            None => "尚未回读设备组合映射".to_string(),
            Some(true) => format!("上限 {} 条 · 单条最多 {} 键",
                mai2control_ui::proto::KBD_COMBO_COUNT,
                mai2control_ui::proto::KBD_COMBO_KEY_COUNT),
        };
        ui.set_combo_status(status.into());
    }

    // 批量应用抽屉回填: 勾选态与"每参数待写值"共用 batch_sel_version 门控, 值在 param_version
    // 推进/换通道时补空(★只补空, 不覆盖用户手改过的项★, 见 batch_fill_missing_values)。
    // 全部并入既有 16ms tick 门控, 不新增定时器、不提高频率。
    if ctrl.batch_sel_version() != last_batch_sel_version
        || current_param_version != last_param_version
        || channel_changed
        // 换算法后逐通道声明会变(项数/别名/默认值), 而上面三个门控都看不见 schema 的变化 ——
        // 不加这一条, 批量抽屉里的算法行会一直停在上一版算法的清单上。
        || ctrl.algo_schema_version() != last_batch_algo_schema_version
    {
        // 先补空再取版本号: 补空自身会 bump 版本, 顺序反了会多触发一次无谓的行模型重建。
        ctrl.batch_fill_missing_values();
        last_batch_sel_version = ctrl.batch_sel_version();
        ui.set_batch_source(ctrl.batch_source() as i32);
        ui.set_batch_ch_selected(slint::ModelRc::new(slint::VecModel::from(
            ctrl.batch_ch_selected())));
        ui.set_batch_param_selected(slint::ModelRc::new(slint::VecModel::from(
            ctrl.batch_param_selected())));
        // ★与单通道精调共用 build_param_row★: 围栏与单选项(时钟源 0..6+AUTO 等)只有一处派生,
        // 不会再出现"精调页是单选、批量面板还是数字框且能填非法值"这种两处不一致(实测截图)。
        // 值取的是 batch_source 的值而不是当前精调通道的 —— 抽屉里勾目标通道时不该改这一列。
        // -1 = 面板上尚无值(源通道未回读且用户未手填), .slint 侧显示"—", 区别于"值为 0"。
        let values = ctrl.batch_values();
        for (i, id) in BATCH_PARAM_IDS.iter().copied().enumerate() {
            let row = build_param_row(id, values.get(i).copied().unwrap_or(-1));
            // ★只在真的变了才写★ set_row_data 会通知 Repeater 更新该行, 无条件回写等于每
            // 16ms 把用户正在编辑的那一行推回设备值(GuardedSpinBox 有焦点守卫, 但没必要多此一举)。
            if batch_params_model.row_data(i).as_ref() != Some(&row) {
                batch_params_model.set_row_data(i, row);
            }
        }
        // ---- 逐通道算法配置(cfg_ch): 与上面的硬件参数**同款**参与批量设置 ----
        // 勾选决定写不写、值取自源通道且可手改、-1 显示"—"。三件事都与 batch_param_rows 一致,
        // 因为在用户眼里它就是"又一批每通道各一份的可调项"; 交互长得不一样等于宣布这一区规则另算。
        last_batch_algo_schema_version = ctrl.algo_schema_version();
        ui.set_batch_algo_ch_selected(slint::ModelRc::new(slint::VecModel::from(
            ctrl.batch_algo_ch_selected())));
        let algo_ch_values = ctrl.batch_algo_ch_values();
        let batch_algo_rows: Vec<AlgoSettingRow> = ctrl
            .algo_setting_decls()
            .into_iter()
            .filter(|decl| decl.per_channel)
            .map(|decl| {
                let (alias, description) = ctrl.algo_decl_text_named(
                    1,
                    decl.idx + mai2control_ui::proto::algo::ALGO_CFG_CH_META_BASE,
                    &decl.name,
                    &decl.alias,
                    &decl.description,
                );
                AlgoSettingRow {
                    idx: decl.idx as i32,
                    name: decl.name.into(),
                    default_val: decl.default as i32,
                    // ★这里不能用 algo_cfg_ch()★ 那个会回落到声明默认值; 批量面板写下去是要真发到
                    // 一批通道的, 拿一个从未与设备核对过的默认值冒充"源通道当前值"就是在覆盖用户
                    // 没打算改的东西。故用 batch_algo_ch_values 的 -1 哨兵如实显示"尚无值"。
                    value: algo_ch_values.get(decl.idx as usize).copied().unwrap_or(-1),
                    value_type: decl.value_type.into(),
                    range: decl.range.into(),
                    description: description.into(),
                    alias: alias.into(),
                    shared_scope: false,
                }
            })
            .collect();
        while batch_algo_ch_rows_model_timer.row_count() > batch_algo_rows.len() {
            batch_algo_ch_rows_model_timer
                .remove(batch_algo_ch_rows_model_timer.row_count() - 1);
        }
        for (row, setting) in batch_algo_rows.into_iter().enumerate() {
            if row < batch_algo_ch_rows_model_timer.row_count() {
                if batch_algo_ch_rows_model_timer.row_data(row).as_ref() != Some(&setting) {
                    batch_algo_ch_rows_model_timer.set_row_data(row, setting);
                }
            } else {
                batch_algo_ch_rows_model_timer.push(setting);
            }
        }
    }

    if current_param_version != last_param_version || channel_changed {
        last_param_version = current_param_version;
        let params = ctrl.params_of(current_channel as u8);
        let param_rows = build_param_rows(&params);
        while curve_params_model.row_count() > param_rows.len() {
            curve_params_model.remove(curve_params_model.row_count() - 1);
        }
        for (row, param) in param_rows.into_iter().enumerate() {
            if row < curve_params_model.row_count() {
                if curve_params_model.row_data(row).as_ref() != Some(&param) {
                    curve_params_model.set_row_data(row, param);
                }
            } else {
                curve_params_model.push(param);
            }
        }

        if let Some(finger_th) = ctrl.param(current_channel as u8, PARAM_FINGER_TH) {
            ui.set_finger_th_val(finger_th as i32);
        }
        if let Some(noise_th) = ctrl.param(current_channel as u8, PARAM_NOISE_TH) {
            ui.set_noise_th_val(noise_th as i32);
        }

        // 全局页的三项统一采样设置固定显示 CH0 代表值，不受精调通道影响。
        if let Some(value) = ctrl.param(0, PARAM_SNS_CLK_DIV) {
            ui.set_csd_sns_clk_div(value as i32);
        }
        // ★时钟树 snsClk 改显示全 36 通道范围★: 逐通道自适应后各通道分频不同, CH0 单值不反映任何情况。
        let (div_lo, div_hi) = ctrl.sns_clk_div_range();
        ui.set_csd_sns_clk_div_min(div_lo as i32);
        ui.set_csd_sns_clk_div_max(div_hi as i32);
        if let Some(value) = ctrl.param(0, PARAM_RESOLUTION) {
            ui.set_csd_resolution(value as i32);
        }
        if let Some(value) = ctrl.param(0, PARAM_SNS_CLK_SOURCE) {
            ui.set_csd_sns_clk_source(value as i32);
            // 下标由围栏统一判定: 128 ⇒ AUTO(末项), 非法值 ⇒ -1(下拉留空, 不假装选中某项)。
            ui.set_csd_clk_index(
                mai2control_ui::proto::param_fence(PARAM_SNS_CLK_SOURCE)
                    .ui_choice_index(value),
            );
        }
    }

    // 全通道实时状态：随遥测版本或 Cp 缓存版本刷新(遥测流全 36 通道；Cp 无新遥测时
    // 也可能因 CP_GET 响应更新，否则 cp_text 会卡在旧值不刷新)。active=status bit0。
    // 原地按行更新(仅变化的行才写)，不替换 model，避免元素重建导致的悬浮闪烁/交互丢失。
    let current_cp_version_all = ctrl.cp_version();
    // 排序/筛选选择与启用态(param_version 覆盖 0x0C 草稿)也要驱动重建: 否则改了下拉框或关掉
    // 一个通道后, 网格要等下一帧遥测才变, 看起来像"点了没反应"。
    let sort_all = ui.get_channel_sort();
    let show_disabled_all = ui.get_channel_show_disabled();
    let current_param_version_all = ctrl.param_version();
    let telemetry_state_all = channel_telemetry_state(&ctrl);
    // 同主图: 36 张卡片的格式化+逐行 diff 是另一处遥测驱动的重活, 同样限速到 visual_refresh_tick;
    // 排序/筛选/启用态变化(用户交互)仍立即生效, 不等下一个节拍。
    // 同主图: 36 张卡片只住在"设置"页, 其它页(主页/工具箱/日志/关于)不该为它付格式化与逐行 diff
    // 的代价(实测 ~0.38ms/次, 在主页也几乎每帧命中)。门控跳过时不更新 last_*, 回到该页时
    // 版本判据自然成立并立即重建一次, 不会出现"回来看到旧数据"。
    let channels_page_visible = ui.get_current_view() == 1;
    if channels_page_visible
        && ((current_telem_version != last_telem_version_all && visual_refresh_tick)
            || current_cp_version_all != last_cp_version_all
            || current_param_version_all != last_param_version_all
            || telemetry_state_all != last_channel_telemetry_state
            || sort_all != last_sort_all
            || show_disabled_all != last_show_disabled_all)
    {
        let prof_cards_start = Instant::now();
        last_telem_version_all = current_telem_version;
        last_cp_version_all = current_cp_version_all;
        last_param_version_all = current_param_version_all;
        last_channel_telemetry_state = telemetry_state_all;
        last_sort_all = sort_all;
        last_show_disabled_all = show_disabled_all;
        let (all, hidden) = build_channel_status(&ctrl, sort_all, show_disabled_all, telemetry_state_all);
        // 行数会随筛选变化: 数量变了必须整表替换(逐行 set_row_data 只能改已有行, 多出来的
        // 旧行会留在网格里变成幽灵卡片)。数量不变时仍只写变化的行, 保持无闪烁。
        if all_channels_model.row_count() != all.len() {
            all_channels_model.set_vec(all);
        } else {
            for (i, row) in all.into_iter().enumerate() {
                if all_channels_model.row_data(i).as_ref() != Some(&row) {
                    all_channels_model.set_row_data(i, row);
                }
            }
        }
        ui.set_channel_hidden_count(hidden);
        prof_cards_us += prof_cards_start.elapsed().as_micros() as u64;
        prof_cards_hits += 1;
    }
    // 采集开关(真停流)与画面冻结是两件事, 这里回填的是前者。
    ui.set_telem_paused(ctrl.telem_user_paused());
    // 单通道精调页的通道启用开关(草稿优先, 与网格卡片同一真相源)。
    ui.set_curve_ch_enabled(ctrl.ch_enabled(ui.get_sel_channel().clamp(0, 35) as u8));
}
