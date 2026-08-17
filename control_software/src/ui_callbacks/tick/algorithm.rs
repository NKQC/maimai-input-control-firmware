{
    // 后台编译产物回收: try_recv 不阻塞, 拿到结果才写状态(Rc<RefCell<AppController>> 只在本线程碰)。
    // 成功且要求上传时当帧续走 upload_compiled(其内部走 mpsc, 不阻塞 UI)。
    {
        let finished = match algo_job_timer.borrow().as_ref() {
            Some(job) => match job.rx.try_recv() {
                Ok(result) => Some(result),
                Err(std::sync::mpsc::TryRecvError::Empty) => None,
                // 线程 panic 才会断连: 当作一次失败结束, 否则 busy 会永久卡住。
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    Some(Err(anyhow::anyhow!("编译线程异常退出")))
                }
            },
            None => None,
        };
        if let Some(result) = finished {
            let job = algo_job_timer.borrow_mut().take();
            // ★分母用设备回报的槽容量★(未回读到时是内置兜底值, 见 algo_caps_known)。
            let cap = ctrl.algo_slot_capacity();
            match (result, job) {
                (Ok(out), Some(job)) => {
                    let len = ctrl.apply_compiled(&job.src, out);
                    ui.set_algo_asm_bytes(len as i32);
                    let pct = (len * 100) / cap;
                    if job.upload_after {
                        ui.set_algo_phase("上传中…".into());
                        match ctrl.upload_compiled() {
                            Ok(()) => ui.set_algo_status(format!(
                                "编译成功: ASM {} / {} 字节 ({}%); {}",
                                len, cap, pct, ctrl.algo_upload_status()).into()),
                            Err(error) => {
                                ctrl.push_log_warn(format!("算法: 编译成功但上传失败: {}", error));
                                ui.set_algo_status(format!("编译成功但上传失败: {}", error).into());
                            }
                        }
                    } else {
                        ui.set_algo_status(format!(
                            "编译成功: ASM {} / {} 字节 ({}%), 可上传", len, cap, pct).into());
                    }
                }
                (Err(error), job) => {
                    ui.set_algo_asm_bytes(0);
                    ctrl.push_log_error(format!("算法: 编译失败: {}", error));
                    let tail = if job.map(|j| j.upload_after).unwrap_or(false) { "(未上传)" } else { "" };
                    ui.set_algo_status(format!("编译失败{}: {}", tail, error).into());
                }
                // 结果与任务槽必然同生同灭, 该分支不可达; 兜底解锁避免 busy 悬挂。
                (Ok(_), None) => {}
            }
            ui.set_algo_busy(false);
            ui.set_algo_phase("".into());
            algo_busy_timer.set(false);
        }
    }

    // 上传状态只在对应 seq 的 ACK/NAK 或无回执超时时变化，避免"已发送"被误显示为成功。
    if ctrl.algo_upload_version() != last_algo_upload_version {
        last_algo_upload_version = ctrl.algo_upload_version();
        ui.set_algo_status(ctrl.algo_upload_status().into());
    }

    // 算法信息回填(version/首次填充门控): 首次 tick 即显示明确状态，首次响应后必然回填。
    let algo_version = ctrl.algo_version();
    let algo_page_visible = ctrl.state() == ConnState::Connected
        && ui.get_current_view() == 1
        && ui.get_settings_tab() == SETTINGS_PAGE_ALGO;
    if algo_version != last_algo_version
        || !algo_info_filled
        || algo_page_visible && !last_algo_page_visible
        || ctrl.algo_caps_known() != last_algo_caps_known
    {
        last_algo_version = algo_version;
        last_algo_caps_known = ctrl.algo_caps_known();
        let txt = match ctrl.algo_info() {
            Some(i) => {
                let mut txt = format!(
                    "当前算法: {} | PSoC valid={} | len={}B | crc16=0x{:04X}",
                    if i.is_default { "默认(v3.1 HDR)" } else { "自定义" },
                    i.psoc_valid, i.len, i.crc16
                );
                // ★隔离态绝不能显示成正常运行★ 此刻 PSoC 跑的是原生 CapSense 判定, 算法与 C 源
                // 仍在 flash 可回读, 但一个字节都没在执行。上一行的 valid/len/crc 全都"看起来正常",
                // 只有这一句能把"算法在跑"与"算法被摘掉了"分开。
                if i.extended && i.quarantined {
                    txt.push_str(
                        " ‖ 算法已被设备隔离(连续致命), PSoC 正在跑原生判定; 重新上传或点救援即解除",
                    );
                }
                // "已受理"不等于"已生效": 存进设备但还没下发到 PSoC 的状态必须如实写出。
                if i.extended && i.download_pending {
                    txt.push_str(" ‖ 已存入设备, 等待下发");
                }
                txt
            }
            None if ctrl.state() == ConnState::Disconnected => "算法信息未读取（未连接）".to_string(),
            None => "算法信息未读取".to_string(),
        };
        ui.set_algo_info_text(txt.into());
        algo_info_filled = true;

        // ---- 容量与堆占用: 一律取自设备 ----
        // ★未回报时显示 "—" 而不是 0★(本仓既有约定: 0 与"未知"必须可区分)。堆占用用 -1 当哨兵,
        // 容量则回填"当前生效口径"(设备值或兜底值)并另用 algo_caps_known 让界面注明"容量待回读"。
        let caps_known = ctrl.algo_caps_known();
        ui.set_algo_caps_known(caps_known);
        ui.set_algo_asm_capacity(ctrl.algo_slot_capacity() as i32);
        ui.set_algo_upload_limit(ctrl.algo_upload_limit() as i32);
        ui.set_algo_c_capacity(ctrl.algo_src_capacity() as i32);
        ui.set_algo_heap_capacity(ctrl.algo_heap_capacity() as i32);
        ui.set_algo_heap_used(match ctrl.algo_info() {
            // heap_used 是算法自报用量的峰值; 只有扩展响应才带它, 旧固件下"不知道"必须是 -1。
            Some(i) if i.extended => i.heap_used as i32,
            _ => -1,
        });
        ui.set_algo_caps_warning(
            if ctrl.algo_caps_mismatch() {
                AppController::algo_caps_mismatch_text(
                    ctrl.algo_slot_capacity(),
                    ctrl.algo_upload_limit(),
                )
            } else {
                String::new()
            }
            .into(),
        );
    }
    last_algo_page_visible = algo_page_visible;
    // 设备映射表 C 源回读(version 门控)→ 把"当前算法"的可编辑 C 载入编辑器:
    // 设备存了源(上传时随附)→ 精确还原可修改; 设备无源且为默认算法 → 载入内嵌默认模板;
    // 无源且自定义(旧固件/异常)→ 提示并保留编辑器。全程无需机器码反汇编。
    if ctrl.algo_device_src_version() != last_algo_src_version {
        last_algo_src_version = ctrl.algo_device_src_version();
        let dev_src = ctrl.algo_device_src().to_string();
        // ★不覆盖用户正在编辑的文本★: 编辑器只在"空"或"内容仍是上次自动载入的那份"时才自动填。
        // schema/面板已与编辑器解耦(走 algo_schema_source), 所以这里不填也不影响算法面板。
        let editor_now = ui.get_algo_c_source().to_string();
        // ★显式回读无条件覆盖★: 用户点"读取信息"、以及连接后的首次自动同步, 意图都是"把设备里
        // 真正存的那份取回来看"。此时保留草稿等于答非所问; 而 editor_untouched 还会把程序内置的
        // 初始示例算法误判成"用户的改动", 于是连接后编辑器永远停在示例上、不与设备同步。
        // 自动回读(版本变化触发的那种)仍然保护草稿, 语义不变。
        let explicit_readback = ctrl.take_algo_explicit_readback_src();
        let editor_untouched = explicit_readback
            || editor_now.trim().is_empty()
            || editor_now == editor_autoload_mark;
        if !dev_src.trim().is_empty() {
            if editor_untouched {
                ui.set_algo_c_source(dev_src.clone().into());
                ui.set_algo_line_numbers(line_numbers_for(&dev_src).into());
                ui.set_algo_c_bytes(AppController::algo_src_used(&dev_src) as i32);
                editor_autoload_mark = dev_src.clone();
                ui.set_algo_status("已从设备映射表载入当前算法 C 源(可直接修改后重新编译上传)".into());
            } else {
                ui.set_algo_status(
                    "已回读设备算法 C 源(算法面板按设备口径刷新); 编辑器保留你的改动未被覆盖".into());
            }
        } else if ctrl.algo_info().map(|i| i.is_default).unwrap_or(false) {
            // 默认算法设备侧无源 → 用内嵌默认源(★去注释★, 与"上传即保存去注释"语义一致)载入编辑器,
            // 而非展示带注释的原始模板。同时把去注释源回灌设备映射表一次, 使之后"读取信息"真正从设备取回。
            let default_src = AppController::strip_c_comments(ALGO_V31_TEMPLATE);
            if editor_untouched {
                ui.set_algo_c_source(default_src.clone().into());
                ui.set_algo_line_numbers(line_numbers_for(&default_src).into());
                ui.set_algo_c_bytes(AppController::algo_src_used(&default_src) as i32);
                editor_autoload_mark = default_src.clone();
            }
            if !default_src_synced {
                let _ = ctrl.send_algo_src(&default_src);
                default_src_synced = true;
                ui.set_algo_status("默认算法(v3.1 HDR): 已载入去注释 C 源并回灌设备(下次读取将直接取回设备保存源)".into());
            } else {
                ui.set_algo_status("默认算法(v3.1 HDR): 已载入去注释 C 源可修改".into());
            }
        } else {
            ui.set_algo_status(
                "设备未存该算法 C 源(可能为旧固件上传): 无法还原; 可点“加载模板”从默认源改起".into());
        }
        // ★串行接力★ C 源回读到此已收全(本分支由 algo_device_src_version 变化驱动), 通道空闲,
        // 此刻才发机器码回读 —— 与 C 源同拍发送会因设备 vendor 单响应槽丢掉一条。
        if explicit_readback {
            if let Err(error) = ctrl.request_algo_code() {
                ctrl.push_log_warn(format!("回读算法机器码失败: {}", error));
            }
        }
    }
    // 反汇编页回填(version 门控): 本地编译产物优先显示 objdump 反汇编;
    // 无本地编译(算法来自设备)时, 回读到的设备机器码以 hex dump 呈现真实 ASM 字节。
    if ctrl.algo_asm_version() != last_algo_asm_version {
        last_algo_asm_version = ctrl.algo_asm_version();
        ui.set_algo_asm(ctrl.algo_asm().into());
    }
    if ctrl.algo_device_code_version() != last_algo_code_version {
        last_algo_code_version = ctrl.algo_device_code_version();
        // 显式回读(读取信息 / 连接后首次同步)要的就是"设备里那份机器码", 必须覆盖;
        // 平时(本地刚编译出反汇编)则保留本地产物, 不被设备 hex dump 顶掉。
        let explicit_asm = ctrl.take_algo_explicit_readback_asm();
        let device_hex = ctrl.algo_device_code_hex().to_string();
        let device_code_len = ctrl.algo_device_code().len();
        if !device_hex.is_empty() && (explicit_asm || ctrl.algo_asm().is_empty()) {
            ui.set_algo_asm(
                format!(
                    "; 设备当前算法机器码(ASM, hex dump; 由设备回读)\n{}",
                    device_hex
                )
                .into(),
            );
        }
        // ★容量条也要跟着设备真值走★ algo_asm_bytes 原先只在**本地编译**后回填, 于是
        // "从设备回读机器码"这条路上它一直是 0, 界面显示 "ASM: 0 / 1024 字节" —— 明明已经
        // 读回了 540B 的机器码。回读到非空就按设备长度刷新占用。
        if device_code_len != 0 {
            ui.set_algo_asm_bytes(device_code_len as i32);
        }
    }
    // 追踪数据只用于单通道精调主图；算法页已改为静态元数据表。
    if ctrl.algo_trace_version() != last_algo_trace_version {
        last_algo_trace_version = ctrl.algo_trace_version();
        if !ctrl.plot_frozen() {
            algo_overlay_dirty = true;
        }
    }

    // 算法可调变量或运行值变化时重建元数据行；设置编辑控件保持原地更新。
    // ★运行值这一路必须限速★ 它现在随遥测帧到达(~170Hz), 每帧都 bump algo_trace_version。
    // 而这个块本身是重活: 两次解析 C 源(算 report/setting 声明) + 逐行 diff 推 VecModel。
    // 不限速就等于把它挂到 tick 全速(62Hz)上跑。用户交互驱动的那两路(cfg/metadata)不受限速,
    // 仍然立即生效 —— 只有"运行值又刷新了"这一路等下一个视觉节拍(~30Hz), 肉眼无差别。
    if ctrl.algo_cfg_version() != last_algo_cfg_version
        || ctrl.algo_cfg_ch_version() != last_algo_cfg_ch_version
        || channel_changed
        || (ctrl.algo_trace_version() != last_algo_metadata_trace_version && visual_refresh_tick)
        || ctrl.algo_metadata_version() != last_algo_metadata_version
    {
        last_algo_cfg_version = ctrl.algo_cfg_version();
        last_algo_cfg_ch_version = ctrl.algo_cfg_ch_version();
        last_algo_metadata_trace_version = ctrl.algo_trace_version();
        last_algo_metadata_version = ctrl.algo_metadata_version();
        // ★声明按作用域分流★ 一次解析, 分成两组: `ALGO_SETTING*` → cfg[8](一份值对 36 通道生效),
        // `ALGO_SETTING_CH*` → cfg_ch[8](每通道各一份)。两者的 idx 是**互相独立**的下标空间,
        // 所以不能只靠 idx 认变量, 也不能把两组塞进同一个模型(见 tick/mod.rs 两个模型的说明)。
        let decls = ctrl.algo_setting_decls();
        let sel_ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let rows: Vec<AlgoSettingRow> = decls
            .iter()
            .filter(|d| !d.per_channel)
            .map(|d| {
                let (alias, description) =
                    ctrl.algo_decl_text_named(1, d.idx, &d.name, &d.alias, &d.description);
                AlgoSettingRow {
                idx: d.idx as i32,
                name: d.name.clone().into(),
                default_val: d.default as i32,
                value: ctrl.algo_cfg(d.idx) as i32,
                value_type: d.value_type.clone().into(),
                range: d.range.clone().into(),
                description: description.into(),
                alias: alias.into(),
                // 共享项: 设备上 PSoC 的 g_algo_cfg[8] 与 RP 的 PsocAlgo::_cfg[8] 都不带通道下标,
                // ALGO_SET_CFG 也没有通道字段 ⇒ 一份值必然对全部 36 通道生效。
                shared_scope: true,
            }})
            .collect();
        // 逐通道行: 值取**当前精调通道**(与该页 curve_ch_enable_set 同一手法, 通道号不进行数据流,
        // 免得界面与 Rust 各持一个"当前通道"而漂移)。
        // 元数据 override 的 index 用 idx+8: override 的键是 (fingerprint, kind, index), kind=1
        // 只有一套下标空间, 而 cfg[0] 与 cfg_ch[0] 是两个不同变量 —— 不错开就会共用同一条别名/注释。
        let ch_rows: Vec<AlgoSettingRow> = decls
            .iter()
            .filter(|d| d.per_channel)
            .map(|d| {
                let (alias, description) = ctrl.algo_decl_text_named(
                    1,
                    d.idx + mai2control_ui::proto::algo::ALGO_CFG_CH_META_BASE,
                    &d.name,
                    &d.alias,
                    &d.description,
                );
                AlgoSettingRow {
                    idx: d.idx as i32,
                    name: d.name.clone().into(),
                    default_val: d.default as i32,
                    value: ctrl.algo_cfg_ch(sel_ch, d.idx) as i32,
                    value_type: d.value_type.clone().into(),
                    range: d.range.clone().into(),
                    description: description.into(),
                    alias: alias.into(),
                    shared_scope: false,
                }
            })
            .collect();
        // 批量抽屉的"全通道算法配置"下拉: 只列共享项别名, 顺序与 algo_setting_rows 严格一致 ——
        // 下拉的 current-index 就是拿去索引那个模型的(见 all_channels.slint), 两者错位就会编辑到别的变量。
        let shared_names: Vec<slint::SharedString> = rows
            .iter()
            .map(|row| {
                slint::SharedString::from(if row.alias.is_empty() {
                    row.name.to_string()
                } else {
                    row.alias.to_string()
                })
            })
            .collect();
        ui.set_batch_algo_shared_names(slint::ModelRc::new(slint::VecModel::from(shared_names)));
        while algo_setting_rows_model_timer.row_count() > rows.len() {
            algo_setting_rows_model_timer.remove(algo_setting_rows_model_timer.row_count() - 1);
        }
        for (row, setting) in rows.into_iter().enumerate() {
            if row < algo_setting_rows_model_timer.row_count() {
                if algo_setting_rows_model_timer.row_data(row).as_ref() != Some(&setting) {
                    algo_setting_rows_model_timer.set_row_data(row, setting);
                }
            } else {
                algo_setting_rows_model_timer.push(setting);
            }
        }
        while algo_setting_ch_rows_model_timer.row_count() > ch_rows.len() {
            algo_setting_ch_rows_model_timer
                .remove(algo_setting_ch_rows_model_timer.row_count() - 1);
        }
        for (row, setting) in ch_rows.into_iter().enumerate() {
            if row < algo_setting_ch_rows_model_timer.row_count() {
                if algo_setting_ch_rows_model_timer.row_data(row).as_ref() != Some(&setting) {
                    algo_setting_ch_rows_model_timer.set_row_data(row, setting);
                }
            } else {
                algo_setting_ch_rows_model_timer.push(setting);
            }
        }
        let mut metadata = Vec::new();
        for decl in ctrl.algo_report_decls() {
            let (alias, description) =
                ctrl.algo_decl_text_named(0, decl.idx, &decl.name, &decl.alias, &decl.description);
            metadata.push(AlgoMetadataRow {
                idx: decl.idx as i32,
                name: decl.name.into(),
                value_type: decl.value_type.into(),
                range: decl.range.into(),
                current_value: ctrl
                    .algo_trace_report_last(decl.idx)
                    .map(|value| format!("{}", value))
                    .unwrap_or_else(|| "暂无运行值".to_string())
                    .into(),
                description: description.into(),
                alias: alias.into(),
                kind: "上报变量".into(),
            });
        }
        // 可调变量的元数据卡片: 共享项与逐通道项都要能改别名/注释, 但 override 的 index 必须错开
        // (见上面 ch_rows 的说明)。★kind 字符串必须恒为"可调变量"★: algo.slint 是按
        // `row.kind == "可调变量" ? 1 : 0` 反推 kind 的, 换个字面量就会把 setting 的编辑写到
        // report 的 override 键上。作用域改由 name 后缀说明, 不动 kind。
        for decl in decls.iter() {
            let meta_index = if decl.per_channel {
                decl.idx + mai2control_ui::proto::algo::ALGO_CFG_CH_META_BASE
            } else {
                decl.idx
            };
            let (alias, description) = ctrl.algo_decl_text_named(
                1,
                meta_index,
                &decl.name,
                &decl.alias,
                &decl.description,
            );
            let current_value = if decl.per_channel {
                format!("CH{}: {}", sel_ch, ctrl.algo_cfg_ch(sel_ch, decl.idx))
            } else {
                ctrl.algo_cfg(decl.idx).to_string()
            };
            metadata.push(AlgoMetadataRow {
                idx: meta_index as i32,
                name: if decl.per_channel {
                    format!("{} (cfg_ch[{}], 逐通道)", decl.name, decl.idx)
                } else {
                    format!("{} (cfg[{}], 全通道共享)", decl.name, decl.idx)
                }
                .into(),
                value_type: decl.value_type.clone().into(),
                range: decl.range.clone().into(),
                current_value: current_value.into(),
                description: description.into(),
                alias: alias.into(),
                kind: "可调变量".into(),
            });
        }
        while algo_metadata_rows_model_timer.row_count() > metadata.len() {
            algo_metadata_rows_model_timer.remove(algo_metadata_rows_model_timer.row_count() - 1);
        }
        for (row, metadata_row) in metadata.into_iter().enumerate() {
            if row < algo_metadata_rows_model_timer.row_count() {
                if algo_metadata_rows_model_timer.row_data(row).as_ref() != Some(&metadata_row) {
                    algo_metadata_rows_model_timer.set_row_data(row, metadata_row);
                }
            } else {
                algo_metadata_rows_model_timer.push(metadata_row);
            }
        }
    }
}
