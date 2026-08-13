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
            let cap = AppController::algo_slot_capacity();
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
    if algo_version != last_algo_version || !algo_info_filled || algo_page_visible && !last_algo_page_visible {
        last_algo_version = algo_version;
        let txt = match ctrl.algo_info() {
            Some(i) => format!(
                "当前算法: {} | PSoC valid={} | len={}B | crc16=0x{:04X}",
                if i.is_default { "默认(v3.1 HDR)" } else { "自定义" },
                i.psoc_valid, i.len, i.crc16
            ),
            None if ctrl.state() == ConnState::Disconnected => "算法信息未读取（未连接）".to_string(),
            None => "算法信息未读取".to_string(),
        };
        ui.set_algo_info_text(txt.into());
        algo_info_filled = true;
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
        || (ctrl.algo_trace_version() != last_algo_metadata_trace_version && visual_refresh_tick)
        || ctrl.algo_metadata_version() != last_algo_metadata_version
    {
        last_algo_cfg_version = ctrl.algo_cfg_version();
        last_algo_metadata_trace_version = ctrl.algo_trace_version();
        last_algo_metadata_version = ctrl.algo_metadata_version();
        let rows: Vec<AlgoSettingRow> = ctrl
            .algo_setting_decls()
            .into_iter()
            .map(|d| {
                let (alias, description) =
                    ctrl.algo_decl_text_named(1, d.idx, &d.name, &d.alias, &d.description);
                AlgoSettingRow {
                idx: d.idx as i32,
                name: d.name.into(),
                default_val: d.default as i32,
                value: ctrl.algo_cfg(d.idx) as i32,
                value_type: d.value_type.into(),
                range: d.range.into(),
                description: description.into(),
                alias: alias.into(),
            }})
            .collect();
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
        for decl in ctrl.algo_setting_decls() {
            let (alias, description) =
                ctrl.algo_decl_text_named(1, decl.idx, &decl.name, &decl.alias, &decl.description);
            metadata.push(AlgoMetadataRow {
                idx: decl.idx as i32,
                name: decl.name.into(),
                value_type: decl.value_type.into(),
                range: decl.range.into(),
                current_value: ctrl.algo_cfg(decl.idx).to_string().into(),
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
