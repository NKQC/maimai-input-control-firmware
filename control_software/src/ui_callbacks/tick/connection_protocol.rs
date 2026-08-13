{
    // 连接成功边沿主动拉取当前通道参数与 Cp，并开启全通道遥测；
    // 断线边沿清空 CpPollState，避免重连后展示旧设备/旧会话 Cp。
    let connected = ctrl.state() == ConnState::Connected;
    // ★CSD 模式回填: 两个 UI 属性同一个来源★ mode_known 与 scan_mode 全部由
    // csd_mode_effective()(草稿 → 设备真值 → None) 派生, 不再各走一条路。
    // None(尚未取得任何真值) ⇒ 显式 mode_known=false 且 scan_mode 归 0:
    // 旧实现在此用 `if let Some` 直接不赋值, 使 scan_mode 锁存上一次的模式 —— 掉线重连后
    // 页面仍按旧模式渲染, 与 mode_known=false 组合出"下拉说半自动、时钟树说 AUTO"的自相矛盾。
    let csd_mode = ctrl.csd_mode_effective();
    ui.set_mode_known(csd_mode.is_some());
    ui.set_scan_mode(csd_mode.map_or(0, |mode| (mode != 0) as i32));
    if connected && !was_connected {
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        // Cp 一律不自动获取: 连接边沿只清状态并显示"未测量", 等用户点"测量电容"。
        let mut cp_state = cp_poll_timer.borrow_mut();
        cp_state._stop("未测量".to_string());
        ui.set_cp_text(cp_state.status.clone().into());
        ctrl.push_log("已连接 → 已启动最小握手回读");
        telemetry_first_frame_seen = false;
        // ★连接后自动把设备上的算法同步到 UI★
        // 连接探针只回读 ALGO_GET_INFO/ROM, 不含 C 源与机器码, 于是 algo_device_src 一直为空 ⇒
        // schema 源回落到编辑器里的初始示例算法(没有任何 META 声明) ⇒ report/setting 声明为空 ⇒
        // 追踪轮询集合为空、从不发 ALGO_GET_TRACE ⇒ 算法页"当前值"永远是"暂无运行值",
        // 单通道精调页则显示"当前算法未声明 ALGO_REPORT"。同步这一步把三者一并补齐。
        algo_sync_pending = true;
        crash_report_pending = true;
        // 连接阶段只允许三项基础探针独占 vendor 通道；全通道参数预取必须等屏障收敛，
        // 否则它会在探针完成后立即与遥测/页面轮询叠加，造成固件 RX FIFO 溢出和后续探针饥饿。
        ctrl.schedule_conn_probes(ch);
        // 页面状态(键盘/灯板/mai2)在对应页面进入时懒加载，避免连接阶段串行探针占满链路。
    } else if !connected && was_connected {
        let mut cp_state = cp_poll_timer.borrow_mut();
        cp_state._stop("未连接".to_string());
        ui.set_cp_text(cp_state.status.clone().into());
    }
    // 仅在真正进入已连接的“触控全局调整”页(索引 3)时拉取一次 CH0，避免 16ms tick 洪泛。
    // 逐通道遥测的推流范围 = **整个设置页**, 不再细分到具体 Tab。
    // ★为什么不按 Tab 细分★: 先前收成"只有触控通道调整(2)与单通道精调(4)"两页, 结果
    //  - 分区绑定页的实时触摸态(绿色分区)没数据可用;
    //  - 在设置页内来回切 Tab 会反复 TELEM_START/切档, 单通道精调的曲线与读数出现空档、更新不及时。
    // 通道数据是整个设置页的共同底座, 粒度就该是"在不在设置页"。离开设置页(主页/工具箱/日志/关于)
    // 才收回轻档(仅采样率+延迟), 主页延迟卡照常有数据。
    let conn_probes_pending = ctrl.conn_probes_pending();
    // 基础探针收敛后补齐所有通道的启用真值；下降沿配合已知性门控，每个连接会话最多请求一次。
    let enable_state_refresh_queued = connected
        && last_conn_probes_pending
        && !conn_probes_pending
        && !ctrl.ch_enabled_states_known();
    if enable_state_refresh_queued {
        let _ = ctrl.request_param_all_channels(PARAM_ENABLED);
    }
    // 探针收敛后同步设备算法: C 源 + 机器码。标记为"显式回读"是有意的 —— 此刻编辑器里还是
    // 程序内置的初始示例算法, 用设备真值覆盖它才是用户期望的"连接后看到设备上跑的算法";
    // 而 editor_untouched 判据会把这份初始示例当成"用户的改动"从而拒绝覆盖。
    // ★只发 C 源, 不在同一拍连发机器码★ 设备侧 vendor 响应是单槽的, 同一拍连发两条只读命令
    // 会丢掉一条的响应; 而 C 源是多片顺序回读, 丢一片就整轮作废。机器码改由 C 源回读完成后
    // 顺带发起(见 tick/algorithm.rs 的显式回读分支), 天然串行。
    if algo_sync_pending && connected && !conn_probes_pending {
        algo_sync_pending = false;
        ctrl.mark_algo_explicit_readback();
        if let Err(error) = ctrl.request_algo_src() {
            ctrl.push_log_warn(format!("连接后回读算法 C 源失败: {}", error));
        }
        ctrl.push_log("连接探针完成 → 正在同步设备算法 C 源");
    }
    // 上次崩溃遗言: 探针收敛后读一次 → 落日志 → 立刻清除设备置位。
    // ★不在连接边沿发★ 设备侧 vendor 响应是单槽的, 边沿那一拍已被三项基础探针占满。
    // ★与算法同步错开一拍★ `!algo_sync_pending` 保证不与 ALGO_GET_SRC 抢同一拍。
    if crash_report_pending && connected && !conn_probes_pending && !algo_sync_pending {
        crash_report_pending = false;
        let read_ok = match ctrl.last_crash_report() {
            Ok(report) if report.abnormal() => {
                log::warn!(
                    "设备上次启动异常: 运行中崩溃={} hardfault={} 出错核={} 阶段={} 进段时已耗时={}ms 上次峰值轮耗时={}ms 复位原因=0x{:02X}({})",
                    report.was_watchdog,
                    report.was_fault,
                    report.fault_core_text(),
                    report.stage_name(),
                    report.stage_at_ms,
                    report.peak_ms,
                    report.reset_reason,
                    report.reset_reason_text()
                );
                ctrl.push_log_warn(format!(
                    "设备上次启动异常: 阶段={} 进段时已耗时={}ms 峰值轮={}ms hardfault={}({})",
                    report.stage_name(),
                    report.stage_at_ms,
                    report.peak_ms,
                    report.was_fault,
                    report.fault_core_text()
                ));
                true
            }
            Ok(_) => {
                log::info!("设备上次为正常启动(无看门狗/hardfault 遗言)");
                true
            }
            Err(error) => {
                log::warn!("读取设备上次崩溃数据失败: {}", error);
                ctrl.push_log_warn(format!("读取设备上次崩溃数据失败: {}", error));
                false
            }
        };
        // ★只在确实读到之后才清★ 读失败就清等于把还没看过的遗言抹掉; 保留不清, 下次连接还能再读一次。
        // 清除失败必须告警: 不清则同一次崩溃会在之后每次连接重报一遍, 真正的新崩溃被淹没。
        if read_ok {
            match ctrl.clear_last_crash() {
                Ok(()) => log::info!("已清除设备侧上次崩溃数据置位(EP0 0x54)"),
                Err(error) => {
                    log::warn!(
                        "清除设备侧上次崩溃数据失败: {}；下次连接会重复读到同一份遗言",
                        error
                    );
                    ctrl.push_log_warn(format!("清除设备侧上次崩溃数据失败: {}", error));
                }
            }
        }
    }
    let want_channel_stream = connected && ui.get_current_view() == 1;
    ctrl.telem_set_scope(want_channel_stream);
    if !telemetry_first_frame_seen && ctrl.telem_frame_count() != 0 {
        telemetry_first_frame_seen = true;
    }
    // 自愈只处理已建立并曾实际收帧的会话；冷启动首帧由固件 provisioning 完成后自然到达。
    if telemetry_first_frame_seen {
        ctrl.telem_heal_tick();
    }

    let global_tune_visible = connected && ui.get_current_view() == 1 && ui.get_settings_tab() == SETTINGS_PAGE_GLOBAL_TUNE;
    if !conn_probes_pending && global_tune_visible && !last_global_tune_visible && was_connected {
        let _ = ctrl.request_params(0);
        // 时钟树的 snsClk 范围需要全 36 通道的值: 走 PARAM_GET_ALL 的全通道单参数变体, 一帧取回。
        let _ = ctrl.request_param_all_channels(PARAM_SNS_CLK_DIV);
    }
    last_global_tune_visible = global_tune_visible;

    // ★全通道网格进页必须批量回读通道启用开关(0x0C)★
    // `ch_enabled()` 在 0x0C 尚未回读时按"已启用"处理(见其注释: 刚连上就灰一片会让人以为设备坏了),
    // 而连接探针只为**当前选中通道**发 PARAM_GET ⇒ 其余 35 个通道的启用真值一直是空。后果有两处,
    // 都被用户实测到: 禁用的通道不灰显(渲染侧其实早已接好 opacity/文案, 只是 entry.enabled 是假的),
    // 以及取消"显示不启用的通道"时只隐藏了恰好被点过的那一个 —— 点一下某通道会触发它的 PARAM_GET,
    // 于是"点进去之后才隐藏"。
    // 用 PARAM_GET_ALL 的全通道单参数变体一帧取回 36 个值(与上面 snsClk 同一条既有链路), 只在进页
    // 边沿发一次: 网格本来就靠 param_version 门控重建, 回读到齐后会自然刷新。
    let all_channels_visible =
        connected && ui.get_current_view() == 1 && ui.get_settings_tab() == SETTINGS_PAGE_ALL_CHANNELS;
    if !conn_probes_pending
        && all_channels_visible
        && !last_all_channels_visible
        && was_connected
        && !enable_state_refresh_queued
        && !ctrl.ch_enabled_states_known()
    {
        let _ = ctrl.request_param_all_channels(PARAM_ENABLED);
    }
    last_all_channels_visible = all_channels_visible;

    // Protocol page state is polled at 5Hz; manual changes request one delayed confirmation.
    if let Some(verify_at) = mai2_verify_timer.get() {
        if Instant::now() >= verify_at {
            let _ = ctrl.mai2_request_state();
            mai2_verify_timer.set(None);
        }
    }
    let hid_mode = ctrl.work_mode() == Some(1);
    ui.set_hid_mode(hid_mode);
    let protocol_visible = connected && !hid_mode && ui.get_current_view() == 1 && ui.get_settings_tab() == SETTINGS_PAGE_PROTOCOL;
    if protocol_visible && !conn_probes_pending {
        if !last_protocol_visible || _every_ms(reconnect_tick, 200) {
            let _ = ctrl.mai2_request_state();
        }
        if !last_protocol_visible
            || (ui.get_light_panel_expanded() && _every_ms(reconnect_tick, 200))
        {
            let _ = ctrl.led_request_state();
        }
    }
    last_protocol_visible = protocol_visible;

    if !conn_probes_pending && connected && _every_ms(reconnect_tick, 1_000) {
        let _ = ctrl.ping();
    }
    // 物理键盘实时态 ~3Hz 轮询,降低 vendor IN 负载(键状态非高频需求)。
    if !conn_probes_pending && connected && _every_ms(reconnect_tick, 320) {
        let _ = ctrl.kbd_request_state();
    }
    // 物理键盘页驻留期: 实时三态提到 ~15Hz(要看得出防抖/长按的差别),
    // 边沿记录只在"逻辑分析仪"抽屉展开时才拉(~31Hz, 窗口=1)。
    // ★不放到每 tick★: 全设备共用一对 bulk 端点, 遥测已占 30Hz; 边沿是设备侧缓冲的(192 条),
    // 32ms 拉一次不会丢数据, 却能把这条新增流量压到与既有轮询同量级。
    let phys_kbd_visible =
        connected && ui.get_current_view() == 1 && ui.get_settings_tab() == SETTINGS_PAGE_PHYS_KBD;
    if phys_kbd_visible && !conn_probes_pending {
        if !last_phys_kbd_visible {
            let _ = ctrl.kbd_request_keycfg();
        }
        if _every_ms(reconnect_tick, 64) {
            let _ = ctrl.kbd_request_state();
        }
        if ui.get_phys_la_expanded() && _every_ms(reconnect_tick, 32) {
            let _ = ctrl.kbd_request_edges();
        }
    }
    last_phys_kbd_visible = phys_kbd_visible;

    // ★已删除周期性 Cp 轮询★: 原"每 5 tick 轮询一个通道"与用户是否测量无关, 实测把 vendor 端点
    // 打满并与校准/自适应抢链路(NAK 每秒 8~12 条 → endpoint stall → 掉线)。
    // Cp 现在只由手动"测量电容"触发一轮抓取(见下方 CpSweep 推进块)。

    // ★算法运行值不再轮询★ report[]/out_active 现在随 FOCUS_DATA 的 FIELD_ALGO 块同帧到达
    // (见 AppController::_ingest_focus_algo)。原先这里按已声明的 report idx 轮转发 ALGO_GET_TRACE,
    // 但那是阻塞读类命令, 独占流推送期间抢不到设备的单响应槽, 请求成片超时。
    if connected {
        let poll_ctx = mai2control_ui::app_state::PollContext {
            connected,
            current_view: ui.get_current_view(),
            settings_tab: ui.get_settings_tab(),
            hid_mode,
            sel_channel: ui.get_sel_channel().clamp(0, 35) as u8,
            light_panel_expanded: ui.get_light_panel_expanded(),
            phys_la_expanded: ui.get_phys_la_expanded(),
        };
        ctrl.poll_scheduled(&poll_ctx, reconnect_tick, UI_TICK_MS);
    }
    last_conn_probes_pending = conn_probes_pending;
    was_connected = connected;
}
