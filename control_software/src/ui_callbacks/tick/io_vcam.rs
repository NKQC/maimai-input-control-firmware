{
    // 上一拍里"控制器正被借出"时排队的配置写入: 现在借到了, 原样补齐。
    // 队列非空说明确实发生过重入(否则回调当场就写完了), 如实记一条以便追踪。
    let queued: Vec<PendingCfgWrite> = pending_cfg_tick.borrow_mut().drain(..).collect();
    if !queued.is_empty() {
        log::debug!("配置写入重入排队 {} 条, 本拍补齐", queued.len());
        for write in queued {
            write.apply(&mut ctrl);
        }
    }

    // 虚拟摄像头: 推进时序状态机(显示到期转黑)。帧变化时把 RGB24 转 NV12 提交到共享
    // 队列(序号在像素写完之后才提交, 由 FramePublisher 保证), 再刷新 UI 预览。
    // 每 tick 都刷心跳: 过滤器靠 tick_ms 区分"画面没变"与"生产者已消失(进程被杀)"。
    vcam_timer.tick();
    if ui.get_vcam_enabled() {
        let status = vcam::keyboard::runtime_status();
        if status != last_vcam_runtime_status {
            ui.set_vcam_runtime_status(status.clone().into());
            last_vcam_runtime_status = status;
        }
    }
    if let Some(publisher) = publisher_timer.borrow_mut().as_mut() {
        publisher.heartbeat();
    }
    let vframe_ver = vcam_timer.frame_version();
    if vframe_ver != last_vcam_frame_version {
        last_vcam_frame_version = vframe_ver;
        let rgb = vcam_timer.frame_copy();
        if let Some(publisher) = publisher_timer.borrow_mut().as_mut() {
            if let Err(error) = publisher.publish(&rgb) {
                log::error!("虚拟摄像头: 帧发布失败: {}", error);
            }
        }
        let mut buf = slint::SharedPixelBuffer::<slint::Rgb8Pixel>::new(FRAME_W as u32, FRAME_H as u32);
        let dst = buf.make_mut_bytes();
        if dst.len() == rgb.len() {
            dst.copy_from_slice(&rgb);
            ui.set_vcam_preview(slint::Image::from_rgb8(buf));
        }
        ui.set_vcam_last_data(vcam_timer.last_data().into());
    }

    // CSD 调试诊断: 推进"改全局后回读设备状态"的排程与窗口(日志写入见 app_state)。
    ctrl.csd_diag_tick();

    // 恢复默认设备端回填就绪: 全量重读(配置/当前通道+CH0 参数/全局/Cp) 刷新显示。
    if ctrl.take_post_reset_refetch() {
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        // Cp 不在此刷新: 电容获取一律由用户手动"测量电容"触发(自动测量会与重初始化抢链路)。
        ctrl.push_log("恢复默认完成: 重读设备配置 / 全 36 通道参数 / 全局 刷新显示");
        let _ = ctrl.request_config_all();
        let _ = ctrl.request_params(ch);
        let _ = ctrl.request_params(0);
        // ★必须覆盖全 36 通道★: 只重读"当前通道 + CH0"会让其余 34 通道停在恢复默认前的
        // 过期值(或空), 界面表现为"只留了一个通道的数据"。队列每 tick 发一条, 不加快轮询。
        ctrl.schedule_param_refetch_all();
        let _ = ctrl.global_get_all();
        // 重取 DEVICE_INFO: 其报告尾部 csd_flags 携带"恢复默认是否获得可信基线", 驱动异常采样警示行。
        let _ = ctrl.resend_hello();
        // 时钟树的分频范围需要全 36 通道 snsClk(恢复默认后已全变): 一条批量取回, 非 36 条单发。
        let _ = ctrl.request_param_all_channels(PARAM_SNS_CLK_DIV);
    }
    let prof_poll_start = Instant::now();
    ctrl.poll_ui();
    prof_poll_us += prof_poll_start.elapsed().as_micros() as u64;
    // 侦听绑定: 捕获下一次触摸的物理通道并写入草稿(仅在侦听态时有动作)。
    let _ = ctrl.listen_tick();
}
