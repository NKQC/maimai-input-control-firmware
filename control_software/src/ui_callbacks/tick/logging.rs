{
    // 界面偏好对账落盘(值未变则不写盘)。放在 tick 里而不是给每个开关挂回调:
    // Slint 侧有些开关(自动滚动、页签、通道)是直接改 in-out 属性的, 本来就没有回调可挂。
    persist_ui_settings(
        &ui,
        &report_show_timer.borrow(),
        &mut ui_cfg_tick.borrow_mut(),
    );

    // 日志页：内存环形窗口按 500 行分页，页内 ScrollView 完整可滚动；切页可继续加载所有保留日志。
    {
        let hub = mai2control_ui::logging::hub();
        let ver = hub.version();
        let page_start = ui.get_log_page_start();
        if ver != last_log_ver || log_dirty.get() || page_start != last_log_page_start {
            last_log_ver = ver;
            last_log_page_start = page_start;
            log_dirty.set(false);
            let all_entries = hub.snapshot(ctrl.log_filter(), mai2control_ui::logging::RING_MAX);
            let total = all_entries.len();
            const LOG_PAGE_SIZE: usize = 500;
            let max_start = total.saturating_sub(LOG_PAGE_SIZE);
            let follow_latest = ui.get_log_auto_scroll() && !ui.get_log_has_newer();
            let start = if follow_latest {
                max_start
            } else {
                (page_start.max(0) as usize).min(max_start)
            };
            if start != page_start.max(0) as usize {
                last_log_page_start = start as i32;
                ui.set_log_page_start(start as i32);
            }
            let end = (start + LOG_PAGE_SIZE).min(total);
            let entries = &all_entries[start..end];
            let mut text = String::with_capacity(entries.len() * 96);
            let mut line_numbers = String::with_capacity(entries.len() * 8);
            for (offset, entry) in entries.iter().enumerate() {
                text.push_str(&entry.text.replace('\n', " | "));
                text.push('\n');
                line_numbers.push_str(&(start + offset + 1).to_string());
                line_numbers.push('\n');
            }
            ui.set_log_line_numbers(line_numbers.into());
            ui.set_log_text(text.into());
            ui.set_log_page_size(LOG_PAGE_SIZE as i32);
            ui.set_log_total_lines(total as i32);
            ui.set_log_has_older(start > 0);
            ui.set_log_has_newer(end < total);
            let dropped = hub.dropped();
            ui.set_log_stats(
                format!(
                    "当前页 {}–{} / {} 行 · 内存保留 {} 条{}",
                    if total == 0 { 0 } else { start + 1 },
                    end,
                    total,
                    mai2control_ui::logging::RING_MAX,
                    if dropped > 0 {
                        format!(" · 已滚出内存 {} 条(仍在日志文件中)", dropped)
                    } else {
                        String::new()
                    }
                )
                .into(),
            );
        }
    }
}
