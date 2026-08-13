{
    // ★剖面汇报★: 16ms 的 tick 预算里究竟花在哪, 只有实测能回答。每 ~2s 一条。
    let spent = prof_tick_start.elapsed().as_micros() as u64;
    prof_tick_us += spent;
    prof_tick_max_us = prof_tick_max_us.max(spent);
    prof_ticks += 1;
    if prof_ticks >= 120 {
        log::debug!(
            "[PROF] tick n={} avg={}us max={}us | gap avg={}us max={}us | poll avg={}us | chart hits={} avg={}us max={}us | cards hits={} avg={}us | telem={}Hz pts={}",
            prof_ticks,
            prof_tick_us / prof_ticks as u64,
            prof_tick_max_us,
            if prof_gaps > 0 {
                prof_gap_us / prof_gaps as u64
            } else {
                0
            },
            prof_gap_max_us,
            prof_poll_us / prof_ticks as u64,
            prof_chart_hits,
            if prof_chart_hits > 0 { prof_chart_us / prof_chart_hits as u64 } else { 0 },
            prof_chart_max_us,
            prof_cards_hits,
            if prof_cards_hits > 0 { prof_cards_us / prof_cards_hits as u64 } else { 0 },
            ctrl.telem_samples_per_sec(),
            ui.get_curve_point_count(),
        );
        prof_ticks = 0;
        prof_tick_us = 0;
        prof_tick_max_us = 0;
        prof_poll_us = 0;
        prof_chart_us = 0;
        prof_chart_hits = 0;
        prof_chart_max_us = 0;
        prof_cards_us = 0;
        prof_cards_hits = 0;
        prof_gap_us = 0;
        prof_gap_max_us = 0;
        prof_gaps = 0;
    }
}
