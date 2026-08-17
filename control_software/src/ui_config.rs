//! 纯 UI 设置的本地持久化(启动目录 config.cfg)。
//!
//! 为什么单独一个文件: `toolbox.cfg` 存的是"会改动系统状态"的东西(游戏串口固定 COM 号、
//! 扫码器设备路径), 与"纯界面偏好"(日志等级、自动滚动、当前页签)混在一起容易互相牵连;
//! 而且本模块是通用 key=value 存储, 加一项设置不需要动函数签名。
//!
//! 格式: `key=value` 纯文本, 一行一项, `#` 开头为注释。选它的理由: 人能直接看懂改动、
//! 坏了也能手删一行, 不必为界面偏好引入 serde 依赖。

use std::collections::BTreeMap;
use std::path::PathBuf;

pub struct UiConfig {
    _map: BTreeMap<String, String>,
    _path: PathBuf,
}

impl UiConfig {
    /// 从启动目录读取(不存在则返回空配置, 首次 set 时会创建)。
    pub fn load() -> Self {
        let path = Self::_path();
        let mut map = BTreeMap::new();
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                for line in text.lines() {
                    let line = line.trim();
                    if line.is_empty() || line.starts_with('#') {
                        continue;
                    }
                    // 只按第一个 '=' 切分: 值里可能含 '='(如设备路径)。
                    if let Some((k, v)) = line.split_once('=') {
                        map.insert(k.trim().to_string(), v.trim().to_string());
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => log::warn!("读取界面配置失败({}): {}", path.display(), e),
        }
        Self {
            _map: map,
            _path: path,
        }
    }

    pub fn path_text(&self) -> String {
        self._path.display().to_string()
    }

    pub fn get_i32(&self, key: &str, default: i32) -> i32 {
        self._map
            .get(key)
            .and_then(|v| v.parse::<i32>().ok())
            .unwrap_or(default)
    }

    pub fn get_bool(&self, key: &str, default: bool) -> bool {
        match self._map.get(key).map(|s| s.as_str()) {
            Some("1") | Some("true") => true,
            Some("0") | Some("false") => false,
            _ => default,
        }
    }

    pub fn set_i32(&mut self, key: &str, value: i32) {
        self._set(key, value.to_string());
    }

    pub fn set_bool(&mut self, key: &str, value: bool) {
        self._set(key, if value { "1".into() } else { "0".into() });
    }

    /// 值未变则不落盘: UI 上每个 tick 都可能回写同样的值, 避免无意义的磁盘写。
    fn _set(&mut self, key: &str, value: String) {
        if self._map.get(key) == Some(&value) {
            return;
        }
        self._map.insert(key.to_string(), value);
        self._save();
    }

    fn _save(&self) {
        let mut text = String::from("# mai2control-ui 界面设置(自动生成, 可手改)\n");
        for (k, v) in &self._map {
            text.push_str(k);
            text.push('=');
            text.push_str(v);
            text.push('\n');
        }
        if let Err(e) = std::fs::write(&self._path, text) {
            log::warn!("保存界面配置失败({}): {}", self._path.display(), e);
        }
    }

    fn _path() -> PathBuf {
        let base = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|d| d.to_path_buf()))
            .or_else(|| std::env::current_dir().ok())
            .unwrap_or_else(|| PathBuf::from("."));
        base.join("config.cfg")
    }
}

/// 配置键名集中在此, 避免散落的字符串字面量拼错。
pub mod keys {
    pub const LOG_FILTER: &str = "log_filter";
    pub const LOG_AUTO_SCROLL: &str = "log_auto_scroll";
    pub const VCAM_ENABLED: &str = "vcam_enabled";
    pub const VCAM_SUBMIT_SECS: &str = "vcam_submit_secs";
    pub const VCAM_DISPLAY_SECS: &str = "vcam_display_secs";
    pub const VCAM_MIRROR_X: &str = "vcam_mirror_x";
    pub const VCAM_FRAME_W: &str = "vcam_frame_w";
    pub const VCAM_FRAME_H: &str = "vcam_frame_h";
    pub const VCAM_QR_FILL_PCT: &str = "vcam_qr_fill_pct";
    pub const DIAG_EXPANDED: &str = "diag_expanded";
    pub const CURVE_PARAMS_EXPANDED: &str = "curve_params_expanded";
    pub const CURVE_ALGO_CFG_EXPANDED: &str = "curve_algo_cfg_expanded";
    pub const CURVE_SERIES_EXPANDED: &str = "curve_series_expanded";
    pub const CURVE_SPECTRUM_EXPANDED: &str = "curve_spectrum_expanded";
    pub const CHANNEL_SHOW_DISABLED: &str = "channel_show_disabled";
    // —— 全通道页批量抽屉里两个算法分区的展开态 + "全通道算法配置"下拉的选中项 ——
    // 与其它折叠区同性质的纯界面偏好: 抽屉是 `if` 条件实例化的, 展开态本就存在 AppWindow 上;
    // 不持久化的话每次开程序都要把这两区重新点开、把下拉重新选回要调的那一项。
    pub const BATCH_ALGO_CH_EXPANDED: &str = "batch_algo_ch_expanded";
    pub const BATCH_ALGO_CFG_EXPANDED: &str = "batch_algo_cfg_expanded";
    pub const BATCH_ALGO_SHARED_SEL: &str = "batch_algo_shared_sel";
    pub const PHYS_LIVE_EXPANDED: &str = "phys_live_expanded";
    pub const PHYS_KEYS_EXPANDED: &str = "phys_keys_expanded";
    pub const PHYS_LA_EXPANDED: &str = "phys_la_expanded";
    pub const MAI2_PANEL_EXPANDED: &str = "mai2_panel_expanded";
    pub const LIGHT_PANEL_EXPANDED: &str = "light_panel_expanded";
    pub const LATENCY_MEASURE: &str = "latency_measure";
    pub const CURRENT_VIEW: &str = "current_view";
    pub const SETTINGS_TAB: &str = "settings_tab";
    pub const SEL_CHANNEL: &str = "sel_channel";
    /// 二值算法上报线的归一化幅度 N(千分之一整数存储)。实际键 = 前缀 + idx(0..3)。
    /// 纯显示偏好: 只影响本机折线图高度, 不下发设备。
    pub const ALGO_BIN_AMP_PREFIX: &str = "algo_bin_amp";
    /// 触发判定(out_active)线的归一化幅度 N(同上, 千分之一整数存储)。
    pub const ALGO_BIN_AMP_ACTIVE: &str = "algo_bin_amp_active";
    // —— 曲线页"画哪些系列"的记忆 ——
    // 纯显示偏好(与归一化幅度同一性质): 勾哪几条线是看图前第一件要做的事, 重启即丢等于每次开
    // 程序都要重新点一遍。默认值与 .slint 里的属性默认值保持一致, 免得首次启动被"记忆"改掉观感。
    pub const CURVE_SHOW_RAW: &str = "curve_show_raw";
    pub const CURVE_SHOW_BSLN: &str = "curve_show_bsln";
    pub const CURVE_SHOW_DIFF: &str = "curve_show_diff";
    /// 触发判定(out_active)线的勾选。
    pub const CURVE_SHOW_ACTIVE: &str = "curve_show_active";
    /// 算法上报叠加总开关(关着时 4 条上报线勾了也不画)。
    pub const CURVE_ALGO_OVERLAY: &str = "curve_algo_overlay";
    /// 4 条算法上报线各自的勾选。实际键 = 前缀 + idx(0..3)。
    pub const CURVE_REPORT_SHOW_PREFIX: &str = "curve_report_show";
}
