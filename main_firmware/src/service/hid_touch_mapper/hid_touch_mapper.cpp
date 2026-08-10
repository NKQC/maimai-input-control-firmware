#include "hid_touch_mapper.h"

#include "../config_manager/config_manager.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/hid/hid.h"

#include <cstdio>

HidTouchMapper* HidTouchMapper::_instance = nullptr;

HidTouchMapper::HidTouchMapper()
    : _hid(nullptr), _applied_mask(0), _hid_mode(false) {
    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        _point[ch].clear();
    }
}

HidTouchMapper* HidTouchMapper::getInstance() {
    if (_instance == nullptr) _instance = new HidTouchMapper();
    return _instance;
}

void HidTouchMapper::init(HID* hid) {
    _hid = hid;
    // 工作模式锁存: USB 描述符按 mode.work 二选一, 改它必须重启整机才生效(hal_usb.cpp),
    // 所以运行期这个判定不会翻转 —— 每轮重读 KV 只是白费一次字符串查表。
    _hid_mode = (ConfigManager::get_uint8("mode.work") ==
                 static_cast<uint8_t>(UsbWorkMode::WORK_HID));
    _applied_mask = 0;
    reload();
}

void HidTouchMapper::reload() {
    char key_buf[16];
    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        Point& p = _point[ch];
        snprintf(key_buf, sizeof(key_buf), "hid.en%02u", ch);
        p.enabled = ConfigManager::get_bool(key_buf);
        snprintf(key_buf, sizeof(key_buf), "hid.x%02u", ch);
        p.x = ConfigManager::get_uint16(key_buf);
        snprintf(key_buf, sizeof(key_buf), "hid.y%02u", ch);
        p.y = ConfigManager::get_uint16(key_buf);
        // KV 已带 0..32767 围栏; 这里再夹一次挡住存量配置文件里的越界值(描述符 logical max 之外的
        // 坐标会被主机按满量程截断, 表现为点位莫名跑到边缘, 极难自查)。
        if (p.x > TOUCH_LOGICAL_MAX) p.x = TOUCH_LOGICAL_MAX;
        if (p.y > TOUCH_LOGICAL_MAX) p.y = TOUCH_LOGICAL_MAX;
    }
}

uint8_t HidTouchMapper::enabled_count() const {
    uint8_t n = 0;
    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        if (_point[ch].enabled) n++;
    }
    return n;
}

inline void HidTouchMapper::_release_all() {
    if (_applied_mask == 0) return;
    _applied_mask = 0;
    if (_hid != nullptr) _hid->release_all_touch();
}

void HidTouchMapper::tick(uint64_t touch_mask, bool trusted) {
    // ★非 HID 模式一步不做★: WORK_SERIAL 下触摸屏点位映射整体不执行(bind.map 那套走 game_io),
    // 连释放都不必 —— 本服务在该模式下从未按下过任何触点。
    if (!_hid_mode || _hid == nullptr) return;

    // 掩码不可信(链路抖动保留窗外 / 扫描会话逐格改参数)⇒ 全释放。
    // 不是"保持上一轮": 那会在扫描期间把噪声掩码当成真手指按住不放。
    if (!trusted) {
        _release_all();
        return;
    }

    // 期望掩码 = 可信掩码 ∩ 已启用点位。未启用通道即便被触摸也不输出(默认全不启用 ⇒ 默认不输出);
    // 运行中把某通道改成未启用, 下一轮它自然落出期望集合并被当作抬起边沿释放。
    uint64_t want = 0;
    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        if (!_point[ch].enabled) continue;
        if (((touch_mask >> ch) & 1ULL) != 0) want |= (uint64_t{1} << ch);
    }

    const uint64_t changed = want ^ _applied_mask;
    if (changed == 0) return;   // 稳态零流量: 只发边沿, 不重复发按住

    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        if (((changed >> ch) & 1ULL) == 0) continue;
        const bool pressed = ((want >> ch) & 1ULL) != 0;
        HID_TouchPoint point;
        point.clear();
        // 触点 id = 通道号 + 1: 0 在 HID 触屏语义里是保留/无效触点 id, 不能用。
        point.id = static_cast<uint8_t>(ch + 1);
        point.press = pressed;
        if (pressed) {
            point.x = _point[ch].x;
            point.y = _point[ch].y;
        }
        _hid->send_touch_report(point);
    }
    _applied_mask = want;
}
