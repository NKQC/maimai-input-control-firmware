# phase3-stage1b: 分区绑定真正生效 + BIND_* 交互 —— 完成

## 任务目标
实现 bind.mapNN 新语义(值=物理通道索引0..35,0xFFFFFFFF=未映射)在固件生效,
game_io 消费真实绑定表替代恒等映射,BIND_START/ABORT/GET_MAP/SET_MAP 交互式
绑定,及 Rust 上位机侧新增便捷方法。

## 状态: 全部完成 + 编译验证通过

## 涉及/新增文件
1. **新建** `main_firmware/src/service/binding_service/binding_service.h`
   `main_firmware/src/service/binding_service/binding_service.cpp`
   - `BindingService` 单例:`init()` 注册 4 个 host_cmd handler(BIND_START/
     ABORT/GET_MAP/SET_MAP)+ 首次 `reload_binding()`。
   - `reload_binding()`: 用 snprintf "bind.map%02u" 拼 key,读
     `ConfigManager::get_uint32`,>35 或 ==0xFFFFFFFF 归一化为 `_bind_ch[z]=0xFF`。
   - `map_to_areas(touch_mask)`: 34 位逻辑分区 ← 36 位物理通道 mask,按
     `_bind_ch[]` 查表。
   - `tick(touch_mask, link_ok)`: WAIT_TOUCH 态下用 `__builtin_ctzll` 取最低
     置位通道,写 `ConfigManager::set_uint32("bind.mapNN", ch)`,
     `reload_binding()`,发 BIND_EVENT(zone,ch,1)帧,回 IDLE。
   - handler 实现严格照抄 sensor_link.cpp 范式(ACK/NAK 用
     `HostCmdCodec::encode_ack/encode_nak`,响应帧用 `encode_frame`,
     `HostFrame::clear()`+手填 payload)。

2. **改** `main_firmware/src/service/game_io/game_io.cpp`
   - 删除匿名 namespace 里的 `ZONE_CHANNEL_MAP`/`map_channels_to_areas`
     (恒等占位映射,不再需要)。
   - `#include "../binding_service/binding_service.h"`。
   - `task()` 里改为 `Mai2Serial_TouchState touch(BindingService::getInstance()->map_to_areas(channel_mask));`
     latency 打点(_lt0/_lt1)逻辑原样保留。

3. **改** `main_firmware/src/main.cpp`
   - `#include "service/binding_service/binding_service.h"`。
   - `setup()`: `SensorLink::getInstance()->init();` 后加
     `BindingService::getInstance()->init();`。
   - `loop()`: `SensorLink::getInstance()->tick();` 后加
     `BindingService::getInstance()->tick(psoc->link_ok()?psoc->touch_mask():0, psoc->link_ok());`
     (SWD_RELEASE_TO_EXTERNAL 早返回分支未加,按要求)。

4. **改** `control_software/src/app_state/mod.rs`
   - `bind_start()` 签名改为 `pub fn bind_start(&mut self, zone: u8) -> anyhow::Result<()>`,
     zone>=34 直接 Err(不发帧),否则发 `BIND_START` payload=[zone]。
   - 新增 `pub fn set_binding_channel(&mut self, zone: usize, channel: u8)`:
     channel>=36 写 0xFFFFFFFF,否则写 channel,内部复用已有 `set_binding`。
   - 新增 `pub fn binding_channel_of(&self, zone: usize) -> u8`:读
     `get_binding(zone)`,0xFFFFFFFF 或 >35 返回 0xFF,否则低字节。
   - 未改动 `make_binding`/`binding_channel_mask`/`binding_device_mask`
     (按要求留给 Stage2)。

5. **改** `control_software/src/main.rs`
   - `on_bind_start` 回调改为读 `ui.get_selected_zone()`,越界(<0 或 >=34)
     直接 return,否则 `ctrl.bind_start(selected_zone as u8)`。

## 未改动(确认不需要)
- `main_firmware/platformio.ini`: `board_build.src_filter = +<src/**>` 通配,
  新增 binding_service 目录自动纳入编译,无需登记。
- app.slint / Stage2 UI 相关不动。

## 编译/测试验证结果
- `dev.ps1 build-rp` → `[SUCCESS] Took 7.82 seconds`,Flash 15.1%/RAM 13.4%,
  无警告/错误。
- `dev.ps1 build-ui` → `Finished` dev profile,无 error(仅 PowerShell
  Select-Object 对 "Compiling" 行的无害 NativeCommandError 噪音,非编译错误)。
- `cargo test --lib`(control_software) → `70 passed; 0 failed`,含
  绑区相关既有测试(`test_zone_key_mapping`/`test_binding_map_*`/
  `test_set_binding_out_of_range_errors`/`test_handle_bind_event_sets_progress`
  等)全部通过,未破坏任何既有测试。

## 关键决策
- BindingService 与 SensorLink 同构:单例+静态 handler+getInstance() 访问
  实例状态,复用现有 host_cmd 注册/响应范式,不引入新抽象。
- `_bind_ch[]` 用 uint8_t(0xFF=未映射)而非直接存 u32,节省 RAM 且与
  `map_to_areas` 的位运算路径一致。
- Rust `bind_start` 改签名属于范围内明确要求的"改现有方法+同步调用点",
  已按要求修正 main.rs 唯一调用点,无遗留编译错误。

## 无异常
未触发"连续失败超3次"或"架构冲突"情形,任务顺利完成,无需向上级报告异常。
