# 任务进度：Rust 配置 Entry 编解码 + app_state 缓存逻辑

## 任务目标
在 Rust 上位机实现：
1. ✓ CfgValue 枚举 + ConfigEntry 结构体
2. ✓ encode_entry / decode_entry 与固件字节级对齐
3. ✓ decode_entries 批量解析
4. AppController 配置缓存与命令方法
5. poll 中的帧处理扩展
6. 测试验证

## 已完成步骤
- ✓ 读取现有代码：proto/mod.rs、app_state/mod.rs、io/mod.rs
- ✓ 确认权威线格式：protocol_design.md 修订 2（C1~C5）
- ✓ 读取固件参考实现：encode_entry/decode_entry 字节格式
- ✓ 创建 proto/config.rs 模块：
  - CfgValue 枚举（Bool/I8/U8/U16/U32/F32/Str）
  - ConfigValueType 类型码
  - ConfigEntry 结构体
  - encode_entry：Entry → Vec<u8>
  - decode_entry：&[u8] → (Entry, 消费字节数)
  - decode_entries：&[u8] → Vec<Entry>（count+entries格式）
- ✓ 测试全通过（8 tests）：
  - 类型 encode/decode 往返一致
  - 带 range 的 Entry 正确
  - String 支持
  - 字节级对齐与固件一致（test_byte_level_alignment_u16_with_range）

## 待办步骤
✓ 1. 扩展 AppController 添加缓存字段与方法（config_cache、cfg_all_accum）
✓ 2. 实现命令方法：request_config_all / set_config / save_config / reset_defaults
✓ 3. 扩展 poll 处理 CFG_* 响应帧（RESPONSE/STREAM）
✓ 4. 编写 poll 扩展测试
✓ 5. cargo build 与 cargo test 全通过

## 关键决策
- ✓ 复用现有 Frame / HostCmd 枚举
- ✓ CfgValue 为 pub enum，ConfigEntry 为 pub struct
- ✓ 编解码返回 Result<...> 处理错误
- ✓ AppController 用 BTreeMap<String, ConfigEntry> 缓存
- ✓ 流式 GET_ALL 帧用 STREAM(0x02) 标记继续，RESPONSE(0x01) 标记末帧

## 验证结果
- ✓ cargo build 成功（warnings 无关死代码）
- ✓ cargo test 45 tests 全通过，含：
  - proto/config 8 tests：encode/decode 各类型、range、byte-level alignment
  - app_state 12 tests：缓存、命令、poll 扩展（流式帧、单条、组）
  - 其他 25 tests：proto frame、comport、io

## 任务完成
所有内容已实现并验证通过。交付给 #6e-2（Slint UI 配置页）的接入点：
1. `app_state::AppController::request_config_all()` → 拉取全部配置
2. `app_state::AppController::set_config(entry)` → 设置单项
3. `app_state::AppController::config_entries()` → 获取缓存列表
4. `app_state::AppController::config_get(key)` → 查询单项
5. `app_state::AppController::save_config()` → 保存到设备
6. `app_state::AppController::reset_defaults()` → 恢复默认值
