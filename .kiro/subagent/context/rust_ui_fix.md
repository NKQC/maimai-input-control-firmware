# rust_ui_fix

## 任务目标
修复 Rust + Slint 上位机的配置类型码、按缓存原始类型回填数值/枚举配置，以及测试中缺失的 `DeviceInfo` 默认字段。

## 已完成步骤
1. 已确认协议类型码：Bool=0、I8=1、U8=2、U16=3、U32=4、F32=5、Str=6。
2. `src/main.rs`：已将 U8/U16/U32/I8 的行 type_code 修正为 2/3/4/1；数值与枚举回调分别转调 `set_config_number` 和 `set_config_enum`。
3. `src/app_state/mod.rs`：已新增按 `config_get` 缓存值类型重建 `CfgValue` 的 `set_config_number`，以及复用它的 `set_config_enum`；字符串项保留原字符串，找不到 key 回退 U32。
4. 测试用的全部三个已确认 `DeviceInfo` 字面量均已补齐 `psoc_generation: 0`、两个 `false` 标记和 `diagnostics: None`。
5. 已复读修改内容，确认仅涉及指定三处修复范围。
6. 已在 `control_software` 目录运行 `cargo build --bins`：成功，`Finished dev profile`，exit code 0。
7. 已运行 `cargo test --no-run`：成功，`Finished test profile` 并生成 3 个测试可执行文件，exit code 0。

## 待办步骤
无；任务完成。

## 关键决策
`set_config_enum` 将 index 转为 f64 后复用 `set_config_number`，因而与数值编辑采用同一份原始类型匹配逻辑；该行为满足枚举类型的 U8/U16/U32 需求，同时安全覆盖缓存可能存在的其他类型。

## 最终结果
三项限定修复均已完成，且两项要求的 Cargo 编译验证均通过。
