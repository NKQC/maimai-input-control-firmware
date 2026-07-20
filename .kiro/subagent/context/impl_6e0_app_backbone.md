# impl_6e0_app_backbone — app_state 连接骨架 + Slint 主窗口壳

## 状态:已完成

## 目标
在 `control_software` 中实现可运行的主窗口地基:刷新/选择设备、连接(HELLO)、
显示 DEVICE_INFO、断开;为后续 tab(#6e 配置/#6f 绑区/#6g 曲线)预留挂载点。

## 架构结论(复用,未重造)
- `proto::{Frame, encode, Decoder, HostCmd, DeviceInfo}` 直接复用。
- `io::{list_devices, spawn, IoHandle, IoEvent}` 直接复用,`IoHandle::try_recv()`
  非阻塞轮询。
- `comport::{identify_ports, CdcFunction, IdentifiedPort}` 直接复用,按
  `port_name` 与 `io::list_devices()` 结果关联出功能。
- Slint 单线程模式:事件循环在主线程;`AppController` 用
  `Rc<RefCell<AppController>>` 在 callback 与 `slint::Timer`(Repeated,16ms)
  闭包间共享,不引入 async/tokio。

## 已完成改动
1. `src/app_state/mod.rs`(重写,原为占位):
   - `ConnState{Disconnected,Connecting,Connected}`
   - `DeviceEntry{port_name,function,label}`
   - `AppController{devices,io,state,device_info,seq,last_error,status_text}`
   - 方法:`new`/`refresh_devices`/`connect(index)`/`disconnect`/`poll`/
     `next_seq`(私有)/`status_line`/`device_labels`/`device_count`/`state`/
     `last_error`/`device_info_text`。纯逻辑,不依赖 Slint 类型。
   - `handle_event`/`handle_frame` 为私有辅助;DEVICE_INFO 帧解析成功后
     `state=Connected`,失败记录 `last_error` 但不改状态。
   - `#![allow(dead_code)]`:`device_count`/`state`/`last_error` 目前无 UI
     消费者,是留给后续 tab 的既定接口。
   - 单测 7 个:排序、seq 回绕、DEVICE_INFO 帧处理(成功/失败)、断开事件、
     越界连接报错、断开 no-op。全部通过。

2. `ui/app.slint`(重写):
   - `import { Button, ComboBox, VerticalBox, HorizontalBox, GroupBox, ListView } from "std-widgets.slint";`
   - 属性:`in property <[string]> device_labels`、
     `in-out property <int> selected_device`、
     `in property <string> conn_status`、
     `in property <string> device_info_text`。
   - callback:`refresh()`/`connect_clicked()`/`disconnect_clicked()`。
   - 布局:顶部连接栏(ComboBox+刷新/连接/断开按钮+状态行)、DEVICE_INFO
     展示 GroupBox、中部占位 GroupBox("配置 / 绑区 / 曲线 页面(后续实现)")
     ——这是 #6e/#6f/#6g 的挂载点,后续把这个 GroupBox 替换为 TabWidget 或
     直接在其位置插入业务组件即可。

3. `src/main.rs`(重写):
   - `slint::include_modules!()` + `mod app_state; mod comport; mod io; mod proto;`
   - `Rc<RefCell<AppController>>` 装配;启动即 `refresh_devices()` 并
     `push_devices()` 回填 `device_labels`/`selected_device`。
   - `on_refresh`/`on_connect_clicked`/`on_disconnect_clicked` 均用
     `window.as_weak()` + `ctrl.clone()` 转发到 `AppController` 方法,连接失败时
     直接把 `anyhow::Error` 文本写入 `conn_status`。
   - `slint::Timer::default()` + `start(Repeated, 16ms, ...)`:每 tick 调用
     `ctrl.borrow_mut().poll()`,再回填 `conn_status`/`device_info_text`。
   - `push_devices()`:`ModelRc::new(VecModel::from(Vec<SharedString>))` 回填
     下拉;若当前选中项越界则重置为 0,无设备则置 -1。

## 验证结果
- `cargo build`:Exit 0。仅有的警告全部是 `comport`/`io` 模块中**既有**、
  本任务未触碰的字段/函数(`plan_assignment`/`apply_assignment`/
  `recv_timeout`/`DeviceCandidate` 部分字段等,均为 #6c/#6d 遗留的暂未消费
  的公开 API,不属于本次改动引入)。`app_state`/`main.rs` 本身零警告。
- `cargo test`:31 passed(其中 app_state 新增 7 个),0 failed。
- 未尝试运行 GUI(按要求,无显示环境会阻塞),仅以 build+test 验证。

## 公开契约(供后续 tab / 主 agent 参考)
- `app_state::AppController` 公开 API:见上文方法列表。
- Slint 属性/回调契约:`device_labels`/`selected_device`/`conn_status`/
  `device_info_text`/`refresh()`/`connect_clicked()`/`disconnect_clicked()`。
- 挂载点:`ui/app.slint` 中部 `GroupBox { title: "功能页面"; ... }` 是
  #6e(配置)/#6f(绑区)/#6g(曲线)三个后续页面的插入位置;`AppController`
  可继续加字段(如当前 tab 索引)与方法,复用本任务已建好的连接/轮询骨架。
- 真机连接验证留给 #7,本任务未做真机测试。

## 待办
无(本任务范围内已全部完成)。后续 tab 任务由主 agent 另行派发。
