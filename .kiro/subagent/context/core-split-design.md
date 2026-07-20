# core1/core0 分核设计 (任务2)

## 目标
- core1: PSoC SPI 唯一所有者。固定 1ms 周期传感器循环。经 seqlock 发布 touch/snapshot/stats。
- core0: 纯协议(USB/host_cmd/telemetry 打包解析)。只读 core1 发布的缓存态。低频 SPI 操作(param/mode/apply/get_raw)经命令信箱投递给 core1 执行。
- core1 保持 multicore_lockout_victim(flash 写时被 core0 暂停,SPI 短暂停,可接受)。
- 关键: PSoC SPI 是有状态流水线协议,严禁两核交错→core1 独占。

## 现状 SPI 触点(全在 core0)
- psoc.cpp Psoc::update(): 触控读 + (遥测)snapshot_pump + (500ms)stats。loop() 每轮调。
- Psoc::set_param/get_param/get_raw/apply_params/set_mode → _spi.xxx。SensorLink host_cmd handler(core0) + CsdConfig download/capture 调。
- setup() 内(core1 启动前): PsocUpdater::run(SWD,非SPI)、prepare_flash_indicator(_spi.indicator_on)、psoc->update()一次。这些在 core1 启动前 core0 直接做,无冲突。

## 实现

### psoc.h/.cpp 重构
1. 新增 seqlock 发布: `volatile uint32_t _pub_seq`(touch), `volatile uint32_t _snap_seq`(snapshot)。core1 写: seq++(奇);__dmb();写数据;__dmb();seq++(偶)。core0 读: 循环 s1=seq;__dmb();读;__dmb();s2=seq; while(s1!=s2||(s1&1))。stats(_samples_per_sec/_scan_period_us)为单 u32 天然原子,无需 seqlock。
2. touch_mask()/link_ok()/snapshot() 这些 core0 读接口改为 seqlock 读取内部 `_pub_touch_mask`/`_pub_link_ok`/`_pub_snapshot`。
3. 命令信箱(core0→core1):
```
enum class SpiOp:uint8_t{NONE,SET_PARAM,GET_PARAM,GET_RAW,SET_MODE,APPLY};
volatile SpiOp _cmd_op; volatile uint8_t _cmd_ch,_cmd_pid; volatile uint32_t _cmd_val,_cmd_result;
volatile bool _cmd_ok,_cmd_pending,_cmd_done;
```
core0 侧 `_submit(op,ch,pid,val,out*)`: 填参;_cmd_done=false;__dmb();_cmd_op=op;_cmd_pending=true; 自旋 while(!_cmd_done){超时100ms返回false; tight_loop_contents();} out赋值;_cmd_pending=false;_cmd_op=NONE;返回_cmd_ok。set_param/get_param/get_raw/apply_params/set_mode 全部改为调 _submit(签名不变)。
4. core1 执行: `_spi_service()`(core1 每周期调一次): 先处理信箱(if _cmd_pending&&!_cmd_done: switch 调 _spi.对应方法, 填 _cmd_result/_cmd_ok, __dmb(), _cmd_done=true); 再触控读(_spi.read_touch)→seqlock 发布; 遥测激活则 _spi.snapshot_pump→_snap_seq 发布; 每500ms _spi.get_stats 折算(写 _samples_per_sec/_scan_period_us)。latency_note(g_lat_spi_us) 保留。
5. `core1_run()`(core1_entry 调,无限循环): while(1){ uint32_t cs=time_us_32(); _spi_service(); while(time_us_32()-cs<1000u) tight_loop_contents(); } 固定 1ms 周期(每周期时间一致)。
6. `update()`: 改为空(或保留供 setup 单次直调 _spi_service 的入口)。loop() 不再调 update()。setup() 里 core1 启动前的那次 psoc->update() 改为直接 _spi_service() 一次(core1 未起,直调安全)。
7. download_to_psoc/capture: CsdConfig 调 psoc->set_mode/set_param/apply/get_param → 自动走信箱。download 仅半自动+valid 时循环(默认auto便宜)。capture 读324参数=324信箱往返(~数百ms,罕见,可接受)。这些发生在 core0,core1 执行,无死锁(psoc 对外API走信箱,_spi 直调只在core1)。

### main.cpp
- core1_entry: `multicore_lockout_victim_init(); Psoc::getInstance()->core1_run();`(不再 tight_loop 空转)。
- setup(): 保持 psoc->init()/PsocUpdater::run/prepare_flash_indicator 在 core1 启动前;那次 `psoc->update()` 保留(此时 core1 未启动,update 内部直调 SPI 一次 OK——或改为直调 _spi_service())。multicore_launch_core1 位置不变(watchdog_enable 前)。
- loop(): 删除 `psoc->update();`。其余(updater->update() 读缓存态; download 块; UsbComm; 租约; SensorLink::tick 读snapshot(); BindingService::tick 读touch_mask(); game_io 读touch_mask(); flash 块; vendor_service; LED)全部不变——它们读的都是 core1 发布的缓存。

## 安全/正确性要求
- 所有跨核共享用 volatile + __dmb() 屏障(RP2040 无 cache,volatile+屏障足够)。seqlock 保护多字(u64 mask / snapshot struct)防撕裂。
- core1 必须先 multicore_lockout_victim_init() 再进 core1_run(否则 flash 死锁)。
- core0 _submit 自旋期间不喂狗;单命令<2ms,download<1s,均 < 看门狗5s。
- 不改任何 Psoc 对外 public 方法签名(SensorLink/CsdConfig 调用处零改动)。

## 验证
build-rp SUCCESS; 烧录; probe 4/4; soak(遍历配置/遥测/set_param/set_mode/calibrate/capture/校准)PASS; GUI 强杀+重连 OK。延迟组成 SPI/proc 仍合理。

## 状态
- [ ] psoc.h/.cpp 分核重构
- [ ] main.cpp core1_run + 去 update
- [ ] build + flash + soak 验证
