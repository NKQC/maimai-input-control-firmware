# PSoC Milestone 3 - SPI 从机重写任务

## 任务目标
重写 PSoC4 (CY8C4147AZI) main.c，实现：
- SPI 从机 (SCB0, SLAVE, MODE0, 8-bit, MSB-first, CS ActiveLow)
- Capsense 初始化和扫描（36个按键）
- W 状态灯 (P1.6) ping/pong 闪烁
- FW_VERSION 0x00000400

## 已完成步骤

### 1. 架构确认（步骤1-2）
- SCB0 SPI 从机配置宏：`scb_0_HW`、`scb_0_IRQ`、`scb_0_config`
- Capsense 宏：`CYBSP_CSD_HW`、`CYBSP_CSD_IRQ`、36个BUTTON_WDGT_ID
- W 状态灯宏：`CYBSP_LED_SLD3_PORT` (GPIO_PRT1)、`CYBSP_LED_SLD3_NUM` (6)

### 2. 主.c 完整重写（步骤3）
**关键改动：**
- 顶部宏定义：FW_VERSION、帧常数 (MAGIC 0xA5, PING 0x01, PONG 0x02, SIZE 7)
- 文件级变量：`spi_context`、`pong_frame[7]`
- 6 个函数：
  1. `capsense_isr()` - Capsense 中断处理
  2. `initialize_capsense()` - Capsense 初始化+IRQ
  3. `spi_load_pong()` - 构造 PONG 帧（seq + FW_VERSION LE）写入 TX FIFO
  4. `spi_slave_init()` - SPI 初始化+预装 PONG
  5. `spi_slave_task()` - 接收 PING、toggle LED（每4次）、响应 PONG
  6. `main()` - 初始化流程 + 主循环

**移除内容：**
- EZI2C tuner、PWM1/2、SMART_IO（配置不存在）
- Capsense LED 逻辑、RunTuner（不需要）
- 只保留 Capsense Init/Enable/Scan/Process 基础流程

### 3. 编译验证（步骤4）
✓ `make build -j8` 成功
✓ 产生 hex：`build/APP_CY8CKIT-149/Debug/mtb-example-psoc4-capsense-smartsense-buttons-slider.hex`
✓ 内存占用：Flash 25364/131072 字节

## 帧协议确认
- RP2040 主机每 20ms 发 7 字节 PING，同步全双工读 PONG
- PING 帧：[0xA5][0x01][seq][4 字节数据]
- PONG 帧：[0xA5][0x02][seq][FW_VERSION 小端 4 字节]
- **关键**：SPI 从机预装 PONG 到 TX FIFO，主机 clock 时同步读取

## 状态灯逻辑
- P1.6 共阳配置 (active-low)
- 初始化时写 1（关闭）
- 每收到 4 个 PING 触发 1 次 Inv（切换亮灭）

## 验证完成
编译通过，无错误，hex 已生成。

## 下一步（如果需要）
- 烧录到硬件验证通信
- 检查 SPI 帧同步和 LED 闪烁
