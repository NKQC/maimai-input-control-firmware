#include "i2c_module.h"
#include "../capsense/capsense_module.h"
#include "../led/led_module.h"

static cy_stc_scb_i2c_context_t i2c_context;
static uint16_t g_scan_rate_per_second = 0;
static volatile i2c_control_reg_t g_control_reg = { .raw = 0 };
// 新增：触摸状态的主循环快照，I2C读取返回该值以避免抖动
static volatile uint16_t g_touch_status_snapshot = 0u;

// 使用独立的读写缓冲区，避免读缓冲被写入数据污染
static uint8_t _i2c_write_buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t _i2c_read_buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t _current_register = REG_SCAN_RATE; // 缺省寄存器指针

static void _i2c_slave_isr(void);

// 初始化I2C从机模块
void i2c_init(uint8_t slave_address)
{
    static cy_en_scb_i2c_status_t status_local; 
    status_local = CY_SCB_I2C_SUCCESS;

    static const cy_stc_sysint_t i2c_intr_config =
    {
        .intrSrc = scb_1_IRQ,
        .intrPriority = I2C_SLAVE_INTR_PRIORITY,
    };

    static cy_stc_scb_i2c_config_t i2c_config; 
    i2c_config.i2cMode = CY_SCB_I2C_SLAVE;
    i2c_config.useRxFifo = true;
    i2c_config.useTxFifo = true;
    i2c_config.slaveAddress = slave_address;
    i2c_config.slaveAddressMask = 0xFE;
    i2c_config.acceptAddrInFifo = false;
    i2c_config.ackGeneralAddr = false;
    i2c_config.enableWakeFromSleep = false;
    i2c_config.enableDigitalFilter = false;
    i2c_config.lowPhaseDutyCycle = 8;
    i2c_config.highPhaseDutyCycle = 8;

    status_local = Cy_SCB_I2C_Init(scb_1_HW, &i2c_config, &i2c_context);

    if (CY_SCB_I2C_SUCCESS == status_local)
    {
        Cy_SysInt_Init(&i2c_intr_config, _i2c_slave_isr);
        NVIC_ClearPendingIRQ(i2c_intr_config.intrSrc);
        NVIC_EnableIRQ(i2c_intr_config.intrSrc);

        // 缺省：开启LED触摸提示（CONTROL.bit5=1）
        g_control_reg.bits.led_feedback_en = 1u;

        // 配置独立的读写缓冲
        Cy_SCB_I2C_SlaveConfigReadBuf(scb_1_HW, _i2c_read_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);
        Cy_SCB_I2C_SlaveConfigWriteBuf(scb_1_HW, _i2c_write_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);

        // 预填充读缓冲为缺省寄存器的值（高字节在前）
        uint16_t v = 0;
        v = i2c_handle_register_read(_current_register);
        _i2c_read_buffer[0] = (uint8_t)(v >> 8);
        _i2c_read_buffer[1] = (uint8_t)(v & 0xFF);

        Cy_SCB_I2C_Enable(scb_1_HW, &i2c_context);
    }

    if(status_local != CY_SCB_I2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

// 读取LED触摸提示开关状态（CONTROL.bit5）
bool i2c_led_feedback_enabled(void)
{
    return (g_control_reg.bits.led_feedback_en != 0u);
}

// 新增：由主循环提交触摸状态快照，I2C读取统一返回该值
void i2c_set_touch_status_snapshot(uint16_t status)
{
    g_touch_status_snapshot = status;
}

// I2C从机中断处理
static void _i2c_slave_isr(void)
{
    Cy_SCB_I2C_Interrupt(scb_1_HW, &i2c_context);

    static uint32_t slave_status;
    slave_status = Cy_SCB_I2C_SlaveGetStatus(scb_1_HW, &i2c_context);

    // 调试：记录slave_status值
    static uint32_t debug_status_count = 0;
    debug_status_count++;
    
    // 写事务进行中：主机正在写入数据
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_WR_BUSY))
    {
        // 写事务正在进行，等待完成
        // 不需要特殊处理，等待WR_CMPLT事件
    }

    // 写事务完成：处理寄存器写入或寄存器指针设置
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_WR_CMPLT))
    {
        static size_t wc;
        wc = Cy_SCB_I2C_SlaveGetWriteTransferCount(scb_1_HW, &i2c_context);
        
        if (wc >= 3)
        {
            // 寄存器写入：reg_addr + 2字节数据（大端序）
            static uint8_t reg_addr;
            static uint16_t value;
            reg_addr = _i2c_write_buffer[0];
            value = ((uint16_t)_i2c_write_buffer[1] << 8) | (uint16_t)_i2c_write_buffer[2];
            i2c_handle_register_write(reg_addr, value);
            _current_register = reg_addr;
        }
        else if (wc >= 1)
        {
            // 仅设置寄存器指针，准备后续读取
            _current_register = _i2c_write_buffer[0];
        }
        
        Cy_SCB_I2C_SlaveClearWriteStatus(scb_1_HW, &i2c_context);
        
        // 重新配置写缓冲区以接收下一次写入
        Cy_SCB_I2C_SlaveConfigWriteBuf(scb_1_HW, _i2c_write_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);
    }

    // 读事务进行中：主机正在读取数据
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_RD_BUSY))
    {
        // 读事务正在进行，等待完成
    }

    // 读事务开始：主机即将读取数据，准备读缓冲区
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_RD_IN_FIFO))
    {
        // 读取当前寄存器的值并准备到读缓冲区（大端序）
        static uint16_t reg_value;
        reg_value = i2c_handle_register_read(_current_register);
        _i2c_read_buffer[0] = (uint8_t)(reg_value >> 8);  // 高字节
        _i2c_read_buffer[1] = (uint8_t)(reg_value & 0xFF); // 低字节
        
        // 重新配置读缓冲区
        Cy_SCB_I2C_SlaveConfigReadBuf(scb_1_HW, _i2c_read_buffer, 2, &i2c_context);
    }

    // 读事务完成：清除状态
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_RD_CMPLT))
    {
        Cy_SCB_I2C_SlaveClearReadStatus(scb_1_HW, &i2c_context);
    }
}

// 设置扫描速率
void i2c_set_scan_rate(uint16_t rate)
{
    g_scan_rate_per_second = rate;
}

// I2C寄存器读取
uint16_t i2c_handle_register_read(uint8_t reg_addr)
{
    // 噪声只读寄存器范围判断
    if (reg_addr >= REG_NOISE_TH_BASE && reg_addr < (REG_NOISE_TH_BASE + CAPSENSE_WIDGET_COUNT))
    {
        return capsense_get_noise_th((uint8_t)(reg_addr - REG_NOISE_TH_BASE));
    }
    if (reg_addr >= REG_NNOISE_TH_BASE && reg_addr < (REG_NNOISE_TH_BASE + CAPSENSE_WIDGET_COUNT))
    {
        return capsense_get_nnoise_th((uint8_t)(reg_addr - REG_NNOISE_TH_BASE));
    }
    // 总触摸电容只读寄存器组
    if (reg_addr >= REG_TOTAL_TOUCH_CAP_BASE && reg_addr < (REG_TOTAL_TOUCH_CAP_BASE + CAPSENSE_WIDGET_COUNT))
    {
        return capsense_get_total_touch_cap((uint8_t)(reg_addr - REG_TOTAL_TOUCH_CAP_BASE));
    }
    // 触摸电容设置寄存器组（增量/绝对）范围判断
    if (reg_addr >= REG_TOUCH_CAP_SETTING_BASE && reg_addr < (REG_TOUCH_CAP_SETTING_BASE + CAPSENSE_WIDGET_COUNT))
    {
        uint8_t idx = (uint8_t)(reg_addr - REG_TOUCH_CAP_SETTING_BASE);
        // 读取时根据模式返回：绝对模式返回总触摸电容步进；相对模式返回原始编码值
        if (g_control_reg.bits.absolute_mode) {
            return capsense_get_total_touch_cap(idx);
        } else {
            return capsense_sensitivity_to_raw_count(capsense_get_touch_sensitivity(idx));
        }
    }

    switch (reg_addr)
    {
        case REG_SCAN_RATE:
            return g_scan_rate_per_second;

        case REG_TOUCH_STATUS:
            return g_touch_status_snapshot | 0x8000;

        case REG_CONTROL:
        {
            return g_capsense_async.raw & 0xFFFF;
        }
            // 触摸电容设置寄存器组已改为基址范围判断（见函数前部）

        default:
            return 0x0000;
    }
}

// I2C寄存器写入
void i2c_handle_register_write(uint8_t reg_addr, uint16_t value)
{
    // 触摸电容设置寄存器组（增量/绝对）范围写入
    if (reg_addr >= REG_TOUCH_CAP_SETTING_BASE && reg_addr < (REG_TOUCH_CAP_SETTING_BASE + CAPSENSE_WIDGET_COUNT))
    {
        uint8_t idx = (uint8_t)(reg_addr - REG_TOUCH_CAP_SETTING_BASE);
        if (g_control_reg.bits.absolute_mode) {
            // 绝对模式：value 为总触摸电容步进（0.01pF），进行边界检查并换算为增量
            uint16_t cp_base = capsense_get_cp_base_steps(idx);
            uint16_t total = value;
            if (total > TOUCH_CAP_TOTAL_MAX_STEPS) {
                total = TOUCH_CAP_TOTAL_MAX_STEPS;
            }
            // 计算增量（可为负值），并按规则夹取
            int32_t delta = (int32_t)total - (int32_t)cp_base;
            if (delta > (int32_t)TOUCH_SENSITIVITY_MAX_STEPS) {
                delta = (int32_t)TOUCH_SENSITIVITY_MAX_STEPS;
            } else if (delta < -(int32_t)TOUCH_SENSITIVITY_MAX_STEPS) {
                delta = -(int32_t)TOUCH_SENSITIVITY_MAX_STEPS;
            }
            // 正向增量的最小步进（FULL模式下为10步）
            if (delta > 0 && delta < (int32_t)TOUCH_INCREMENT_MIN_STEPS) {
                delta = (int32_t)TOUCH_INCREMENT_MIN_STEPS;
            }
            capsense_set_touch_sensitivity(idx, (int16_t)delta);
        } else {
            // 相对模式：value 为原始编码（0..8191），转换为步进偏移量并累积到当前值
            int16_t offset_steps = capsense_raw_count_to_sensitivity(value);
            int16_t current_steps = capsense_get_touch_sensitivity(idx);
            int16_t new_steps = current_steps + offset_steps;
            capsense_set_touch_sensitivity(idx, new_steps);
        }
        return;
    }
    switch (reg_addr)
    {
        case REG_CONTROL:
        {
            i2c_control_reg_t in; 
            in.raw = value;
            
            // LED控制
            led_set_state(in.bits.led_on);
            
            // 复位请求
            if (in.bits.reset_req) { 
                NVIC_SystemReset(); 
            }
            
            // 校准请求：仅置位全局异步标志，由主循环在CapSense空闲时执行
            if (in.bits.calibrate_req) {
                capsense_request_calibration();
            }
            
            // 完整更新g_control_reg，确保读写一致
            // 注意：calibrate_req和calibration_done由g_capsense_async管理，这里保持g_control_reg同步
            // 直接使用raw赋值，避免位域操作的潜在异常
            g_control_reg.raw = in.raw;
            
            break;
        }

            // 触摸电容设置寄存器组写入已改为基址范围判断（见函数前部）
            break;

        default:
            // 噪声只读寄存器组：写入忽略
            break;
    }
}