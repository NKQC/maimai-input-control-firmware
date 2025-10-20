#include "i2c_module.h"
#include "../capsense/capsense_module.h"
#include "../led/led_module.h"

static cy_stc_scb_i2c_context_t i2c_context;
static uint16_t g_scan_rate_per_second = 0;
static uint16_t g_led_control_reg = 0;

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
    switch (reg_addr)
    {
        case REG_SCAN_RATE:
            return g_scan_rate_per_second;

        case REG_TOUCH_STATUS:
            // 读取触摸状态时熄灭LED
            return capsense_get_touch_status_bitmap();

        case REG_LED_CONTROL:
            return g_led_control_reg;

        case REG_CAP0_THRESHOLD:
        case REG_CAP1_THRESHOLD:
        case REG_CAP2_THRESHOLD:
        case REG_CAP3_THRESHOLD:
        case REG_CAP4_THRESHOLD:
        case REG_CAP5_THRESHOLD:
        case REG_CAP6_THRESHOLD:
        case REG_CAP7_THRESHOLD:
        case REG_CAP8_THRESHOLD:
        case REG_CAP9_THRESHOLD:
        case REG_CAPA_THRESHOLD:
        case REG_CAPB_THRESHOLD:
            return capsense_get_threshold(reg_addr - REG_CAP0_THRESHOLD);

        default:
            return 0x0000;
    }
}

// I2C寄存器写入
void i2c_handle_register_write(uint8_t reg_addr, uint16_t value)
{
    switch (reg_addr)
    {
        case REG_LED_CONTROL:
            g_led_control_reg = value;
            led_set_state(value & 0x01);
            break;

        case REG_CAP0_THRESHOLD:
        case REG_CAP1_THRESHOLD:
        case REG_CAP2_THRESHOLD:
        case REG_CAP3_THRESHOLD:
        case REG_CAP4_THRESHOLD:
        case REG_CAP5_THRESHOLD:
        case REG_CAP6_THRESHOLD:
        case REG_CAP7_THRESHOLD:
        case REG_CAP8_THRESHOLD:
        case REG_CAP9_THRESHOLD:
        case REG_CAPA_THRESHOLD:
        case REG_CAPB_THRESHOLD:
            capsense_set_threshold(reg_addr - REG_CAP0_THRESHOLD, value);
            break;

        default:
            break;
    }
}