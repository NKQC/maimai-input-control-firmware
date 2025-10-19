#include "i2c_module.h"
#include "../capsense/capsense_module.h"
#include "../led/led_module.h"

static cy_stc_scb_i2c_context_t i2c_context;
static uint16_t g_scan_rate_per_second = 0;
static uint16_t g_led_control_reg = 0;

static uint8_t _i2c_buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t _current_register = 0;

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
    i2c_config.useRxFifo = false;
    i2c_config.useTxFifo = false;
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

        Cy_SCB_I2C_SlaveConfigReadBuf(scb_1_HW, _i2c_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);
        Cy_SCB_I2C_SlaveConfigWriteBuf(scb_1_HW, _i2c_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);

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

    if (0u != (slave_status & CY_SCB_I2C_SLAVE_WR_CMPLT))
    {
        if (Cy_SCB_I2C_SlaveGetWriteTransferCount(scb_1_HW, &i2c_context) >= 3)
        {
            static uint8_t reg_addr; reg_addr = _i2c_buffer[0];
            static uint16_t value; value = (_i2c_buffer[1] << 8) | _i2c_buffer[2];
            i2c_handle_register_write(reg_addr, value);
        }
        Cy_SCB_I2C_SlaveClearWriteStatus(scb_1_HW, &i2c_context);
    }

    if (0u != (slave_status & CY_SCB_I2C_SLAVE_RD_CMPLT))
    {
        Cy_SCB_I2C_SlaveClearReadStatus(scb_1_HW, &i2c_context);
    }

    if (0u != (slave_status & CY_SCB_I2C_SLAVE_ADDR_DONE))
    {
        if (Cy_SCB_I2C_SlaveGetWriteTransferCount(scb_1_HW, &i2c_context) >= 1)
        {
            _current_register = _i2c_buffer[0];
            static uint16_t reg_value; reg_value = i2c_handle_register_read(_current_register);
            _i2c_buffer[0] = (reg_value >> 8) & 0xFF;
            _i2c_buffer[1] = reg_value & 0xFF;
        }
        Cy_SCB_I2C_SlaveClearReadStatus(scb_1_HW, &i2c_context);
        Cy_SCB_I2C_SlaveClearWriteStatus(scb_1_HW, &i2c_context);
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