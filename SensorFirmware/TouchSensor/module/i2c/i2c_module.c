/******************************************************************************
* File Name: i2c_module.c
*
* Description: I2C从机模块实现
*
*******************************************************************************/

#include "i2c_module.h"
#include "../capsense/capsense_module.h"
#include "../led/led_module.h"

/*******************************************************************************
* 全局变量定义
*******************************************************************************/
cy_stc_scb_i2c_context_t i2c_context;
uint16_t g_scan_rate_per_second = 0;
uint16_t g_led_control_reg = 0;

/*******************************************************************************
* 静态变量
*******************************************************************************/
static uint8_t _i2c_buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t _current_register = 0;

/*******************************************************************************
* 静态函数声明
*******************************************************************************/
static void _i2c_slave_isr(void);
static uint8_t _read_address_pins(void);

/*******************************************************************************
* Function Name: i2c_init
********************************************************************************
* Summary: 初始化I2C从机模块
*******************************************************************************/
void i2c_init(void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    uint8_t slave_addr;

    /* 配置地址引脚为上拉输入 */
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_3_PORT, ADDR_PIN_P3_3_NUM, CY_GPIO_DM_PULLUP, 1, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_2_PORT, ADDR_PIN_P3_2_NUM, CY_GPIO_DM_PULLUP, 1, HSIOM_SEL_GPIO);

    /* 读取地址引脚并计算从机地址 */
    slave_addr = I2C_SLAVE_BASE_ADDR | _read_address_pins();

    /* I2C从机中断配置 */
    const cy_stc_sysint_t i2c_intr_config =
    {
        .intrSrc = scb_1_IRQ,
        .intrPriority = I2C_SLAVE_INTR_PRIORITY,
    };

    /* I2C从机配置 */
    cy_stc_scb_i2c_config_t i2c_config =
    {
        .i2cMode = CY_SCB_I2C_SLAVE,
        .useRxFifo = false,
        .useTxFifo = false,
        .slaveAddress = slave_addr,
        .slaveAddressMask = 0xFE,
        .acceptAddrInFifo = false,
        .ackGeneralAddr = false,
        .enableWakeFromSleep = false,
        .enableDigitalFilter = false,
        .lowPhaseDutyCycle = 8,
        .highPhaseDutyCycle = 8,
    };

    /* 初始化I2C从机 */
    status = Cy_SCB_I2C_Init(scb_1_HW, &i2c_config, &i2c_context);

    if (CY_SCB_I2C_SUCCESS == status)
    {
        /* 初始化I2C中断 */
        Cy_SysInt_Init(&i2c_intr_config, _i2c_slave_isr);
        NVIC_ClearPendingIRQ(i2c_intr_config.intrSrc);
        NVIC_EnableIRQ(i2c_intr_config.intrSrc);

        /* 配置I2C从机缓冲区 */
        Cy_SCB_I2C_SlaveConfigReadBuf(scb_1_HW, _i2c_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);
        Cy_SCB_I2C_SlaveConfigWriteBuf(scb_1_HW, _i2c_buffer, I2C_SLAVE_BUFFER_SIZE, &i2c_context);

        /* 启用I2C从机 */
        Cy_SCB_I2C_Enable(scb_1_HW, &i2c_context);
    }

    if(status != CY_SCB_I2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*******************************************************************************
* Function Name: _i2c_slave_isr
********************************************************************************
* Summary: I2C从机中断处理函数
*******************************************************************************/
static void _i2c_slave_isr(void)
{
    Cy_SCB_I2C_Interrupt(scb_1_HW, &i2c_context);
    
    /* 处理I2C从机事件 */
    uint32_t slave_status = Cy_SCB_I2C_SlaveGetStatus(scb_1_HW, &i2c_context);
    
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_WR_CMPLT))
    {
        /* 写操作完成 - 处理寄存器写入 */
        if (Cy_SCB_I2C_SlaveGetWriteTransferCount(scb_1_HW, &i2c_context) >= 3)
        {
            uint8_t reg_addr = _i2c_buffer[0];
            uint16_t value = (_i2c_buffer[1] << 8) | _i2c_buffer[2];
            i2c_handle_register_write(reg_addr, value);
        }
        Cy_SCB_I2C_SlaveClearWriteStatus(scb_1_HW, &i2c_context);
    }
    
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_RD_CMPLT))
    {
        /* 读操作完成 */
        Cy_SCB_I2C_SlaveClearReadStatus(scb_1_HW, &i2c_context);
    }
    
    if (0u != (slave_status & CY_SCB_I2C_SLAVE_ADDR_DONE))
    {
        /* 地址匹配 - 准备数据传输 */
        if (Cy_SCB_I2C_SlaveGetWriteTransferCount(scb_1_HW, &i2c_context) >= 1)
        {
            _current_register = _i2c_buffer[0];
            /* 准备读取数据 */
            uint16_t reg_value = i2c_handle_register_read(_current_register);
            _i2c_buffer[0] = (reg_value >> 8) & 0xFF;
            _i2c_buffer[1] = reg_value & 0xFF;
        }
        Cy_SCB_I2C_SlaveClearReadStatus(scb_1_HW, &i2c_context);
        Cy_SCB_I2C_SlaveClearWriteStatus(scb_1_HW, &i2c_context);
    }
}

/*******************************************************************************
* Function Name: _read_address_pins
********************************************************************************
* Summary: 读取P3.3和P3.2引脚确定地址位
* Return: 地址位 (0-3)
*******************************************************************************/
static uint8_t _read_address_pins(void)
{
    uint8_t addr_bits = 0;
    
    /* 读取P3.3作为位1 (值0或2) */
    if (Cy_GPIO_Read(ADDR_PIN_P3_3_PORT, ADDR_PIN_P3_3_NUM) == 0)
    {
        addr_bits |= 0x02;
    }
    
    /* 读取P3.2作为位0 (值0或1) */
    if (Cy_GPIO_Read(ADDR_PIN_P3_2_PORT, ADDR_PIN_P3_2_NUM) == 0)
    {
        addr_bits |= 0x01;
    }
    
    return addr_bits;
}

/*******************************************************************************
* Function Name: i2c_handle_register_read
********************************************************************************
* Summary: 处理I2C寄存器读取操作
* Parameters: reg_addr - 要读取的寄存器地址
* Return: 寄存器值
*******************************************************************************/
uint16_t i2c_handle_register_read(uint8_t reg_addr)
{
    switch (reg_addr)
    {
        case REG_SCAN_RATE:
            return g_scan_rate_per_second;
            
        case REG_TOUCH_STATUS:
            return g_touch_status_bitmap;
            
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
            return g_cap_thresholds[reg_addr - REG_CAP0_THRESHOLD];
            
        default:
            return 0x0000;
    }
}

/*******************************************************************************
* Function Name: i2c_handle_register_write
********************************************************************************
* Summary: 处理I2C寄存器写入操作
* Parameters: reg_addr - 要写入的寄存器地址
*            value - 要写入的值
*******************************************************************************/
void i2c_handle_register_write(uint8_t reg_addr, uint16_t value)
{
    switch (reg_addr)
    {
        case REG_LED_CONTROL:
            g_led_control_reg = value;
            /* 更新LED状态 */
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
            /* 只有阈值寄存器可写 */
            g_cap_thresholds[reg_addr - REG_CAP0_THRESHOLD] = value;
            g_threshold_changed = true;
            break;
            
        default:
            /* 其他寄存器为只读或保留 */
            break;
    }
}