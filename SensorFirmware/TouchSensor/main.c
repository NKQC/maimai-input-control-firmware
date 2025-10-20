#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "cy_syslib.h"

#include "module/capsense/capsense_module.h"
#include "module/i2c/i2c_module.h"
#include "module/led/led_module.h"

#define CY_ASSERT_FAILED          (0u)

static uint64_t _last_scan_time = 0;
static volatile uint32_t _systick_overflow_count = 0;

#if CY_CAPSENSE_BIST_EN
static uint32_t sensor_cp = 0;
static cy_en_capsense_bist_status_t status;
#endif /* CY_CAPSENSE_BIST_EN */


static uint64_t _get_system_time_us(void);
static void _update_scan_rate(void);
static inline uint8_t _get_i2c_address(void);

#if CY_CAPSENSE_BIST_EN
static void _measure_sensor_cp(void);
#endif /* CY_CAPSENSE_BIST_EN */


// 主函数：初始化系统并运行主循环
int main(void)
{
    static cy_rslt_t result; 
    result = CY_RSLT_SUCCESS;

    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    __enable_irq();
    SysTick_Config(SystemCoreClock / 1000);

    led_init();
    led_on();

    i2c_init(_get_i2c_address());
    capsense_init();
    capsense_start_scan();
    _last_scan_time = _get_system_time_us();

    for (;;)
    {
        if (!capsense_is_busy())
        {
            capsense_process_widgets();
            _update_scan_rate();
            capsense_update_touch_status();
            capsense_apply_threshold_changes();
#if CY_CAPSENSE_BIST_EN
            _measure_sensor_cp();
#endif
            capsense_start_scan();
        }
    }
}

// 更新扫描速率
static void _update_scan_rate(void)
{
    static uint32_t scan_count = 0;
    static uint64_t last_rate_update_time = 0;
    static uint64_t current_time = 0;
    static uint64_t time_diff = 0;

    scan_count++;

    current_time = _get_system_time_us();
    time_diff = current_time - last_rate_update_time;

    if (time_diff >= 1000000)
    {
        i2c_set_scan_rate((uint16_t)((scan_count * 1000000ULL) / time_diff));
        scan_count = 0;
        last_rate_update_time = current_time;
    }
}

#if CY_CAPSENSE_BIST_EN
// 测量传感器电极自电容
static void _measure_sensor_cp(void)
{
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_CAP0_WDGT_ID,
                                                  CY_CAPSENSE_CAP0_SNS0_ID,
                                             &sensor_cp, &cy_capsense_context);
}
#endif

// 获取I2C从机地址
static inline uint8_t _get_i2c_address(void)
{
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_3_PORT, ADDR_PIN_P3_3_NUM, CY_GPIO_DM_PULLUP, 1, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_2_PORT, ADDR_PIN_P3_2_NUM, CY_GPIO_DM_PULLUP, 1, HSIOM_SEL_GPIO);
    Cy_SysLib_DelayUs(100);  // 等待GPIO稳定
    static uint8_t addr_bits; 
    addr_bits = 0;

    if (Cy_GPIO_Read(ADDR_PIN_P3_3_PORT, ADDR_PIN_P3_3_NUM))
    {
        addr_bits |= 0x04;
    }

    if (Cy_GPIO_Read(ADDR_PIN_P3_2_PORT, ADDR_PIN_P3_2_NUM))
    {
        addr_bits |= 0x02;
    }
    
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_3_PORT, ADDR_PIN_P3_3_NUM, CY_GPIO_DM_STRONG, 1, P3_3_CPUSS_SWD_CLK);
    Cy_GPIO_Pin_FastInit(ADDR_PIN_P3_2_PORT, ADDR_PIN_P3_2_NUM, CY_GPIO_DM_STRONG, 1, P3_2_CPUSS_SWD_DATA);

    return I2C_SLAVE_BASE_ADDR + addr_bits;
}

// 获取系统时间（微秒）
static uint64_t _get_system_time_us(void)
{
    static uint32_t systick_val;
    static uint32_t overflow_count;

    __disable_irq();

    systick_val = SysTick->VAL;
    overflow_count = _systick_overflow_count;

    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) && (systick_val > (SysTick->LOAD >> 1)))
    {
        overflow_count++;
    }

    __enable_irq();

    static uint32_t ticks_in_current_ms;
    static uint64_t total_us;
    ticks_in_current_ms = SysTick->LOAD - systick_val;
    total_us = (uint64_t)overflow_count * 1000 + (ticks_in_current_ms * 1000) / SysTick->LOAD;

    return total_us;
}

// SysTick中断处理
void SysTick_Handler(void)
{
    _systick_overflow_count++;
}
