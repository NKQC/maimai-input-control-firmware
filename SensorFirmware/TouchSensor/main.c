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
static uint16_t _last_touch_status = 0;

static inline uint64_t _get_system_time_us(void);
static inline void _update_scan_rate(void);
static inline uint8_t _get_i2c_address(void);

// 主函数：初始化系统并运行主循环
int main(void)
{
    static cy_rslt_t result; 
    register uint16_t status = 0;
    result = CY_RSLT_SUCCESS;

    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    __enable_irq();
    SysTick_Config(SystemCoreClock / 1000);

    led_init();

    i2c_init(_get_i2c_address());
    capsense_init();
    capsense_start_scan();
    _last_scan_time = _get_system_time_us();
    led_on();
    for (;;)
    {
        if (!capsense_is_busy())
        {
            capsense_handle_async_ops();
            capsense_process_widgets();
            capsense_update_touch_status();
            capsense_apply_threshold_changes();
            status = capsense_get_touch_status_bitmap();
            i2c_set_touch_status_snapshot(status);
            if (i2c_led_feedback_enabled()) {
                led_set_state(status != _last_touch_status);
                _last_touch_status = status;
            }
            _update_scan_rate();
            capsense_start_scan();
        }
    }
}

// 更新扫描速率
static inline void _update_scan_rate(void)
{
    static uint32_t scan_count = 0;
    static uint64_t last_update_us = 0;

    scan_count++;

    uint64_t now_us = _get_system_time_us();
    uint64_t elapsed = now_us - last_update_us;

    if (elapsed >= 1000000ULL)
    {
        i2c_set_scan_rate((uint16_t)((scan_count * 1000000ULL) / elapsed));
        scan_count = 0;
        last_update_us = now_us;
    }
}



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
static inline uint64_t _get_system_time_us(void)
{
    __disable_irq();
    uint32_t systick_val = SysTick->VAL;
    uint32_t overflow = _systick_overflow_count;

    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) && (systick_val > (SysTick->LOAD >> 1)))
    {
        overflow++;
    }

    __enable_irq();

    uint32_t ticks_in_ms = SysTick->LOAD - systick_val;
    return (uint64_t)overflow * 1000ULL + ((uint64_t)ticks_in_ms * 1000ULL) / SysTick->LOAD;
}

// SysTick中断处理
void SysTick_Handler(void)
{
    _systick_overflow_count++;
}
