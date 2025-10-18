/*******************************************************************************
* 文件名: main.c
* 描述: maimai触摸控制器主程序 - PSoC 4 CapSense I2C从机实现
* 功能: 动态地址I2C从机，基于寄存器的接口，LED控制
*******************************************************************************/


/*******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "cy_syslib.h"

/* 模块头文件 */
#include "module/capsense/capsense_module.h"
#include "module/i2c/i2c_module.h"
#include "module/led/led_module.h"


/*******************************************************************************
* 宏定义
*******************************************************************************/
#define CY_ASSERT_FAILED          (0u)

/*******************************************************************************
* 全局变量定义
*******************************************************************************/
/* 时间测量相关变量 */
static uint64_t _last_scan_time = 0;
static volatile uint32_t _systick_overflow_count = 0;

#if CY_CAPSENSE_BIST_EN
uint32_t sensor_cp = 0;
cy_en_capsense_bist_status_t status;
#endif /* CY_CAPSENSE_BIST_EN */


/*******************************************************************************
* 静态函数声明
*******************************************************************************/
static uint64_t _get_system_time_us(void);
static void _update_scan_rate(void);

#if CY_CAPSENSE_BIST_EN
static void _measure_sensor_cp(void);
#endif /* CY_CAPSENSE_BIST_EN */


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize I2C slave communication
*  - scan touch input continuously and update registers
*
* Return:
*  int
*
*******************************************************************************/
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: 主函数 - 初始化系统并运行主循环
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* 初始化设备和板级外设 */
    result = cybsp_init();

    /* 板级初始化失败，停止程序执行 */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* 使能全局中断 */
    __enable_irq();

    /* 初始化SysTick定时器用于时间测量 */
    SysTick_Config(SystemCoreClock / 1000); /* 1ms时钟 */

    /* 初始化LED模块 */
    led_init();

    /* 初始化I2C从机 */
    i2c_init();

    /* 初始化CapSense */
    capsense_init();

    /* 开始第一次扫描 */
    capsense_start_scan();
    _last_scan_time = _get_system_time_us();

    /* LED在进入主循环前点亮 */
    led_on();

    for (;;)
    {
        if (!capsense_is_busy())
        {
            /* 处理所有传感器 */
            capsense_process_widgets();

            /* 更新扫描速率计算 */
            _update_scan_rate();

            /* 更新触摸状态位图 */
            capsense_update_touch_status();
            
            /* 应用来自I2C的阈值更改 */
            capsense_apply_threshold_changes();

#if CY_CAPSENSE_BIST_EN
            /* 使用BIST测量传感器电极的自电容 */
            _measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */

            /* 读取一次0x0寄存器后熄灭LED */
            static bool first_read_done = false;
            if (!first_read_done)
            {
                /* 模拟读取0x0寄存器 */
                i2c_handle_register_read(REG_SCAN_RATE);
                led_off();
                first_read_done = true;
            }

            /* 开始下一次扫描 */
            capsense_start_scan();
        }
    }
}








/*******************************************************************************
* Function Name: _update_scan_rate
********************************************************************************
* Summary: 更新扫描速率计算
*******************************************************************************/
static void _update_scan_rate(void)
{
    static uint32_t scan_count = 0;
    static uint64_t last_rate_update_time = 0;
    
    scan_count++;
    
    uint64_t current_time = _get_system_time_us();
    uint64_t time_diff = current_time - last_rate_update_time;
    
    /* 每秒更新一次扫描速率 */
    if (time_diff >= 1000000) /* 1秒 = 1,000,000微秒 */
    {
        g_scan_rate_per_second = (uint16_t)((scan_count * 1000000ULL) / time_diff);
        scan_count = 0;
        last_rate_update_time = current_time;
    }
}

#if CY_CAPSENSE_BIST_EN
/* 测量传感器电极自电容 */
static void _measure_sensor_cp(void)
{
    /* 测量传感器电极自电容 */
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_CAP0_WDGT_ID,
                                                  CY_CAPSENSE_CAP0_SNS0_ID,
                                             &sensor_cp, &cy_capsense_context);
}
#endif /* CY_CAPSENSE_BIST_EN */

/* 获取系统时间（微秒） - 使用SysTick定时器 */
static uint64_t _get_system_time_us(void)
{
    uint32_t systick_val;
    uint32_t overflow_count;
    
    /* 禁用中断以确保原子读取 */
    __disable_irq();
    
    systick_val = SysTick->VAL;
    overflow_count = _systick_overflow_count;
    
    /* 检查是否在读取期间发生溢出 */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) && (systick_val > (SysTick->LOAD >> 1)))
    {
        overflow_count++;
    }
    
    __enable_irq();
    
    /* 计算总时间（微秒） */
    /* SysTick向下计数，所以我们需要从LOAD值中减去当前值 */
    uint32_t ticks_in_current_ms = SysTick->LOAD - systick_val;
    uint64_t total_us = (uint64_t)overflow_count * 1000 + (ticks_in_current_ms * 1000) / SysTick->LOAD;
    
    return total_us;
}

/*******************************************************************************
* Function Name: SysTick_Handler
********************************************************************************
* Summary: SysTick中断处理程序，用于计算溢出次数以进行时间测量
*******************************************************************************/
void SysTick_Handler(void)
{
    _systick_overflow_count++;
}
