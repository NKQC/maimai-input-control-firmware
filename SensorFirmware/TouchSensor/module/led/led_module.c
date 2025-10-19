#include "led_module.h"

// 初始化LED引脚
void led_init(void)
{
    Cy_GPIO_Pin_FastInit(LED_PIN_PORT, LED_PIN_NUM, CY_GPIO_DM_STRONG_IN_OFF, 0, HSIOM_SEL_GPIO);
}

// 点亮LED
void led_on(void)
{
    Cy_GPIO_Set(LED_PIN_PORT, LED_PIN_NUM);
}

// 熄灭LED
void led_off(void)
{
    Cy_GPIO_Clr(LED_PIN_PORT, LED_PIN_NUM);
}

// 设置LED状态
void led_set_state(bool state)
{
    if (state)
    {
        led_on();
    }
    else
    {
        led_off();
    }
}