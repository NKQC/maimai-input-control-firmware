#ifndef LED_MODULE_H
#define LED_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"

#define LED_PIN_PORT              (GPIO_PRT4)
#define LED_PIN_NUM               (0u)

void led_init(void);
void led_on(void);
void led_off(void);
void led_set_state(bool state);

#endif /* LED_MODULE_H */