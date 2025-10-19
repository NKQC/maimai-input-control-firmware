#ifndef I2C_MODULE_H
#define I2C_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"

#define I2C_SLAVE_INTR_PRIORITY     (3u)
#define I2C_SLAVE_BASE_ADDR         (0x08)
#define I2C_SLAVE_BUFFER_SIZE       (16u)
#define CY_ASSERT_FAILED            (0u)

#define ADDR_PIN_P3_3_PORT          GPIO_PRT3
#define ADDR_PIN_P3_3_NUM           3u
#define ADDR_PIN_P3_2_PORT          GPIO_PRT3
#define ADDR_PIN_P3_2_NUM           2u

#define REG_SCAN_RATE               (0x0)
#define REG_TOUCH_STATUS            (0x1)
#define REG_LED_CONTROL             (0x2)
#define REG_CAP0_THRESHOLD          (0x3)
#define REG_CAP1_THRESHOLD          (0x4)
#define REG_CAP2_THRESHOLD          (0x5)
#define REG_CAP3_THRESHOLD          (0x6)
#define REG_CAP4_THRESHOLD          (0x7)
#define REG_CAP5_THRESHOLD          (0x8)
#define REG_CAP6_THRESHOLD          (0x9)
#define REG_CAP7_THRESHOLD          (0xA)
#define REG_CAP8_THRESHOLD          (0xB)
#define REG_CAP9_THRESHOLD          (0xC)
#define REG_CAPA_THRESHOLD          (0xD)
#define REG_CAPB_THRESHOLD          (0xE)

void i2c_init(uint8_t slave_address);
void i2c_set_scan_rate(uint16_t rate);
uint16_t i2c_handle_register_read(uint8_t reg_addr);
void i2c_handle_register_write(uint8_t reg_addr, uint16_t value);

#endif