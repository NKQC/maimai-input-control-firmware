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
#define REG_CONTROL                 (0x2)    // 原REG_LED控制重命名
// 触摸电容设置寄存器组（每Widget 1个）：按 0.01 pF 步进写增量（基址+范围：0x03..0x0E）
#define REG_TOUCH_CAP_SETTING_BASE  (0x03)
// 总触摸电容只读寄存器组（每Widget 1个）：Cp基数 + 设置增量，单位 0.01 pF
#define REG_TOTAL_TOUCH_CAP_BASE    (0x0F)   // CAP0总触摸电容地址起点：0x0F + i

// 噪声只读寄存器组（每Widget 1个）：正噪声阈值与负噪声阈值（顺位后移）
#define REG_NOISE_TH_BASE           (0x1B)   // CAP0噪声阈值地址起点：0x1B + i
#define REG_NNOISE_TH_BASE          (0x27)   // CAP0负噪声阈值地址起点：0x27 + i

// 控制寄存器位域定义：
// bit0: reset_req，置1发起软件复位
// bit1: led_on，置1 LED点亮
// bit2: calibrate_req，置1发起校准请求（异步）
// bit3: calibration_done，只读；0表示校准中，1表示校准完成（启动后完成也置1）
// bit4: absolute_mode，置1时触摸电容设置为绝对值（单位步进）；0为相对（ZERO_BIAS有符号偏移）
// bit5: led_feedback_en，置1自动按触摸状态指示LED（默认1）；0关闭自动提示
typedef union {
    uint16_t raw;
    struct {
        uint16_t reset_req        : 1;
        uint16_t led_on           : 1;
        uint16_t calibrate_req    : 1;
        uint16_t calibration_done : 1;
        uint16_t absolute_mode    : 1;
        uint16_t led_feedback_en  : 1;
        uint16_t reserved         : 10;
    } bits;
} i2c_control_reg_t;

void i2c_init(uint8_t slave_address);
void i2c_set_scan_rate(uint16_t rate);
uint16_t i2c_handle_register_read(uint8_t reg_addr);
void i2c_handle_register_write(uint8_t reg_addr, uint16_t value);

// 读取LED触摸提示开关状态（CONTROL.bit5）
bool i2c_led_feedback_enabled(void);

#endif