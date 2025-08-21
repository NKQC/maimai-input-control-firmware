#include <FreeRTOS.h>
#include "core.h"
#include "watchdog/watchdog.h"
#include "usb/usb.h"
#include "storage/storage.h"
#include "core/uart/uart.h"
#include "planner/planner.h"
#include <driver/dedic_gpio.h>
#include "sensor_check/sensor.h"
#include "../display/display.h"

/**
 * 警告: Arduino loop() 运行在Core 1
 * 计划用于USB OTG 处理和发送
 *
 * @param SensorCore
 * @param sencor_threadname
 * @param sensor_memory
 * @param sensor_loop
 * @param sensor_priority
 * @param sensor_pvCreatedTask
 * @param xCoreID
 * @return BaseType_t
 */

#define TASK_BIT_1 (0x01 << 0)
#define TASK_BIT_2 (0x01 << 1)
#define TASK_BIT_ALL (TASK_BIT_1 | TASK_BIT_2)

#define TOUCH_THRESHOLD_RESET_COUNT 3000000
#define CHECK_STABLE (core.touch_delaytime)

// sensor_loop
TaskHandle_t v_sensor_loop;

// uart_loop
TaskHandle_t v_uart_loop;

uint32_t update_speed = 0;

// STATE GPIO
uint32_t IRAM_ATTR sensor_readGPIO = 0b0;

uint32_t IRAM_ATTR sensor_readTouch = 0b0;

// 延迟队列
uint32_t IRAM_ATTR sensor_TouchQueue[TOUCHQUEUE_SIZE][2] = {0}; // 0->Val 1->Time
uint32_t IRAM_ATTR TouchQueue_point = 0;                        // sensor_TouchQueue point

uint32_t IRAM_ATTR sensor_current = 0b0;

uint32_t IRAM_ATTR sensor_gpiocurrent = 0b0;

uint32_t IRAM_ATTR sensor_old = 0b0;

uint32_t IRAM_ATTR report_clk = 0b0;

uint32_t IRAM_ATTR old_report_clk = 0b0;

millis_t IRAM_ATTR hid_timer = millis(); // HID同步时钟

millis_t IRAM_ATTR triggle_hold_time = 0; // 风险按键保持时间

uint32_t IRAM_ATTR command_button = 0b0;

uint32_t IRAM_ATTR old_command_button = 0b0;

uint8_t multi, _multi, multi_old;

int32_t Core::mode = MODE_SERIAL;

millis_t Core::touch_delaytime = 0;

// http://www.tongxinmao.com/Article/Detail/id/520

const uint8_t keyboard_map[BUTTON_NUM] = {
    // 键盘 1 - BUTTON_NUM
    // For Maimai Fes+ 1.35+
    0X1a, // W 1
    0X08, // E 2
    0X07, // D 3
    0X06, // C 4
    0X1b, // X 5
    0X1d, // Z 6
    0X04, // A 7
    0X14, // Q 8
    0X0A, // 9 9
    0X20, // 3 10
    0X28, // Enter 11
    0X2c  // Space 12
};

// 队列处理函数
#ifdef TOUCHREPORT_DELAY
static inline void TouchQueue_add(uint32_t val, millis_t time)
{
    if (TouchQueue_point > (TOUCHQUEUE_SIZE - 2))
        TouchQueue_point = 0;
    else
        TouchQueue_point++;
    sensor_TouchQueue[TouchQueue_point][0] = val;
    sensor_TouchQueue[TouchQueue_point][1] = (uint32_t)time;
}
static inline uint32_t TouchQueue_Timeread(millis_t time)
{
    if (time > millis())
        return 0xFFFFFFFF;
    uint32_t point = TouchQueue_point;
    uint32_t nearby = 0;
    for (uint32_t i = 0; i < TOUCHQUEUE_SIZE; i++)
    {
        if (point)
            point--;
        else
            point = TOUCHQUEUE_SIZE - 1;
        if (!nearby && (sensor_TouchQueue[point][1] == time + 1 || sensor_TouchQueue[point][1] == time - 1))
            nearby = sensor_TouchQueue[point][0];
        if (sensor_TouchQueue[point][1] == time)
            return sensor_TouchQueue[point][0];
    }
    return nearby;
}
#endif

// 硬件自检
static inline void hardware_self_check()
{
    display.draw_strings("Hardware CHECK...", 1, 2);
    display.draw_uint32(millis(), 4, 4);
    uint8_t init_scan = sensor.i2c_scan();
    sensor.test_led();
    if (init_scan != 2)
    {
        uart.send("ERROR:TOUCH READ ERROR");
        uart.send(init_scan);
        display.draw_strings("TOUCH ERROR", 1, 1);
        display.draw_strings("RESET->", 20, 2);
        display.update();
        while (1)
        {
        }
    }
}

// "1-A1", "2-A2", "3-A3", "4-A4", "5-A5", "6-A6", "7-A7", "8-A8",
// "9-B1", "10-B2", "11-B3", "12-B4", "13-B5", "14-B6", "15-B7", "16-B8",
// "17-C1", "18-C2",
// "19-D1", "20-D2", "21-D3", "22-D4", "23-D5", "24-D6", "25-D7", "26-D8",
// "27-E1", "28-E2", "29-E3", "30-E4", "31-E5", "32-E6", "33-E7", "34-E8"

// 分区圆形映射表 从1起+-
static const uint8_t ring_ad[17] = {
    19, 1, 20, 2, 21, 3, 22, 4, 23, 5, 24, 6, 25, 7, 26, 8, 19};

static const uint8_t ring_be[17] = {
    27, 9, 28, 10, 29, 11, 30, 12, 31, 13, 32, 14, 33, 15, 34, 16, 27};

// 获得一个触摸区域临近的两个区域 (只考虑环内间 不考虑内外环之间)
inline void check_near_region(uint8_t point, uint32_t *return_point)
{
    if ((point > 0 && point < 9) || (point > 18 && point < 27))
        for (uint8_t i = 1; i < 17; i++)
            if (ring_ad[i] == point)
            {
                return_point[0] = ring_ad[i - 1];
                return_point[1] = ring_ad[i + 1];
                return;
            }
    if ((point > 8 && point < 17) || (point > 26 && point < 35))
        for (uint8_t i = 1; i < 17; i++)
            if (ring_be[i] == point)
            {
                return_point[0] = ring_be[i - 1];
                return_point[1] = ring_be[i + 1];
                return;
            }
    return_point[0] = return_point[1] = 0;
    return;
}

// 通过一个触摸区域获得在相同触摸点绑定的另一个触摸区域编号 空则返回0
inline uint32_t get_another_bind_pad(uint32_t bind)
{
    for (uint8_t i = 0; i < TOUCH_NUM; i++)
    {
        if (sensor.mai_map[i][0] == bind)
            return sensor.mai_map[i][1];
        if (sensor.mai_map[i][1] == bind)
            return sensor.mai_map[i][0];
    }
    return 0;
}
// 返回输入中指定位的值(检查区域触发) 如果是空绑区则判断为触发
inline uint8_t _check_triggle(uint32_t *input, uint8_t point)
{
    if (!point)
        return 1;
    uint8_t triggle = 0;
    if (point < 26)
        triggle = (input[0] >> (point - 1)) & 1;
    else
        triggle = (input[1] >> (point - 26)) & 1;
    return triggle;
}
// 置零指定位 （删除指定位的触发情况）
inline void del_triggle(uint32_t *edit, uint8_t point)
{
    if (!point)
        return;
#ifdef NO_A_REGION
    if (point < 9) // 禁止对A区和0修改
        return;
#endif
#ifdef NO_BE_REGION
    if ((point > 8 && point < 17) || (point > 26 && point < 35)) // 禁止对BE区修改
        return;
#endif
    if (point <= 25)
        edit[0] &= ~(1 << (point - 1));
    else
        edit[1] &= ~(1 << (point - 26));
}

// 用于连区物理设计的触发区域自修正
inline void auto_region_detect(uint32_t *input)
{
    uint32_t _input[2] = {input[0], input[1]};
    uint32_t _near[2] = {0};
    uint32_t __near[2] = {0};
    uint32_t in_once_pad = 0;
    for (uint8_t i = 1; i <= 34; i++)
    {
        if (_check_triggle(_input, i))
        {
            check_near_region(i, _near);
            in_once_pad = get_another_bind_pad(i);
            if (_near[0] == in_once_pad)
            {
                if (_check_triggle(_input, _near[1]))
                {
                    // 如果触摸在交界处 还应判断远端与其下一个触摸区域有无触摸 以此来确定远端是否处于和下一个区域一起的连续触摸中
                    check_near_region(in_once_pad, __near);
                    if (!_check_triggle(_input, __near[0] == i ? __near[1] : __near[0]))
                        // 确保远端触摸位点临近处无触摸 即可删除
                        del_triggle(_input, _near[0]);
                }
            }
            else if (_near[1] == in_once_pad)
                if (_check_triggle(_input, _near[0]))
                {
                    check_near_region(in_once_pad, __near);
                    if (!_check_triggle(_input, __near[0] == i ? __near[1] : __near[0]))
                        del_triggle(_input, _near[1]);
                }
        }
    }
    input[0] = _input[0];
    input[1] = _input[1];
}

/**
 * 0 -> not init
 * 1 -> machine init
 * 2 -> init core
 * 3 -> run
 * 0xAA - > SETTINGS
 *
 * 0x80 -> linked for uart  40 Touch
 * 0xFF -> error
 */
uint8_t Core::status = 0x00; // 设备状态
uint8_t Core::serial_ok = 1; // 默认启动

// 加权读取阈值
inline bool _touch_read(uint32_t _get, uint8_t target, uint8_t id)
{
    return !((_get & (0b11 << (target * 2))) > (sensor.touch_pressure_read((id ? 12 : 0) + target)));
}

// 触摸汇总
inline uint32_t send_touch_info(uint32_t *get)
{
    uint32_t call = 0;
    uint8_t _cc = 0;
    while (_cc < 12)
    {
        call |= _touch_read(get[0], _cc, 0) << _cc;
        _cc++;
    }
    for (uint8_t aa = 0; aa < 12; aa++)
    {
        call |= _touch_read(get[1], aa, 1) << _cc;
        _cc++;
    }
    return (call & 0xFFFFFF);
}

// 显示正在被触发的触摸位点
static inline void touch_triggle_show()
{
    bool has_down = false;
    static millis_t last_triggle_time = millis();
    for (uint8_t a = 0; a < TOUCH_NUM; a++)
    {
        if (sensor.check_down(sensor_readTouch, a) && !sensor.check_down(sensor_old, a))
        {
            if (!has_down)
            {
                has_down = true;
                uart.send("TRIGGLE: ", 0);
            }
            uart.send("+", 0);
            uart.send(a + 1, 0);
            uart.send(" ", 0);
        }
        else if (!sensor.check_down(sensor_readTouch, a) && sensor.check_down(sensor_old, a))
        {
            if (!has_down)
            {
                has_down = true;
                uart.send("TRIGGLE: ", 0);
            }
            uart.send("-", 0);
            uart.send(a + 1, 0);
            uart.send(" ", 0);
        }
    }
    if (has_down)
    {
        uart.send(millis() - last_triggle_time, 0);
        last_triggle_time = millis();
        uart.send(" |");
    }
    sensor_old = sensor_readTouch;
}

// 显示正在被触发的按键
static inline void key_triggle_show()
{
    bool has_down = false;
    for (uint8_t a = 0; a < BUTTON_NUM; a++)
    {
        if (sensor.check_down(sensor_readGPIO, a))
        {
            if (!has_down)
            {
                has_down = true;
                uart.send("KEY: ", 0);
            }
            uart.send(a + 1, 0);
            uart.send(" ", 0);
        }
    }
    if (has_down)
        uart.send(" |");
}

/**
 * CORE 0 LOOP
 *
 * @param cache
 */
void sensor_loop(void *cache)
{
    while (core.status != 0x1)
    {
        dog.feed();
        vTaskDelay(1);
    }
    uart.send("INFO:INIT SENSOR");
    sensor.tsm12mc_reset();
    sensor.start_sample();
    hardware_self_check();
    uart.send("INFO:SENSOR OK");

    core.status = 0x2;
    register uint32_t _sensor_readGPIO = 0;
    register millis_t test_time = millis();
    register uint32_t cps = 0;
    register uint32_t s = 0;
    register uint32_t touch_read[2] = {0};
    while (1)
    {
        if (millis() > test_time)
        {
            test_time = millis() + 1000UL;
            update_speed = cps;
            old_report_clk = report_clk;
            report_clk = 0; // 用于检测反馈速度
            cps = 0;
        }

        sensor.tsm12mc_read(0, &touch_read[0]);
        sensor.tsm12mc_read(1, &touch_read[1]);
        sensor_readTouch = send_touch_info(touch_read);
#ifdef TOUCHREPORT_DELAY
        TouchQueue_add(sensor_readTouch, millis());
#endif
        _sensor_readGPIO = 0;
        for (s = 0; s < GPIO_SENSOR_NUM; s++)
            _sensor_readGPIO |= (planner.read_GPIO(planner.sensor_gpio[s]) << s);
        sensor_readGPIO = _sensor_readGPIO;
        // COMMAND BUTTON
        {
            uint32_t command = 0;
            command |= !planner.read_GPIO(planner.sensor_gpio[MODE_SETTINGS_KEY]);
            if (command != old_command_button) {
                command_button |= command;
            }        
            old_command_button = command;
        }
        cps++;
        dog.feed();
    }
}

inline void settings_mode();
inline void main_loop();

/**
 * CORE 1 LOOP
 *
 * @param cache
 */
void uart_loop(void *cache)
{
    while (core.status != 0x2 && core.status != 0x3)
    {
        dog.feed();
        vTaskDelay(1);
    }
    // sensor.test_gpio(); // DEBUG
    uart.send("INFO:INIT SCREEN");
    display.begin();
    uart.send("INFO:SCREEN OK");
    /**
     * @huhuzhu
     * INFO
     */
    uart.send("--------------------");
    uart.send("SERIAL/TOUCH PAD/KEYBOARD");
    uart.send(FIRMWARE_VERSION);
    uart.send("@huhuzhu");
    uart.send("--------------------");
    if (core.mode == MODE_SERIAL)
    {
        uart.send("SERIAL MODE");
        display.update(1);
        display.draw_strings("SERIAL_MODE", 1, 1);
        display.draw_strings("@huhuzhu", 1, 2);
        display.update();
    }

    else if (core.mode == MODE_TOUCH)
    {
        uart.send("HID TOUCH MODE");
        display.update(1);
        display.draw_strings("HIDTOUCH_MODE", 1, 1);
        display.draw_strings("@huhuzhu", 1, 2);
        display.update();
    }
    sensor.settings_led(0);
    vTaskDelay(50);

    { // 启动准备
        display.update(1);
        display.draw_strings("START...", 8, 1);
        display.update();
    }
init_settings:                 // 给设置预留的跳转
    while (core.status & 0xA0) // SETTINGS MODE
        settings_mode();

    planner.set_GPIO(planner.sensor_gpio[TEST_LED], 0);
    vTaskDelay(500);
    core.status = 0x3; // 进入运行状态
    {                  // 启动准备
        display.update(1);
        display.draw_strings("WORK...", 1, 1);
        if (core.mode == MODE_SERIAL)
            display.draw_strings("SERIAL_MODE", 1, 2);
        else
            display.draw_strings("HIDTOUCH_MODE", 1, 2);
        if (!usb.HID_ready())
        {
            display.draw_strings("HID ERROR", 1, 3);
            usb.HID_end();
        }
        else
        {
            display.draw_strings(FIRMWARE_VERSION, 1, 3);
#ifdef TOUCHREPORT_DELAY
            display.draw_strings("DELAY:", 1, 4);
            display.draw_uint32(core.touch_delaytime, 36, 4, 4);
#endif
        }
        display.update();
    }
    uart.send("INFO: RUNNING...");
    core.serial_ok = 1;

    // 故障处理
    while (core.status > 0xF0)
    {
        display.update(1);
        display.draw_strings("ERROR:", 8, 1);
        if ((core.mode - 0xF0) & 0b1)
            display.draw_strings("TOUCH1 ERROR", 1, 2);
        if ((core.mode - 0xF0) & 0b10)
            display.draw_strings("TOUCH2 ERROR", 1, 2);
        display.draw_strings("RESET->", 1, 3);
        display.update();
        while (1)
        {
            // NOOP
        }
    }

    // 主线程 工作循环
    while (1)
    {
        main_loop();
        // 设置模式 跳转出工作循环
        if (command_button & 0b1 || core.status == 0xA1)
        {
            command_button = 0;
            core.status = 0xA1;
            goto init_settings;
        }
    }
}

// 总初始化
void Core::init()
{
    nvs.nvs_init();
    nvs.nvs_setup_read();
    planner.init_gpio();
    uart.init(true);
    usb.init();
    sensor.sensor_bus_init();
    core.mode = planner.read_GPIO(planner.sensor_gpio[MODE_SELECT_KEY]);
    dog.init();
    core.vtask_init();
    core.status = 0x1;
}

void Core::core_main()
{
    millis_t touch_threshold_reset = millis();
    uart.send("INFO:START");
    while (1)
    {
        millis_t time = millis();
        // 自动重置触摸阈值锁定
        if (millis() > touch_threshold_reset)
        {
            {
                sensor.touch_refence_update(0, true);
            }
            touch_threshold_reset = time + TOUCH_THRESHOLD_RESET_COUNT;
        }
        vTaskDelay(1);
    }
}

void Core::vtask_init()
{
    // Sensor Thread
    xTaskCreatePinnedToCore(
        sensor_loop,    // 启动函数
        "SensorT",      // 进程名称
        65535,          // 执行分配内存 (bit)
        NULL,           // 函数输入参数指针
        1,              // 优先级
        &v_sensor_loop, // 句柄
        0               // 执行CPU
    );

    // Uart Thread
    xTaskCreatePinnedToCore(
        uart_loop,    // 启动函数
        "UartT",      // 进程名称
        65536,        // 执行分配内存 (bit)
        NULL,         // 函数输入参数指针
        1,            // 优先级
        &v_uart_loop, // 句柄
        1             // 执行CPU
    );
}

inline void check_touch(millis_t time) {
    // 自动锁定触摸阈值
    uint32_t sample_check = TouchQueue_Timeread(time);
    if (sample_check != 0xFFFFFFFF && sample_check)
    {
        uint32_t old_check = sample_check;
        uint8_t avaliable = 1;
        for (uint16_t scount = 0; scount < CHECK_STABLE; scount++)
        {
            sample_check = TouchQueue_Timeread(time - scount);
            if (sample_check != old_check) {
                avaliable = 0;
                break;
            }
            old_check == sample_check;
        }
        if (avaliable)
            sensor.touch_refence_update(~sample_check);       
    }
}

/**
 * 设置模式循环
 *
 */
inline void settings_mode()
{
    static uint8_t touch_point_current = 0;
    static uint8_t extra_point = 0;
    static uint8_t once = 0xFF;
    static uint8_t command = 0;
    static uint8_t led_current = 0;
    static uint8_t tap = 0, old_tap = 0;
    static millis_t led_timer = millis();
    static uint8_t touch_spl_settings = 0;
    static uint8_t mode = 0;     // 0->普通调整模式 1->引导式点位录入
    static uint8_t auto_num = 0; // 用于引导式点位绑定

    if (core.status == 0xA1)
    {
        core.status = 0xA0;
        once = 0xFF;
    }
    if (once == 0xFF)
    {
        core.serial_ok = 0;
        sensor.settings_led(1);
        once = 1;
        uart.send(" ");
        uart.send("INFO: SETTING MODE");
        {
            display.update(1);
            if (core.mode == MODE_TOUCH)
                display.draw_strings("HIDTOUCH_ST", 1, 1);
            else if (core.mode == MODE_SERIAL)
                display.draw_strings("MAISERIAL_ST", 1, 1);
            else
                display.draw_strings("SETTINGS?", 8, 1);
            display.draw_strings("LOADING...", 5, 2);
            display.update();
        }
        mode = 0;
        auto_num = 0;
        display.update(1);
    }
    else if (once == 0xF0)
    {
        display.update(1);
        extra_point = 2;
        vTaskDelay(1000);
    }
    if (!touch_point_current)
        touch_point_current = 1;

    // 退出设置
    if (command_button & 0b1)
    {
        command_button = 0;
        nvs.nvs_setup_read();
        core.status = 0x2;
        core.serial_ok = 1;
        return;
    }
    {
        command = 0;
        // 获取指令
        for (uint8_t id = 1; id < 14; id++)
        {
            if (sensor.check_down(sensor_readGPIO, id - 1))
            {
                if ((core.mode == MODE_TOUCH) && (id > 1 && id < 6))
                {
                    command = id;
                    break;
                }
                if ((core.mode == MODE_SERIAL) && (id > 1 && id < 4))
                {
                    command = id;
                    vTaskDelay(25);
                    break;
                }
                tap = id;
                break;
            }
            else
            {
                tap = 0;
            }
        }
        if (tap != old_tap)
            command = tap;
        old_tap = tap;

        /**
         * 菜单选择
         *
         */
        switch (mode & 0b1111)
        {
        case 0: // 普通设置模式
        {
            if (!touch_spl_settings) // 正常设置模式
            {
                switch (core.mode)
                {
                case MODE_TOUCH:
                {
                    switch (command) // 功能选择
                    {
                    case 1:
                    { // 开始调整位点
                        if (touch_point_current > 12)
                            extra_point = extra_point ? 0 : 1;
                        break;
                    }
                    case 2:
                    { // 左移动
                        if (!extra_point)
                        {
                            if (sensor.touch_bar[touch_point_current - 1].x > 25)
                                sensor.touch_bar[touch_point_current - 1].x -= 25;
                        }
                        else
                        {
                            if (sensor.touch_bar[touch_point_current + 7].x > 25)
                                sensor.touch_bar[touch_point_current + 7].x -= 25;
                        }
                        break;
                    }
                    case 3:
                    { // 右移动
                        if (!extra_point)
                        {
                            if (sensor.touch_bar[touch_point_current - 1].x < 32743)
                                sensor.touch_bar[touch_point_current - 1].x += 25;
                        }
                        else
                        {
                            if (sensor.touch_bar[touch_point_current + 7].x < 32743)
                                sensor.touch_bar[touch_point_current + 7].x += 25;
                        }
                        break;
                    }
                    case 4:
                    { // 上移动
                        if (!extra_point)
                        {
                            if (sensor.touch_bar[touch_point_current - 1].y > 25)
                                sensor.touch_bar[touch_point_current - 1].y -= 25;
                        }
                        else
                        {
                            if (sensor.touch_bar[touch_point_current + 7].y > 25)
                                sensor.touch_bar[touch_point_current + 7].y -= 25;
                        }
                        break;
                    }
                    case 5:
                    { // 下移动
                        if (!extra_point)
                        {
                            if (sensor.touch_bar[touch_point_current - 1].y < 32743)
                                sensor.touch_bar[touch_point_current - 1].y += 25;
                        }
                        else
                        {
                            if (sensor.touch_bar[touch_point_current + 7].y < 32743)
                                sensor.touch_bar[touch_point_current + 7].y += 25;
                        }
                        break;
                    }
                    case 6:
                    { // 上一个位点
                        if (touch_point_current > 1)
                        {
                            touch_point_current--;
                            once = 1;
                            extra_point = 0;
                        }
                        break;
                    }
                    case 7:
                    { // 下一个位点
                        if (touch_point_current < TOUCH_NUM)
                        {
                            touch_point_current++;
                            once = 1;
                            extra_point = 0;
                        }
                        break;
                    }
                    case 8:
                    { // 保存并返回到运行模式
                        core.status = 0x2;
                        nvs.nvs_storage();
                        break;
                    }
                    case 10:
                    { // 单点感度调整
                        touch_spl_settings = touch_point_current;
                        once = 0xF0;
                        break;
                    }
                    }
                    break;
                }
                case MODE_SERIAL:
                {
                    switch (command)
                    {
                    case 1:
                    { // 开始调整位点
                        extra_point = extra_point ? 0 : 1;
                        break;
                    }
                    case 2:
                    { // 调整目标映射区

                        if (!extra_point)
                        {
                            if (sensor.mai_map[touch_point_current - 1][0] > 0)
                                sensor.mai_map[touch_point_current - 1][0] -= 1;
                        }
                        else
                        {
                            if (sensor.mai_map[touch_point_current - 1][1] > 0)
                                sensor.mai_map[touch_point_current - 1][1] -= 1;
                        }
                        break;
                    }
                    case 3:
                    {
                        if (!extra_point)
                        {
                            if (sensor.mai_map[touch_point_current - 1][0] < 35)
                                sensor.mai_map[touch_point_current - 1][0] += 1;
                        }
                        else
                        {
                            if (sensor.mai_map[touch_point_current - 1][1] < 35)
                                sensor.mai_map[touch_point_current - 1][1] += 1;
                        }
                        break;
                    }
                    case 6:
                    { // 上一个位点
                        if (touch_point_current > 1)
                        {
                            touch_point_current--;
                            once = 1;
                            extra_point = 0;
                        }
                        break;
                    }
                    case 7:
                    { // 下一个位点
                        if (touch_point_current < TOUCH_NUM)
                        {
                            touch_point_current++;
                            once = 1;
                            extra_point = 0;
                        }
                        break;
                    }
                    case 8:
                    { // 保存并返回到运行模式
                        core.status = 0x2;
                        nvs.nvs_storage();
                        break;
                    }
                    case 9:
                    { // 引导式点位录入
                        mode = 1;
                        once = 0xF0;
                        display.update(1);
                        break;
                    }
                    case 10:
                    { // 单点感度调整
                        touch_spl_settings = touch_point_current;
                        once = 0xF0;
                        break;
                    }
                    }
                }
                }
            }
            else // 感度设置模式
            {
                switch (command) // 功能选择
                {
                case 2:
                {
                    if (touch_spl_settings > 1)
                        touch_spl_settings--;
                    break;
                }
                case 3:
                {
                    if (touch_spl_settings < TOUCH_NUM)
                        touch_spl_settings++;
                    break;
                }
                case 4:
                {
                    sensor.touch_spl_set(touch_spl_settings, sensor.touch_spl_read(touch_spl_settings) - 1);
                    break;
                }
                case 5:
                {
                    sensor.touch_spl_set(touch_spl_settings, sensor.touch_spl_read(touch_spl_settings) + 1);
                    break;
                }
                case 8:
                { // 保存并返回到运行模式
                    core.status = 0x2;
                    extra_point = 0;
                    nvs.nvs_storage();
                    break;
                }
                case 9:
                { // 引导式点位录入
                    mode = 1;
                    auto_num = 0;
                    once = 0xF0;
                    display.update(1);
                    break;
                }
                case 10:
                { // 退出感度调整模式
                    touch_spl_settings = 0;
                    extra_point = 0;
                    break;
                }
                }
            }
            break;
        }
        case 1: // 引导式点位绑定
        {
            switch (command) // 功能选择
            {
            case 1:
            {
                mode = 0b10000001;
                display.update(1);
                once = 0xF0;
                auto_num = 1;
                touch_point_current = 1;
                extra_point = 0;
                sensor.clean_mai_map();
                break;
            }
            case 6:
            { // 上一个位点
                if (touch_point_current > 1)
                {
                    touch_point_current--;
                    once = 1;
                    extra_point = 0;
                }
                break;
            }
            case 7:
            { // 下一个位点
                if (touch_point_current < TOUCH_NUM)
                {
                    touch_point_current++;
                    once = 1;
                    extra_point = 0;
                }
                break;
            }
            case 8:
            { // 保存并返回到运行模式
                core.status = 0x2;
                extra_point = 0;
                mode = 0;
                nvs.nvs_storage();
                break;
            }
            case 9:
            { // 引导式点位录入返回
                mode = 0;
                once = 0xF0;
                display.update(1);
                break;
            }
            }
            break;
        }
        }
        command = 0;
        // 刷新应用层
        {
            if (touch_point_current > 0) // 动作处理部分
            {
                if (once)
                {
                    once = 0;
                }
                switch (mode & 0b1111)
                {
                case 0: // 普通设置模式
                {
                    if (!touch_spl_settings)
                        switch (core.mode)
                        {
                        case MODE_TOUCH:
                        {
                            {
                                display.draw_strings("C-", 1, 0);
                                display.draw_uint32(touch_point_current, 20, 0, 2);
                                if (extra_point)
                                    display.draw_strings("E+", 32, 0);
                                else
                                    display.draw_strings("  ", 32, 0);
                                display.draw_uint32(update_speed, 44, 0, 4);

                                if (extra_point)
                                    display.draw_uint32(sensor.touch_bar[touch_point_current + 7].x, 0, 1, 5);
                                else
                                    display.draw_uint32(sensor.touch_bar[touch_point_current - 1].x, 0, 1, 5);
                                display.draw_strings("*", 34, 1);
                                if (extra_point)
                                    display.draw_uint32(sensor.touch_bar[touch_point_current + 7].y, 40, 1, 5);
                                else
                                    display.draw_uint32(sensor.touch_bar[touch_point_current - 1].y, 40, 1, 5);

                                display.draw_strings("1E+ X2-3+", 1, 2);
                                display.draw_strings("Y4-5+ P6-7+", 1, 3);
                                display.draw_strings("R:", 1, 4);
                                display.draw_uint32(sensor_readTouch, 20, 4, 8);
                                display.update();
                            }
                            break;
                        }
                        case MODE_SERIAL:
                        {
                            {
                                display.draw_strings("C-", 1, 0);
                                display.draw_uint32(touch_point_current, 20, 0, 2);
                                if (extra_point)
                                    display.draw_strings("E+", 32, 0);
                                else
                                    display.draw_strings("  ", 32, 0);
                                display.draw_uint32(update_speed, 44, 0, 4);
                                display.draw_strings("T-", 1, 1);
                                display.draw_uint32(sensor.mai_map[touch_point_current - 1][extra_point], 20, 1, 2);
                                display.draw_strings("1E+ 2-3+", 1, 2);
                                display.draw_strings("P6-7+ 8S 10T", 1, 3);
                                display.draw_strings("R:", 1, 4);
                                display.draw_uint32(sensor_readTouch, 20, 4, 8);
                                display.update();
                            }
                            break;
                        }
                        break;
                        }
                    else
                    {
                        display.draw_strings("P>", 1, 0);
                        display.draw_uint32(touch_spl_settings, 20, 0, 2);
                        display.draw_strings("T>", 1, 1);
                        display.draw_uint32(sensor.touch_spl_read(touch_spl_settings), 20, 1, 4);
                        display.draw_strings("2- 3+ 8S", 1, 2);
                        display.draw_strings("T4- 5+ 10T", 1, 3);
                        display.draw_strings("R:", 1, 4);
                        display.draw_uint32(sensor_readTouch, 20, 4, 8);
                        display.update();
                    }

                    // 处理EXTRA点位灯光
                    if (extra_point == 1 && (led_timer < millis()))
                    {
                        led_current = led_current ? 0 : 1;
                        planner.set_GPIO(planner.sensor_gpio[TEST_LED], led_current);
                        led_timer = millis() + 250U;
                    }
                    else if (!extra_point)
                    {
                        planner.set_GPIO(planner.sensor_gpio[TEST_LED], 0);
                    }
                    else if (extra_point == 2)
                    {
                        planner.set_GPIO(planner.sensor_gpio[TEST_LED], 1);
                    }
                    break;
                }
                case 1:
                { // 仅支持SERIAL模式
                    if (core.mode != MODE_SERIAL)
                        break;
                    static uint8_t cmd = 0;
                    uint8_t multi = planner.multi_touch(sensor_readTouch);
                    {
                        if (!(mode & 0b10000000))
                            cmd = 10;
                        else
                        {
                            static uint8_t multi_old = 0;
                            if (multi_old != multi)
                            {
                                if (multi == 1 && (auto_num < 35))
                                {
                                    for (uint8_t ft = 0; ft < TOUCH_NUM; ft++)
                                    {
                                        if (sensor.check_down(sensor_readTouch, ft))
                                            touch_point_current = (ft + 1);
                                    }
                                    if (sensor.mai_map[touch_point_current - 1][0] == 0 && auto_num != 0)
                                    {
                                        sensor.mai_map[touch_point_current - 1][0] = auto_num;
                                        extra_point = 0;
                                        cmd = 1;
                                        auto_num++;
                                    }
                                    else if (sensor.mai_map[touch_point_current - 1][1] == 0 && auto_num != 0)
                                    {
                                        sensor.mai_map[touch_point_current - 1][1] = auto_num;
                                        extra_point = 1;
                                        cmd = 1;
                                        auto_num++;
                                    }
                                    else
                                    {
                                        cmd = 4;
                                    }
                                }
                                else if (multi > 1)
                                {
                                    cmd = 2;
                                }
                                else if (auto_num > 34)
                                {
                                    cmd = 3;
                                }
                                else
                                {
                                    cmd = 0;
                                }
                            }
                            multi_old = multi;
                        }
                    }
                    {
                        switch (cmd)
                        {
                        case 0:
                        {
                            display.draw_strings("TOUCH POINT          ", 1, 0);
                            break;
                        }
                        case 1:
                        {
                            display.draw_strings("OK          ", 1, 0);
                            break;
                        }
                        case 2:
                        {
                            display.draw_strings("NOT MORE 1          ", 1, 0);
                            break;
                        }
                        case 3:
                        {
                            display.draw_strings("SUCCESS 9>BACK ", 1, 0);
                            break;
                        }
                        case 4:
                        {
                            display.draw_strings("BIND MORE 2          ", 1, 0);
                            break;
                        }
                        case 10:
                        {
                            display.draw_strings("PRESS 1 BEGIN          ", 1, 0);
                            break;
                        }
                        }
                        display.draw_strings("C/T", 1, 1);
                        display.draw_uint32(auto_num, 18, 1, 4);
                        display.draw_uint32(touch_point_current, 42, 1, 4);
                        display.draw_strings("S>", 1, 2);
                        display.draw_uint32(sensor.mai_map[touch_point_current - 1][0], 14, 2, 2);
                        display.draw_uint32(sensor.mai_map[touch_point_current - 1][1], 36, 2, 2);
                        display.draw_strings("RST>1 6-7+", 1, 3);
                        display.draw_strings("MULTI>", 1, 4);
                        display.draw_uint32(multi, 42, 4, 2);
                        display.update();
                    }
                    break;
                }
                }
                /**
                 * 数据发送区
                 *
                 */
                if (core.mode == MODE_SERIAL)
                {
                    usb.serial_recv();
                }

                switch (mode & 0b1111)
                {
                case 0:
                {
                    switch (core.mode)
                    {
                    case MODE_TOUCH:
                    {
                        if (!extra_point)
                            usb.Send_touchscreen(255, sensor.touch_bar[touch_point_current - 1].x, sensor.touch_bar[touch_point_current - 1].y, touch_point_current - 1, 1, hid_timer);
                        else
                            usb.Send_touchscreen(255, sensor.touch_bar[touch_point_current + 7].x, sensor.touch_bar[touch_point_current + 7].y, touch_point_current - 1, 1, hid_timer);
                        hid_timer++;
                        break;
                    }
                    case MODE_SERIAL:
                    {

                        uint32_t ss[2] = {0};
                        if (!extra_point)
                        {
                            if (sensor.mai_map[touch_point_current - 1][0] <= 25)
                                ss[0] = 1 << sensor.mai_map[touch_point_current - 1][0];
                            else
                                ss[1] = 1 << (sensor.mai_map[touch_point_current - 1][0] - 25);
                            usb.Send_maiserial(ss[0], ss[1]);
                        }
                        else
                        {
                            if (sensor.mai_map[touch_point_current - 1][1] <= 25)
                                ss[0] = 1 << sensor.mai_map[touch_point_current - 1][1];
                            else
                                ss[1] = 1 << (sensor.mai_map[touch_point_current - 1][1] - 25);
                            usb.Send_maiserial(ss[0], ss[1]);
                        }
                        break;
                    }
                    }
                    break;
                }
                }
            }
            if ((mode & 0b1111) != 1 && core.serial_ok)
            { // TOUCH原始数据输出
                uart.send(" ");
                uart.send("---------------");
                uart.send("T:", 0);
                uart.send(sensor_readTouch);
                uart.send("IO:", 0);
                uart.send(sensor_readGPIO);

                uart.send("---------------");
            }
            uint32_t _old_sensor_readGPIO = sensor_readGPIO;
            while (sensor_readGPIO == _old_sensor_readGPIO && (core.status & 0xA0))
            {
                touch_triggle_show();
                key_triggle_show();
                if (core.mode == MODE_SERIAL)
                {
                    usb.serial_recv();
                }
                vTaskDelay(uart.serial_delay);
            }
        }
        static uint8_t keyboard_send[13] = {0};
        usb.Send_keyboard(keyboard_send);
        dog.feed();
    }
}

/**
 * 主循环
 *
 */
inline void main_loop()
{
#ifndef TOUCHREPORT_DELAY
    sensor_current = sensor_readTouch;
#endif
    sensor_gpiocurrent = sensor_readGPIO;
    
    check_touch(millis()); // 触摸阈值锁定检测
    /**
     * @huhuzhu
     * 触控上报部分
     */
    switch (core.mode)
    {
    case MODE_TOUCH:
    {
        multi = planner.multi_touch(sensor_readTouch);
        multi += planner.multi_extra_touch(sensor_readTouch);
        _multi = multi;

        if ((!multi))
            _multi = multi_old;
        for (uint8_t id = 0; id < TOUCH_NUM; id++)
        {
            if (sensor.check_down(sensor_current, id))
            {
                if (id > (TOUCH_NUM - 8))
                {
                    usb.Send_touchscreen(255, sensor.touch_bar[id + 8].x, sensor.touch_bar[id + 8].y, id + 8, _multi, hid_timer);
                    _multi = 0;
                }
                usb.Send_touchscreen(255, sensor.touch_bar[id].x, sensor.touch_bar[id].y, id, _multi, hid_timer);
                _multi = 0;
            }
            else if (sensor.check_down(sensor_old, id))
            {
                if (id > (TOUCH_NUM - 8))
                {
                    usb.Send_touchscreen(0, sensor.touch_bar[id + 8].x, sensor.touch_bar[id + 8].y, id + 8, _multi, hid_timer);
                    _multi = 0;
                }
                usb.Send_touchscreen(0, sensor.touch_bar[id].x, sensor.touch_bar[id].y, id, _multi, hid_timer);
                _multi = 0;
            }
        }
        multi_old = multi;
        hid_timer++;
        break;
    }
    case MODE_SERIAL:
    {
        usb.serial_recv();
        if (!core.serial_ok)
            break;
        millis_t _check_time = millis() - core.touch_delaytime;
#ifdef TOUCHREPORT_DELAY
        while (!(millis() > uart.serial_send_time))
        {
            dog.feed();
        }
        sensor_current = TouchQueue_Timeread(_check_time);
#endif
// 在触发式串口情况下 只有触摸内容更变才发送数据
#ifdef TRIGGLE_SERIAL_MODE
        if ((sensor_current) == (sensor_old))
        {
            dog.feed();
            // ets_delay_us(500);
            break;
        }
#endif
        if ((sensor_current) != (sensor_old))
        {
            report_clk += 1;
            triggle_hold_time = millis() + WARN_HOLDTIME;
            if (report_clk > TRIGGLE_TUNE)
                old_report_clk = TRIGGLE_TUNE + 1;
        }

        register uint32_t ss[2] = {0};
        register int32_t map[2] = {0};

        for (uint8_t id = 0; id < TOUCH_NUM; id++)
            if (sensor.check_down(sensor_current, id))
            {
                map[0] = sensor.mai_map[id][0];
                map[1] = sensor.mai_map[id][1];
                if (map[0] <= 25)
                    ss[0] |= 1 << map[0];
                else
                    ss[1] |= 1 << (map[0] - 25);
                if (map[1] <= 25)
                    ss[0] |= 1 << map[1];
                else
                    ss[1] |= 1 << (map[1] - 25);
            }
        ss[0] >>= 1;
        ss[1] >>= 1;

#ifdef AUTO_REGION_DETECT
        auto_region_detect(ss);
#endif

#ifdef A_PAD_TO_KEYBOARD
        // 将A1-A8区映射到keyboard
        if (old_report_clk < TRIGGLE_TUNE && usb.HID_state)
        {
            if ((millis() - triggle_hold_time < WARN_HOLDTIME) && (!sensor.check_down(ss[0], 0) && !sensor.check_down(ss[0], 7)))
            {
                // 当A1+A8被按下 不再处理其他模拟外键 同时屏蔽对应键
                sensor_gpiocurrent &= (uint32_t)(~(1 << 9));
                ss[0] &= 0xFFFFFF7E;
            }
            else
                for (uint8_t i = 0; i < 8; i++)
                    if (!sensor.check_down(ss[0], i))
                        sensor_gpiocurrent &= (uint32_t)(~(1 << i));
            // ss[0] &= 0xFFFFFF00;
            // 如果真的有跟hera一样操作的谱 直接双点未尝不可 实际上会被屏蔽映射 不会造成影响
        }
#endif
        usb.Send_maiserial(ss[0], ss[1]);
        break;
    }
    }

    /**
     * @huhuzhu
     * 键盘上报部分
     */
    uint8_t keyboard_send[13] = {0};

    for (uint8_t id = 0; id < BUTTON_NUM; id++)
    {
        if (sensor.check_down(sensor_gpiocurrent, id))
        {
            keyboard_send[id] = keyboard_map[id];
            keyboard_send[12] = 0xFF;
        }
    }
    usb.Send_keyboard(keyboard_send);
    sensor_old = sensor_current;
    dog.feed();
}