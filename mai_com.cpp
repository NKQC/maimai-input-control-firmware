#include "usb.h"
#include "../uart/uart.h"
#include "../../sensor_check/sensor.h"
#include "../../storage/storage.h"


enum
{
    commandRSET = 0x45,  // E
    commandHALT = 0x4C,  // L
    commandSTAT = 0x41,  // A
    commandRatio = 0x72, // r
    commandSens = 0x6B,  // k
};

// 显示所有点位绑定情况
inline void _show_all_bind_point()
{
    uart.send("-------------- ALL PORT BIND --------------");
    uart.send("POINT: NUM / A + B");
    for (uint8_t i = 0; i < TOUCH_NUM; i++)
    {
        uart.send("B:", 0);
        if (i + 1 < 10)
            uart.send(" ", 0);
        uart.send(i + 1, 0);
        uart.send(" / ", 0);
        uart.send(const_touchpoint[sensor.mai_map[i][0]], 0);
        uart.send(" + ", 0);
        uart.send(const_touchpoint[sensor.mai_map[i][1]], 0);
        uart.send(" ", 0);

        if ((i + 1) % 3 == 0)
            uart.send(" |");
    }
    uart.send("-------------- PORT BIND END ---------------");
}

// 显示触摸参考锁定情况
inline void _show_touch_lock_info()
{
    uart.send("-------------- ALL POINT LOCK INFO --------------");
    uart.send("POINT: NUM / AUTO/LOCK Current:(+)Lock (-)Auto");
    for (uint8_t i = 0; i < TOUCH_NUM; i++)
    {
        uart.send("P:", 0);
        if (i + 1 < 10)
            uart.send(" ", 0);
        uart.send(i + 1, 0);
        uart.send(" / ", 0);
        if (sensor.touch_refence_lock & (1 << i))
            uart.send("AUTO", 0);
        else
            uart.send("LOCK", 0);
        if (sensor.touch_refence_map & (1 << i))
            uart.send("+", 0);
        else
            uart.send("-", 0);
        uart.send(" ", 0);

        if ((i + 1) % 3 == 0)
            uart.send(" |");
    }
    uart.send("-------------- ALL POINT LOCK END ---------------");

    uart.send("AUTO TOUCH_SAMPLE_LOCK: ", 0);
    if (sensor.touch_refence_lock)
        uart.send("OFF");
    else
        uart.send("ON");
    uart.send("LOCK_CURRENT: ", 0);
    uart.send(sensor.touch_refence_map & 0x7FFFFFFF);
    
    uart.send("Set ALL Point AUTO:  {99P80}");
    uart.send("Set ALL Point AUTO LOCK: {99P90}");
    uart.send("Set One Point AUTO/LOCK: {99P(Point)} example:{99P1} -> Point 1 Set");
}

// [Serial定延迟流控] 检查时间是否满足发送
inline bool _need_serial_send()
{
    return millis() > uart.serial_send_time;
}

// [Serial定延迟流控] 开始下次计时
inline void _serial_send_end()
{
    if (uart.serial_delay == 0)
        uart.serial_send_time = 0;
    else
        uart.serial_send_time = millis() + uart.serial_delay;
}

// 显示所有点位灵敏度
inline void _show_all_sensitivity()
{
    int8_t sens, pres;
    uart.send("------------------ ALL TOUCH PORT ------------------");
    uart.send("POINT: NUM / SENS / PRUSSURE  min(15)->max(0)");
    for (uint8_t i = 0; i < TOUCH_NUM; i++)
    {
        uart.send("P:", 0);
        if (i + 1 < 10)
            uart.send(" ", 0);
        uart.send(i + 1, 0);
        uart.send(" / ", 0);
        int8_t sens = sensor.touch_spl_read(i);
        if (sens < 10)
            uart.send(" ", 0);
        uart.send(sens, 0);
        uart.send(" / ", 0);
        pres = sensor.touch_pressure_read(i);
        if (pres < 10)
            uart.send(" ", 0);
        uart.send(pres, 0);
        uart.send(" ", 0);
        if ((i + 1) % 3 == 0)
            uart.send(" |");
    }
    uart.send("------------------ TOUCH PORT END ------------------");
}

// send1 > 0-5 send2 > 6-7
void IRAM_ATTR USB_OTG::Send_maiserial(uint32_t send1, uint32_t send2)
{
    if (!core.serial_ok)
        return;
#ifndef TRIGGLE_SERIAL_MODE
    if (!_need_serial_send())
        return;
#endif

    // 优化：预先组装完整数据包，避免循环和多次write调用
    uint8_t packet[9] = {
        '(',
        (uint8_t)(send1 & 0b11111),
        (uint8_t)((send1 >> 5) & 0b11111),
        (uint8_t)((send1 >> 10) & 0b11111),
        (uint8_t)((send1 >> 15) & 0b11111),
        (uint8_t)((send1 >> 20) & 0b11111),
        (uint8_t)(send2 & 0b11111),
        (uint8_t)((send2 >> 5) & 0b11111),
        ')'
    };
    
    // 一次性发送完整数据包
    Serial0.write(packet, 9);
    _serial_send_end();
    Serial0.flush();
}

inline void serial_command() {
    static uint8_t settings_change = 0;
    char get = Serial0.read();
    if (get == '{')
    {
        uint8_t count = 1;
        uint8_t packet[COMMAND_LENGE] = {0};

        packet[0] = get;

        while (count < COMMAND_LENGE) // 拓展的命令宽度
        {
            packet[count] = Serial0.read();
            if (packet[count] == '}')
                break;
            count++;
        }

        switch (packet[3])
        {
        case commandRSET:
            // esp_restart();
            sensor.tsm12mc_reset();
            sensor.start_sample();
            core.serial_ok = 0;
            break;
        case commandHALT:
            break; // 设置模式
        case commandRatio:
        {
            Serial0.write('(');
            Serial0.write(packet[1]); // L,R
            Serial0.write(packet[2]); // sensor
            Serial0.write('r');
            Serial0.write(packet[4]); // Ratio
            Serial0.write(')');
            break;
        }
        case commandSens:
        {
            // sensor.touch_spl_set(uart.get_tsm12_id(packet[2]), packet[4] + 5);
            Serial0.write('(');
            Serial0.write('R' /*packet[1]*/); // L,R
            Serial0.write(packet[2]);         // sensor
            Serial0.write('k');
            Serial0.write(packet[4]); // Ratio
            settings_change = 1;
            Serial0.write(')');
        }
        break;
        case commandSTAT:
            sensor.start_sample();
            core.serial_ok = 1;
            if (settings_change)
            {
                nvs.nvs_storage();
                settings_change = 0;
            }
            break;

        // 附加指令

        // Baud {  U0}
        case 'U':
        {
            uint8_t is_change = 1;
            switch (packet[4])
            {
            case '0':
                uart.uart_speed = 9600;
                break;
            case '1':
                uart.uart_speed = 115200;
                break;
            case '2':
                uart.uart_speed = 250000;
                break;
            case '3':
                uart.uart_speed = 500000;
                break;
            case '4':
                uart.uart_speed = 1000000;
                break;
            case '5':
                uart.uart_speed = 1500000;
                break;
            case '6':
                uart.uart_speed = 2000000;
                break;
            default:
                is_change = 0;
                break;
            }
            if (is_change)
            {
                uart.send("INFO:UARTSPEED:", 0);
                uart.send(uart.uart_speed);
                uart.change_speed(uart.uart_speed);
                uart.send("INFO:UARTSPEED:", 0);
                uart.send(uart.uart_speed);
            }

            break;
        }
        // 设置单点灵敏度 {12S12} = Point:12 S:12
        case 'S':
        {
            char _point[2] = {packet[1], packet[2]};
            int32_t point = atoi(_point);
            int32_t value = 0;
            if (point == 0)
            {
                _show_all_sensitivity();
                break;
            }

            _point[0] = packet[4];
            if (packet[5] != '}')
            {
                _point[1] = packet[5];
                value = atoi(_point);
            }
            else
            {
                value = _point[0] - '0';
            }
            if (point > 90)
            {
                sensor.set_sample_time(value);
                break;
            }
            uart.set_sensitivity(point, value);
            break;
        }
        case 'P':
        {
            char _point[2] = {packet[1], packet[2]};
            int32_t point = atoi(_point);
            int32_t value = 0;
            if (point == 0)
            {
                _show_all_sensitivity();
                break;
            }
            _point[0] = packet[4];
            if (packet[5] != '}')
            {
                _point[1] = packet[5];
                value = atoi(_point);
            }
            else
            {
                value = _point[0] - '0';
            }
            if (point > 90)
            {
                if (value == 99 || !value)
                    _show_touch_lock_info();
                else
                {
                    if (value == 80)
                    {
                        sensor.touch_refence_lock = 0xFFFFFFFF;
                        uart.send("INFO:TOUCH_SAMPLE_LOCK SET AUTO: ALL");
                    }
                    else if (value == 90)
                    {
                        sensor.touch_refence_lock = 0;
                        uart.send("INFO:TOUCH_SAMPLE_LOCK SET LOCK: ALL");
                    }
                    else
                    {
                        if (sensor.touch_refence_lock & (1 << (value - 1)))
                        {
                            sensor.touch_refence_lock &= ~(1 << (value - 1));
                            uart.send("INFO:TOUCH_SAMPLE_LOCK SET LOCK:", 0);
                            uart.send(value);
                        }
                        else
                        {
                            sensor.touch_refence_lock |= (1 << (value - 1));
                            uart.send("INFO:TOUCH_SAMPLE_LOCK SET AUTO:", 0);
                            uart.send(value);
                        }
                        uart.send("DEBUG: REG->", 0);
                        uart.send(sensor.touch_refence_lock);
                    }
                    sensor.tsm12mc_reset();
                    sensor.start_sample();
                }
                break;
            }
            uart.set_pressure(point, value);
            break;
        }
        // 保存设置
        case 'C':
        {
            nvs.nvs_storage();
            break;
        }
// 设置延迟
#ifdef TOUCHREPORT_DELAY
        case 'D':
        {
            char p_point[2] = {0};
            int32_t value = 0;
            p_point[0] = packet[1];
            p_point[1] = packet[2];
            if (!atoi(p_point))
            {
                uart.send("INFO:DELAY_TIME:", 0);
                uart.send(core.touch_delaytime);
                break;
            }
            {
                char _point[3] = {0};
                _point[0] = packet[4];
                if (packet[4] != '}')
                {
                    _point[1] = packet[5];
                    if (_point[5] != '}')
                    {
                        _point[2] = packet[6];
                        value = atoi(_point);
                    }
                    else
                    {
                        value = atoi(_point);
                    }
                }
                else
                {
                    value = packet[4] - '0';
                }
                if (value >= 0 && value < TOUCHQUEUE_SIZE)
                {
                    core.touch_delaytime = value;
                    uart.send("INFO:CHANGE DELAY_TIME:", 0);
                    uart.send(core.touch_delaytime);
                }
                else
                {
                    uart.send("ERROR:DELAY_TIME:", 0);
                    uart.send(value, 0);
                    uart.send("TOO BIG Max:", 0);
                    uart.send(TOUCHQUEUE_SIZE - 1, 0);
                }
            }
            break;
        }
#endif
        // 显示绑定情况
        case 'B':
        {
            _show_all_bind_point();
            break;
        }
        case 'N': {
            core.status = (core.status & 0xA0) ? 0x2 : 0xA1;
            break;
        }
        default:
            uart.send("WARN:Unknow Command: ", 0);
            uart.send(" ");
            uart.send("------- COMMAND -------");
            uart.send("SettingsMode/Run: {00N0}");
            uart.send("SerialBaud: {  U0} = 9600");
            uart.send("1:115200 2:250000 3:500000 4:1000000 5:1500000 6:2000000");
            uart.send("Touch Sensitivity: {01S12} = Point:1 S:12  S-RANGE:(max)0-15(min)");
            uart.send("Sample Time: {99S00} = S-RANGE:(min)0-7(max)");
            uart.send("{00S0} or {00P0}: Show All Sensitivity And Pressure");
            uart.send("Triggle Pressure: {01P1} = Point:1 P:1  P-RANGE:(min)0-2(max)");
            uart.send("Touch_sample_lock MENU: {99P99}");
            uart.send("START: {  A }");
            uart.send("RESET_TOUCH: {  E }");
            uart.send("SAVE CONFIG: {  C }");
            uart.send("SHOW BIND: {  B }");
#ifdef TOUCHREPORT_DELAY
            uart.send("CHANGE DELAY: {01D123} = Delay:123ms {00D0} -> Show DelayTime");
#endif
            uart.send("--------- END ---------");
            break;
        }
        memset(packet, 0, COMMAND_LENGE);
        Serial0.flush();
    }
}

// MAIMAI 通讯指令
void IRAM_ATTR USB_OTG::serial_recv()
{
    serial_command();
}