#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <hardware/watchdog.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

// HAL层包含
#include "hal/i2c/hal_i2c.h"
#include "hal/uart/hal_uart.h"
#include "hal/spi/hal_spi.h"
#include "hal/pio/hal_pio.h"
#include "hal/usb/hal_usb.h"

extern "C" {
#include "hal/global_irq.h"
}

// 协议层包含
#include "protocol/touch_sensor/gtx312l/gtx312l.h"
#include "protocol/touch_sensor/touch_sensor.h"
#include "protocol/mcp23s17/mcp23s17.h"
#include "protocol/neopixel/neopixel.h"
#include "protocol/st7735s/st7735s.h"
#include "protocol/mai2serial/mai2serial.h"
#include "protocol/mai2light/mai2light.h"
#include "protocol/usb_serial_logs/usb_serial_logs.h"
#include "protocol/hid/hid.h"

// 服务层包含
#include "service/config_manager/config_manager.h"
#include "service/input_manager/input_manager.h"
#include "service/light_manager/light_manager.h"
#include "service/ui_manager/ui_manager.h"

// 系统配置
#define DEBUG_INPUTMANAGER_LOG false
#define DEBUG_LIGHTMANAGER_LOG false
#define DEBUG_UIMANAGER_LOG false

#define SYSTEM_VERSION "3.0.2"
#define HARDWARE_VERSION "3.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// 引脚定义
#define LED_BUILTIN_PIN 0
#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5
#define I2C1_SDA_PIN 6
#define I2C1_SCL_PIN 7
// ST7735S
#define SPI0_MISO_PIN 16 // 不使用引脚 但确实是SPI0 RX
#define SPI0_MOSI_PIN 19
#define SPI0_SCK_PIN 18
#define ST7735S_DC_PIN 21
#define ST7735S_RST_PIN 20
#define ST7735S_CS_PIN 17
#define ST7735S_BLK_PIN 22  // 背光PWM控制引脚
#define SPI0_FREQ 12000000
// MCP23S17
#define SPI1_MISO_PIN 28
#define SPI1_MOSI_PIN 27
#define SPI1_SCK_PIN 26
#define MCP23S17_CS_PIN 29
#define SPI1_FREQ 10000000

#define UART0_TX_PIN 12
#define UART0_RX_PIN 13
#define UART0_CTS_PIN 255//14
#define UART0_RTS_PIN 255//15
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9
#define NEOPIXEL_PIN 11
#define NEOPIXEL_LEDS_NUM 32

// 摇杆引脚定义
#define JOYSTICK_BUTTON_A_PIN 2    // 摇杆A按钮(上方向)
#define JOYSTICK_BUTTON_B_PIN 3    // 摇杆B按钮(下方向)
#define JOYSTICK_BUTTON_CONFIRM_PIN 1  // 摇杆确认按钮

// Watchdog配置
#define WATCHDOG_TIMEOUT_MS 5000
#define WATCHDOG_FEED_INTERVAL_MS 1000

// 双核心初始化同步bitmap结构体
struct CoreInitBitmap {
    volatile uint32_t core0_hal_ready : 1;
    volatile uint32_t core1_hal_ready : 1;
    volatile uint32_t core0_protocol_ready : 1;
    volatile uint32_t core1_protocol_ready : 1;
    volatile uint32_t service_ready : 1;
    volatile uint32_t usb_log_ready : 1;  // USB log可用标志
    volatile uint32_t core1_failed : 1;   // Core1启动失败标志
    volatile uint32_t reserved : 25;
    
    inline void reset() volatile {
        *(volatile uint32_t*)this = 0;
    }
    
    // 简化的等待函数，支持USB log flush
    inline bool wait_for_both_hal(uint32_t timeout_ms = 5000) volatile {
        uint32_t start = millis();
        while ((!core0_hal_ready || !core1_hal_ready) && (millis() - start) < timeout_ms) {
            // 如果当前是Core1且USB log可用，则flush
            if (get_core_num() == 1 && usb_log_ready) {
                USB_SerialLogs* logs = USB_SerialLogs::get_global_instance();
                if (logs) {
                    logs->flush();
                }
            }
            tight_loop_contents();
        }
        return core0_hal_ready && core1_hal_ready;
    }
    
    inline bool wait_for_both_protocol(uint32_t timeout_ms = 30000) volatile {
        uint32_t start = millis();
        while ((!core0_protocol_ready || !core1_protocol_ready) && (millis() - start) < timeout_ms) {
            // 如果当前是Core1且USB log可用，则flush
            if (get_core_num() == 1 && usb_log_ready) {
                USB_SerialLogs* logs = USB_SerialLogs::get_global_instance();
                if (logs) {
                    logs->flush();
                }
            }
            tight_loop_contents();
        }
        return core0_protocol_ready && core1_protocol_ready;
    }
    
    // 检查Core1是否启动失败
    inline bool is_core1_failed() volatile {
        return core1_failed;
    }
    
    // 标记Core1启动失败
    inline void mark_core1_failed() volatile {
        core1_failed = 1;
    }
};

// Core1 stack
#define CORE1_STACK_SIZE 0x10000
static uint32_t core1_stack[CORE1_STACK_SIZE / sizeof(uint32_t)];

// 全局同步bitmap
static volatile CoreInitBitmap init_sync;

// 全局对象声明
static HAL_I2C* hal_i2c0 = nullptr;
static HAL_I2C* hal_i2c1 = nullptr;
static HAL_SPI* hal_spi0 = nullptr;
static HAL_SPI* hal_spi1 = nullptr;
static HAL_UART* hal_uart0 = nullptr;
static HAL_UART* hal_uart1 = nullptr;
static HAL_PIO* hal_pio1 = nullptr;
static HAL_USB* hal_usb = nullptr;

static TouchSensorManager* touch_sensor_manager = nullptr; // 新的触摸传感器管理器
static MCP23S17* mcp23s17 = nullptr;
static NeoPixel* neopixel = nullptr;
static ST7735S* st7735s = nullptr;
static Mai2Serial* mai2_serial = nullptr;
static Mai2Light* mai2_light = nullptr;
static USB_SerialLogs* usb_logs = nullptr;
static HID* hid = nullptr;

static ConfigManager* config_manager = nullptr;
static InputManager* input_manager = nullptr;
static LightManager* light_manager = nullptr;
static UIManager* ui_manager = nullptr;

// 系统状态
static bool system_error = false;
static uint32_t last_watchdog_feed[2] = {0, 0};

// 函数声明
bool core0_init_hal_layer();
bool core1_init_hal_layer();
bool core0_init_protocol_layer();
bool core1_init_protocol_layer();
bool init_service_layer();
inline bool init_basic();
void deinit_system();
void core0_task();
void core1_task();
void core1_main();
void status_update_task();
void print_system_info();
void AutoRegisterTouchSensor();
void emergency_shutdown();


/**
 * 心跳任务
 */
inline void heartbeat_task() {
    static uint32_t last_heartbeat = 0;
    static uint8_t next_core = 0;
    uint32_t current_time = (time_us_32() / 1000);
    if (current_time - last_heartbeat > 500 && get_core_num() == next_core) {
        gpio_put(LED_BUILTIN_PIN, next_core);
        next_core = !next_core;
        last_heartbeat = current_time;
    }
}

/**
 * 看门狗任务
 */
inline void watchdog_feed() {
    uint32_t current_time = (time_us_32() / 1000);
    uint8_t core = get_core_num();
    if (current_time - last_watchdog_feed[core] >= WATCHDOG_FEED_INTERVAL_MS) {
        watchdog_update();
        last_watchdog_feed[core] = current_time;
    }
}

/**
 * 错误处理函数
 */
void error_handler(const char* error_msg) {
    system_error = true;
    
    if (usb_logs) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "SYSTEM ERROR: %s", error_msg);
        usb_logs->error(std::string(buffer));
        ui_manager->show_error(std::string(buffer));
    }
    
    while (1) {
        ui_manager->task(); 
        usb_logs->task();
        watchdog_feed();
    }
}

/**
 * 基础初始化
 * 失败时直接重启系统
 */
inline bool init_basic() {
    gpio_put(LED_BUILTIN_PIN, 1);
    // 初始化USB
    hal_usb = HAL_USB_Device::getInstance();
    if (!hal_usb || !hal_usb->init()) {
        watchdog_reboot(0, 0, 0);
        return false;
    }
    
    // 初始化USB Serial Logs
    usb_logs = new USB_SerialLogs(hal_usb);
    if (!usb_logs || !usb_logs->init()) {
        watchdog_reboot(0, 0, 0);
        return false;
    }

    USB_SerialLogs_Config log_config;
    log_config.enable_colors = false;
    log_config.min_level = USB_LogLevel::DEBUG;
    usb_logs->set_config(log_config);
    
    // 设置全局USB logs实例并标记可用
    USB_SerialLogs::set_global_instance(usb_logs);
    init_sync.usb_log_ready = 1;

    // 初始化SPI
    hal_spi0 = HAL_SPI0::getInstance();
    if (!hal_spi0 || !hal_spi0->init(SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_FREQ)) {
        error_handler("Failed to initialize SPI0");
        return false;
    }

    // 初始化ST7735S
    st7735s = new ST7735S(hal_spi0, ST7735S_ROTATION_90, ST7735S_CS_PIN, ST7735S_DC_PIN, ST7735S_RST_PIN, ST7735S_BLK_PIN);
    if (!st7735s || !st7735s->init()) {
        error_handler("Failed to initialize ST7735S");
        return false;
    }

    // 初始化ConfigManager
    config_manager = ConfigManager::getInstance();
    if (!config_manager || !config_manager->initialize()) {
        error_handler("Failed to initialize ConfigManager");
    }
    
    ui_manager = UIManager::getInstance();
    if (!ui_manager) {
        error_handler("Failed to create UIManager");
        return false;
    }
    
    UIManager_Config ui_config = {};
    ui_config.config_manager = config_manager;
    ui_config.light_manager = light_manager;
    ui_config.st7735s = st7735s;
    ui_config.joystick_a_pin = JOYSTICK_BUTTON_A_PIN;
    ui_config.joystick_b_pin = JOYSTICK_BUTTON_B_PIN;
    ui_config.joystick_confirm_pin = JOYSTICK_BUTTON_CONFIRM_PIN;
    
    if (!ui_manager->init(ui_config)) {
        error_handler("Failed to initialize UIManager");
        return false;
    }
    
    ui_manager->enable_debug_output(DEBUG_UIMANAGER_LOG);
    gpio_put(LED_BUILTIN_PIN, 0);
    return true;
}

/**
 * 打印系统信息
 */
void print_system_info() {
    if (usb_logs) {
        usb_logs->infof("=== Mai2 Controller V%s ===", SYSTEM_VERSION);
        usb_logs->infof("Hardware Version: %s", HARDWARE_VERSION);
        usb_logs->infof("Build Date: %s %s", BUILD_DATE, BUILD_TIME);
        usb_logs->infof("CPU Frequency: %lu MHz", rp2040.f_cpu() / 1000000);
        usb_logs->info("==============================");
    }
}

/**
 * HAL层初始化
 */
/**
 * Core0 HAL层初始化 - 负责I2C、UART、PIO
 */
bool core0_init_hal_layer() {

    // 初始化UART
    hal_uart0 = HAL_UART0::getInstance();
    if (!hal_uart0 || !hal_uart0->init(UART0_TX_PIN, UART0_RX_PIN, 9600, true, UART0_CTS_PIN, UART0_RTS_PIN)) {
        error_handler("Failed to initialize UART0");
        return false;
    }
    
    hal_uart1 = HAL_UART1::getInstance();
    if (!hal_uart1 || !hal_uart1->init(UART1_TX_PIN, UART1_RX_PIN, 9600)) {
        error_handler("Failed to initialize UART1");
        return false;
    }

    // 初始化I2C
    hal_i2c0 = HAL_I2C0::getInstance();
    if (!hal_i2c0 || !hal_i2c0->init(I2C0_SDA_PIN, I2C0_SCL_PIN, 400000)) {
        error_handler("Failed to initialize I2C0");
        return false;
    }
    
    hal_i2c1 = HAL_I2C1::getInstance();
    if (!hal_i2c1 || !hal_i2c1->init(I2C1_SDA_PIN, I2C1_SCL_PIN, 400000)) {
        error_handler("Failed to initialize I2C1");
        return false;
    }
    
    // 初始化PIO
    hal_pio1 = HAL_PIO1::getInstance();
    if (!hal_pio1 || !hal_pio1->init(NEOPIXEL_PIN)) {
        error_handler("Failed to initialize PIO1");
        return false;
    }
    
    // 标记Core0 HAL层初始化完成
    init_sync.core0_hal_ready = 1;
    
    return true;
}

/**
 * Core1 HAL层初始化 - 负责SPI、USB
 */
bool core1_init_hal_layer() {
    hal_spi1 = HAL_SPI1::getInstance();
    if (!hal_spi1 || !hal_spi1->init(SPI1_SCK_PIN, SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_FREQ)) {
        error_handler("Failed to initialize SPI1");
        return false;
    }
    
    // 标记Core1 HAL层初始化完成
    init_sync.core1_hal_ready = 1;
    
    return true;
}

/**
 * Core0 协议层初始化 - 负责GTX312L、NeoPixel、Mai2Serial、Mai2Light
 */
bool core0_init_protocol_layer() {
    // 等待两个核心的HAL层都初始化完成
    if (!init_sync.wait_for_both_hal()) {
        error_handler("Timeout waiting for HAL layer initialization");
        return false;
    }
    
    // 初始化NeoPixel
    neopixel = new NeoPixel(hal_pio1, NEOPIXEL_LEDS_NUM);
    if (!neopixel || !neopixel->init()) {
        error_handler("Failed to initialize NeoPixel");
        return false;
    }
    
    // 初始化Mai2Serial
    mai2_serial = new Mai2Serial(hal_uart0);
    if (!mai2_serial || !mai2_serial->init()) {
        error_handler("Failed to initialize Mai2Serial");
        return false;
    }
    
    // 初始化Mai2Light
    mai2_light = new Mai2Light(hal_uart1);
    if (!mai2_light || !mai2_light->init()) {
        error_handler("Failed to initialize Mai2Light");
        return false;
    }
    
    // 标记Core0 协议层初始化完成
    init_sync.core0_protocol_ready = 1;
    
    return true;
}

/**
 * Core1 协议层初始化 - 负责USB Serial Logs、ST7735S、HID、MCP23S17
 */
bool core1_init_protocol_layer() {
    // 等待两个核心的HAL层都初始化完成
    if (!init_sync.wait_for_both_hal()) {
        error_handler("Timeout waiting for HAL layer initialization");
        return false;
    }
    
    // 初始化MCP23S17
    mcp23s17 = new MCP23S17(hal_spi1, MCP23S17_CS_PIN);
    if (!mcp23s17 || !mcp23s17->init()) {
        error_handler("Failed to initialize MCP23S17");
        return false;
    }

    // 初始化HID (使用单例模式)
    hid = HID::getInstance();
    if (!hid || !hid->init(hal_usb)) {
        error_handler("Failed to initialize HID");
        return false;
    }
    
    // 标记Core1 协议层初始化完成
    init_sync.core1_protocol_ready = 1;
    
    return true;
}

/**
 * 服务层初始化
 */
/**
 * 服务层初始化 - 在Core0上运行，等待两个核心的协议层都初始化完成
 */
bool init_service_layer() {
    // 等待两个核心的协议层都初始化完成
    if (!init_sync.wait_for_both_protocol()) {
        error_handler("Timeout waiting for protocol layer initialization");
        return false;
    }
    
    // 初始化InputManager
    input_manager = InputManager::getInstance();
    
    // 准备InputManager初始化配置
    InputManager::InitConfig input_config;
    input_config.mai2_serial = mai2_serial;
    input_config.hid = hid;
    input_config.ui_manager = ui_manager;
    input_config.mcp23s17 = mcp23s17;
    
    if (!input_manager->init(input_config)) {
        error_handler("Failed to initialize InputManager");
        return false;
    }

    // 注册MCP23S17的GPIO 1-11作为键盘
    // 设置GPIOB8为输出模式并输出高电平以点亮LED
    mcp23s17->set_pin_direction(MCP23S17_PORT_B, 7, MCP23S17_OUTPUT); // GPIOB8是端口B的第7位(0-7)
    mcp23s17->write_pin(MCP23S17_PORT_B, 7, 0);

    // 注册按键
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA0, HID_KeyCode::KEY_W);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA1, HID_KeyCode::KEY_E);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA2, HID_KeyCode::KEY_D);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA3, HID_KeyCode::KEY_C);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA4, HID_KeyCode::KEY_X);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA5, HID_KeyCode::KEY_Z);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA6, HID_KeyCode::KEY_A);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA7, HID_KeyCode::KEY_Q);

    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB0, HID_KeyCode::KEY_8);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB1, HID_KeyCode::KEY_3);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB2, HID_KeyCode::KEY_ENTER);
    input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB3, HID_KeyCode::KEY_SPACE);

    // 注册Serial TouchArea -> Keyboard
    input_manager->addTouchKeyboardMapping(MAI2_A1_AREA, 1000, HID_KeyCode::KEY_W);
    input_manager->addTouchKeyboardMapping(MAI2_A2_AREA, 1000, HID_KeyCode::KEY_E);
    input_manager->addTouchKeyboardMapping(MAI2_A3_AREA, 1000, HID_KeyCode::KEY_D);
    input_manager->addTouchKeyboardMapping(MAI2_A4_AREA, 1000, HID_KeyCode::KEY_C);
    input_manager->addTouchKeyboardMapping(MAI2_A5_AREA, 1000, HID_KeyCode::KEY_X);
    input_manager->addTouchKeyboardMapping(MAI2_A6_AREA, 1000, HID_KeyCode::KEY_Z);
    input_manager->addTouchKeyboardMapping(MAI2_A7_AREA, 1000, HID_KeyCode::KEY_A);
    input_manager->addTouchKeyboardMapping(MAI2_A8_AREA, 1000, HID_KeyCode::KEY_Q);
    input_manager->addTouchKeyboardMapping(MAI2_B1_AREA | MAI2_B8_AREA | MAI2_E1_AREA, 1000, HID_KeyCode::KEY_SPACE, true);
    input_manager->addTouchKeyboardMapping(MAI2_C1_AREA | MAI2_C2_AREA, 1000, HID_KeyCode::KEY_ENTER);
    input_manager->addTouchKeyboardMapping(MAI2_D3_AREA | MAI2_D7_AREA, 1000, HID_KeyCode::KEY_F8);

    // 初始化LightManager
    light_manager = LightManager::getInstance();
    
    LightManager::InitConfig light_config;
    light_config.mai2light = mai2_light;
    light_config.neopixel = neopixel; 

    if (!light_manager->init(light_config)) {
        error_handler("Failed to initialize LightManager");
        return false;
    }
    light_manager->enable_debug_output(DEBUG_LIGHTMANAGER_LOG);
    // 扫描I2C设备并初始化触摸设备
    AutoRegisterTouchSensor();
    // 启用InputManager的debug模式以便定位CPU0锁死问题
    input_manager->set_debug_enabled(DEBUG_INPUTMANAGER_LOG);
    
    // 启动InputManager - 分配设备到采样阶段
    input_manager->start();
    
    // 标记服务层初始化完成
    init_sync.service_ready = 1;
    
    return true;
}


/**
 * 扫描I2C设备并初始化触摸传感器
 */
void AutoRegisterTouchSensor() {
    
    if (usb_logs) {
        usb_logs->info("Starting I2C device scan...");
    }
    
    // 创建TouchSensorManager实例
    if (!touch_sensor_manager) {
        touch_sensor_manager = new TouchSensorManager();
    }
    
    // 使用新的统一扫描接口
    uint8_t total_devices = touch_sensor_manager->scanAndRegisterAll(hal_i2c0, hal_i2c1, 8);
    
    for (uint8_t i = 0; i < total_devices; i++) {
        TouchSensor* sensor = touch_sensor_manager->getSensor(i);
        if (sensor) {
            USB_LOG_DEBUG("TouchSensor ID: %0x found", sensor->getModuleMask());
            // 注册到InputManager
            if (input_manager) {
                input_manager->registerTouchSensor(sensor);
            }
        }
    }
    
}

/**
 * 系统反初始化
 */
void deinit_system() {
    // 反初始化服务层
    if (ui_manager) {
        ui_manager->deinit();
        ui_manager = nullptr;
    }
    
    if (light_manager) {
        light_manager->deinit();
        light_manager = nullptr;
    }
    
    if (input_manager) {
        input_manager->deinit();
        input_manager = nullptr;
    }
    
    if (config_manager) {
        config_manager->deinit();
        config_manager = nullptr;
    }
    
    // 反初始化协议层
    if (hid) {
        hid->deinit();
        delete hid;
        hid = nullptr;
    }
    
    if (usb_logs) {
        usb_logs->deinit();
        delete usb_logs;
        usb_logs = nullptr;
    }
    
    if (mai2_light) {
        mai2_light->deinit();
        delete mai2_light;
        mai2_light = nullptr;
    }
    
    if (mai2_serial) {
        mai2_serial->deinit();
        delete mai2_serial;
        mai2_serial = nullptr;
    }
    
    if (st7735s) {
        st7735s->deinit();
        delete st7735s;
        st7735s = nullptr;
    }
    
    if (neopixel) {
        neopixel->deinit();
        delete neopixel;
        neopixel = nullptr;
    }
    
    if (mcp23s17) {
        mcp23s17->deinit();
        delete mcp23s17;
        mcp23s17 = nullptr;
    }
    
    // 反初始化触摸传感器管理器（包含所有触摸传感器设备）
    if (touch_sensor_manager) {
        delete touch_sensor_manager;
        touch_sensor_manager = nullptr;
    }
    
    // 反初始化HAL层
    if (hal_usb) {
        hal_usb->deinit();
        delete hal_usb;
        hal_usb = nullptr;
    }
    
    if (hal_pio1) {
        hal_pio1->deinit();
        delete hal_pio1;
        hal_pio1 = nullptr;
    }
    
    if (hal_uart1) {
        hal_uart1->deinit();
        delete hal_uart1;
        hal_uart1 = nullptr;
    }
    
    if (hal_uart0) {
        hal_uart0->deinit();
        delete hal_uart0;
        hal_uart0 = nullptr;
    }
    
    if (hal_spi1) {
        hal_spi1->deinit();
        delete hal_spi1;
        hal_spi1 = nullptr;
    }
    
    if (hal_spi0) {
        hal_spi0->deinit();
        delete hal_spi0;
        hal_spi0 = nullptr;
    }
    
    if (hal_i2c1) {
        hal_i2c1->deinit();
        delete hal_i2c1;
        hal_i2c1 = nullptr;
    }
    
    if (hal_i2c0) {
        hal_i2c0->deinit();
        delete hal_i2c0;
        hal_i2c0 = nullptr;
    }
}

/**
 * Core0任务 - UART透传测试功能
 * 实现UART0和UART1之间的双向透传
 */
void core0_task() {
    while (1) {
        input_manager->task0();
        config_manager->save_config_task();  // 处理配置保存请求
        heartbeat_task();
        watchdog_feed();
    }
}

/**
 * Core1任务 - InputManager Loop1, UIManager Loop
 */
void core1_task() {
    while (1) {
        input_manager->task1();
        usb_logs->task();
        ui_manager->task();
        light_manager->task();
        heartbeat_task();
        watchdog_feed();
    }
}

/**
 * 紧急关机
 */
void emergency_shutdown() {
    if (usb_logs) usb_logs->flush();
    // 关闭所有LED
    if (neopixel) {
        neopixel->clear_all();
        neopixel->show();
    }
    
    // 关闭屏幕背光
    if (st7735s) {
        st7735s->set_backlight(0);
    }
    
    // 禁用看门狗
    watchdog_disable();
    
    // 等待一段时间后重启
    delay(5000);
    watchdog_reboot(0, 0, 0);
}

/**
 * Arduino setup函数
 */
void setup() {
    // 初始化全局DMA中断管理系统
    global_irq_init();
    
    // 启用看门狗
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    
    // 重置同步bitmap
    init_sync.reset();

    // 提前初始化USB和usblog
    if (!init_basic()) {
        error_handler("Failed to initialize basic layer (USB, USB logs, UI)");
    }
    
    // 启动Core1
    multicore_launch_core1_with_stack(core1_main, core1_stack, CORE1_STACK_SIZE);
    // Core0初始化流程
    bool init_success = true;
    // Core0 HAL层初始化
    if (!core0_init_hal_layer()) {
        init_success = false;
    }
    
    // Core0 协议层初始化
    if (init_success && !core0_init_protocol_layer()) {
        init_success = false;
    }
    
    // 检查Core1是否启动失败
    if (init_success && init_sync.is_core1_failed()) {
        init_success = false;
        if (usb_logs) {
            usb_logs->error("Core1 initialization failed");
            usb_logs->flush();
        }
    }

    // 初始化服务层
    if (init_success && !init_service_layer()) {
        init_success = false;
    }
    
    if (init_success) {
        print_system_info();
        if (usb_logs) {
            usb_logs->info("System initialization completed successfully");
        }
        // 进入Core0任务循环
        core0_task();
    } else {
        error_handler("System initialization failed");
    }
}


/**
 * Core1主函数 - 处理Core1的初始化和任务循环
 */
void core1_main() {
    multicore_lockout_victim_init();
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    gpio_init(LED_BUILTIN_PIN);
    gpio_set_dir(LED_BUILTIN_PIN, GPIO_OUT);
    gpio_put(LED_BUILTIN_PIN, 0);
    
    bool init_success = true;
    
    // Core1 HAL层初始化
    if (!core1_init_hal_layer()) {
        init_success = false;
    }
    
    // Core1 协议层初始化
    if (init_success && !core1_init_protocol_layer()) {
        init_success = false;
    }
    
    if (!init_success) {
        // 标记Core1启动失败
        init_sync.mark_core1_failed();
        error_handler("Core1 initialization failed");
        return;
    }
    
    // 等待服务层初始化完成，期间flush USB logs
    uint32_t start = millis();
    while (!init_sync.service_ready && (millis() - start) < 5000) {
        watchdog_feed();
        if (usb_logs) {
            usb_logs->flush();
        }
    }
    
    if (!init_sync.service_ready) {
        init_sync.mark_core1_failed();
        
        error_handler("Timeout waiting for service layer initialization");
        return;
    }
    
    // 进入Core1任务循环
    core1_task();
}


void loop() {}
