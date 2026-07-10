#include "mcp23s17.h"
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <cstring>


MCP23S17::MCP23S17(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t device_addr)
    : spi_hal_(spi_hal), cs_pin_(cs_pin), device_addr_(device_addr & 0x07),
      initialized_(false), state_changed_(false) {
    memset(&last_state_, 0, sizeof(last_state_));
}

MCP23S17::~MCP23S17() {
    deinit();
}

bool MCP23S17::init() {
    if (initialized_) {
        return true;
    }
    
    
    if (!spi_hal_ || !spi_hal_->is_ready()) {
        return false;
    }
    
    // 配置CS引脚
    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    gpio_put(cs_pin_, 1);  // CS高电平（未选中）
    
    // 配置SPI
    spi_hal_->set_format(8, 0, 0);  // 8位，模式0
    
    // 测试设备通信
    if (!test_device_communication()) {
        return false;
    }
    
    // 配置设备
    if (!configure_device()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

void MCP23S17::deinit() {
    if (initialized_) {
        // 清除所有中断
        clear_interrupts();
        
        // 重置CS引脚
        gpio_put(cs_pin_, 1);
        
        initialized_ = false;
        interrupt_callback_ = nullptr;
    }
}

bool MCP23S17::is_ready() const {
    return initialized_;
}

bool MCP23S17::set_pin_direction(MCP23S17_Port port, uint8_t pin, MCP23S17_Direction dir) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_IODIRA : MCP23S17_REG_IODIRB;
    uint8_t current_value;
    
    if (!read_register(reg, current_value)) {
        return false;
    }
    
    if (dir == MCP23S17_INPUT) {
        current_value |= (1 << pin);
    } else {
        current_value &= ~(1 << pin);
    }
    
    return write_register(reg, current_value);
}

bool MCP23S17::set_port_direction(MCP23S17_Port port, uint8_t direction_mask) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_IODIRA : MCP23S17_REG_IODIRB;
    return write_register(reg, direction_mask);
}

bool MCP23S17::write_pin(MCP23S17_Port port, uint8_t pin, bool value) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPIOA : MCP23S17_REG_GPIOB;
    uint8_t current_value;
    
    if (!read_register(reg, current_value)) {
        return false;
    }
    
    if (value) {
        current_value |= (1 << pin);
    } else {
        current_value &= ~(1 << pin);
    }
    
    return write_register(reg, current_value);
}

bool MCP23S17::write_port(MCP23S17_Port port, uint8_t value) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPIOA : MCP23S17_REG_GPIOB;
    return write_register(reg, value);
}

bool MCP23S17::read_pin(MCP23S17_Port port, uint8_t pin, bool& value) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t port_value;
    if (!read_port(port, port_value)) {
        return false;
    }
    
    value = (port_value & (1 << pin)) != 0;
    return true;
}

bool MCP23S17::read_port(MCP23S17_Port port, uint8_t& value) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPIOA : MCP23S17_REG_GPIOB;
    return read_register(reg, value);
}

bool MCP23S17::read_all_gpio(MCP23S17_GPIO_State& state) {
    if (!is_ready()) {
        return false;
    }
    
    bool result = read_register_pair(MCP23S17_REG_GPIOA, state.port_a, 
                                    MCP23S17_REG_GPIOB, state.port_b);
    if (result) {
        state.timestamp = time_us_32();
    }
    
    return result;
}

bool MCP23S17::set_pin_pullup(MCP23S17_Port port, uint8_t pin, bool enable) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPPUA : MCP23S17_REG_GPPUB;
    uint8_t current_value;
    
    if (!read_register(reg, current_value)) {
        return false;
    }
    
    if (enable) {
        current_value |= (1 << pin);
    } else {
        current_value &= ~(1 << pin);
    }
    
    return write_register(reg, current_value);
}

bool MCP23S17::set_port_pullup(MCP23S17_Port port, uint8_t pullup_mask) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPPUA : MCP23S17_REG_GPPUB;
    return write_register(reg, pullup_mask);
}

bool MCP23S17::set_pin_polarity(MCP23S17_Port port, uint8_t pin, bool inverted) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_IPOLA : MCP23S17_REG_IPOLB;
    uint8_t current_value;
    
    if (!read_register(reg, current_value)) {
        return false;
    }
    
    if (inverted) {
        current_value |= (1 << pin);
    } else {
        current_value &= ~(1 << pin);
    }
    
    return write_register(reg, current_value);
}

bool MCP23S17::set_port_polarity(MCP23S17_Port port, uint8_t polarity_mask) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_IPOLA : MCP23S17_REG_IPOLB;
    return write_register(reg, polarity_mask);
}

bool MCP23S17::enable_pin_interrupt(MCP23S17_Port port, uint8_t pin, MCP23S17_IntType type, uint8_t compare_value) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t gpinten_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPINTENA : MCP23S17_REG_GPINTENB;
    uint8_t intcon_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_INTCONA : MCP23S17_REG_INTCONB;
    uint8_t defval_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_DEFVALA : MCP23S17_REG_DEFVALB;
    
    // 启用中断
    uint8_t gpinten_value;
    if (!read_register(gpinten_reg, gpinten_value)) {
        return false;
    }
    gpinten_value |= (1 << pin);
    if (!write_register(gpinten_reg, gpinten_value)) {
        return false;
    }
    
    // 设置中断类型
    uint8_t intcon_value;
    if (!read_register(intcon_reg, intcon_value)) {
        return false;
    }
    
    if (type == MCP23S17_INT_COMPARE) {
        intcon_value |= (1 << pin);
        
        // 设置比较值
        uint8_t defval_value;
        if (!read_register(defval_reg, defval_value)) {
            return false;
        }
        
        if (compare_value) {
            defval_value |= (1 << pin);
        } else {
            defval_value &= ~(1 << pin);
        }
        
        if (!write_register(defval_reg, defval_value)) {
            return false;
        }
    } else {
        intcon_value &= ~(1 << pin);
    }
    
    return write_register(intcon_reg, intcon_value);
}

bool MCP23S17::disable_pin_interrupt(MCP23S17_Port port, uint8_t pin) {
    if (!is_ready() || pin > 7) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPINTENA : MCP23S17_REG_GPINTENB;
    uint8_t current_value;
    
    if (!read_register(reg, current_value)) {
        return false;
    }
    
    current_value &= ~(1 << pin);
    return write_register(reg, current_value);
}

bool MCP23S17::enable_port_interrupt(MCP23S17_Port port, uint8_t interrupt_mask, MCP23S17_IntType type, uint8_t compare_value) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t gpinten_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPINTENA : MCP23S17_REG_GPINTENB;
    uint8_t intcon_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_INTCONA : MCP23S17_REG_INTCONB;
    uint8_t defval_reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_DEFVALA : MCP23S17_REG_DEFVALB;
    
    // 启用中断
    if (!write_register(gpinten_reg, interrupt_mask)) {
        return false;
    }
    
    // 设置中断类型
    if (type == MCP23S17_INT_COMPARE) {
        if (!write_register(intcon_reg, interrupt_mask)) {
            return false;
        }
        if (!write_register(defval_reg, compare_value)) {
            return false;
        }
    } else {
        if (!write_register(intcon_reg, 0x00)) {
            return false;
        }
    }
    
    return true;
}

bool MCP23S17::disable_port_interrupt(MCP23S17_Port port) {
    if (!is_ready()) {
        return false;
    }
    
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_REG_GPINTENA : MCP23S17_REG_GPINTENB;
    return write_register(reg, 0x00);
}

bool MCP23S17::read_interrupt_flags(uint8_t& intf_a, uint8_t& intf_b) {
    if (!is_ready()) {
        return false;
    }
    
    return read_register_pair(MCP23S17_REG_INTFA, intf_a, MCP23S17_REG_INTFB, intf_b);
}

bool MCP23S17::read_interrupt_capture(uint8_t& intcap_a, uint8_t& intcap_b) {
    if (!is_ready()) {
        return false;
    }
    
    return read_register_pair(MCP23S17_REG_INTCAPA, intcap_a, MCP23S17_REG_INTCAPB, intcap_b);
}

bool MCP23S17::clear_interrupts() {
    if (!is_ready()) {
        return false;
    }
    
    // 读取GPIO寄存器以清除中断
    uint8_t dummy_a, dummy_b;
    return read_register_pair(MCP23S17_REG_GPIOA, dummy_a, MCP23S17_REG_GPIOB, dummy_b);
}

void MCP23S17::set_interrupt_callback(std::function<void(const MCP23S17_GPIO_State&, uint8_t, uint8_t)> callback) {
    interrupt_callback_ = callback;
}

void MCP23S17::handle_interrupt() {
    if (!is_ready()) {
        return;
    }
    
    // 读取中断标志
    uint8_t intf_a, intf_b;
    if (!read_interrupt_flags(intf_a, intf_b)) {
        return;
    }
    
    // 读取当前GPIO状态
    MCP23S17_GPIO_State current_state;
    if (!read_all_gpio(current_state)) {
        return;
    }
    
    // 清除中断
    clear_interrupts();
    
    // 调用回调函数
    if (interrupt_callback_) {
        interrupt_callback_(current_state, intf_a, intf_b);
    }
    
    last_state_ = current_state;
}

void MCP23S17::task() {
    if (!is_ready()) {
        return;
    }
    
    // 定期检查GPIO状态变化（如果没有使用中断）
    static uint32_t last_poll_time = 0;
    uint32_t current_time = time_us_32();
    
    // 每50ms轮询一次
    if (current_time - last_poll_time >= 50000) {
        last_poll_time = current_time;
        
        MCP23S17_GPIO_State current_state;
        if (read_all_gpio(current_state)) {
            // 检查是否有变化
            if (current_state.port_a != last_state_.port_a || 
                current_state.port_b != last_state_.port_b) {
                
                // 计算变化的位
                uint8_t changed_a = current_state.port_a ^ last_state_.port_a;
                uint8_t changed_b = current_state.port_b ^ last_state_.port_b;
                
                if (interrupt_callback_) {
                    interrupt_callback_(current_state, changed_a, changed_b);
                }
                
                last_state_ = current_state;
            }
        }
    }
}

bool MCP23S17::configure_iocon(uint8_t config) {
    return write_register(MCP23S17_REG_IOCON, config);
}

// 私有方法实现
bool MCP23S17::write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_buffer[3];
    
    // 构建SPI命令
    tx_buffer[0] = MCP23S17_OPCODE_WRITE | (device_addr_ << 1);
    tx_buffer[1] = reg;
    tx_buffer[2] = value;
    
    return spi_transfer(tx_buffer, nullptr, 3);
}

bool MCP23S17::read_register(uint8_t reg, uint8_t& value) {
    uint8_t tx_buffer[3];
    uint8_t rx_buffer[3];
    
    // 构建SPI命令
    tx_buffer[0] = MCP23S17_OPCODE_READ | (device_addr_ << 1);
    tx_buffer[1] = reg;
    tx_buffer[2] = 0x00;  // 虚拟字节
    
    if (spi_transfer(tx_buffer, rx_buffer, 3)) {
        value = rx_buffer[2];  // 读取的数据在第三个字节
        return true;
    }
    return false;
}

bool MCP23S17::write_register_pair(uint8_t reg_a, uint8_t value_a, uint8_t reg_b, uint8_t value_b) {
    return write_register(reg_a, value_a) && write_register(reg_b, value_b);
}

bool MCP23S17::read_register_pair(uint8_t reg_a, uint8_t& value_a, uint8_t reg_b, uint8_t& value_b) {
    // 优化：使用连续寄存器读取以减少SPI事务开销
    if (reg_b == reg_a + 1) {
        uint8_t tx_buffer[4];
        uint8_t rx_buffer[4];
        
        // 构建SPI命令读取连续寄存器
        tx_buffer[0] = MCP23S17_OPCODE_READ | (device_addr_ << 1);
        tx_buffer[1] = reg_a;
        tx_buffer[2] = 0x00;  // 虚拟字节
        tx_buffer[3] = 0x00;  // 虚拟字节
        
        if (spi_transfer(tx_buffer, rx_buffer, 4)) {
            value_a = rx_buffer[2];
            value_b = rx_buffer[3];
            return true;
        }
        return false;
    } else {
        // 非连续寄存器，分别读取
        return read_register(reg_a, value_a) && read_register(reg_b, value_b);
    }
}

bool MCP23S17::spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    // 拉低CS
    gpio_put(cs_pin_, 0);
    
    // 执行SPI传输
    bool result = spi_hal_->transfer(tx_data, rx_data, length);
    
    // 拉高CS
    gpio_put(cs_pin_, 1);
    
    return result;
}

bool MCP23S17::configure_device() {
    // 配置IOCON寄存器 - 固定使用BANK=0模式以获得最高效率
    // BANK=0, MIRROR=0, SEQOP=0, DISSLW=0, HAEN=1, ODR=0, INTPOL=0
    uint8_t iocon_config = MCP23S17_IOCON_HAEN;  // 仅启用硬件地址，确保BANK=0
    
    if (!configure_iocon(iocon_config)) {
        return false;
    }
    
    // 高效批量配置：使用寄存器对写入提高效率
    bool result = true;
    result &= write_register_pair(MCP23S17_REG_IODIRA, 0xFF, MCP23S17_REG_IODIRB, 0xFF);  // 全部输入
    result &= write_register_pair(MCP23S17_REG_GPPUA, 0x00, MCP23S17_REG_GPPUB, 0x00);    // 禁用所有内部上拉，实现高阻输入
    
    return result;
}

bool MCP23S17::test_device_communication() {
    // 高效通信测试：读取IOCON寄存器默认值
    uint8_t read_value;
    if (!read_register(MCP23S17_REG_IOCON, read_value)) {
        return false;
    }
    
    // 验证读取到的值是合理的（BANK位应为0，其他位应为默认值）
    return (read_value & MCP23S17_IOCON_BANK) == 0;  // 确保BANK=0模式
}