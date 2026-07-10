#include "global_irq.h"
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <string.h>

// DMA通道最大数量（RP2040有12个DMA通道）
#define MAX_DMA_CHANNELS 12

// 全局DMA通道回调信息数组
static dma_channel_info_t dma_channels[MAX_DMA_CHANNELS];

// 初始化标志
static bool global_irq_initialized = false;

/**
 * DMA_IRQ_0中断处理函数
 * 处理DMA通道0-7的中断
 */
static void dma_irq0_handler(void) {
    // 读取中断状态寄存器
    uint32_t ints = dma_hw->ints0;
    
    // 遍历通道0-7
    for (uint8_t channel = 0; channel < 8; channel++) {
        if (ints & (1u << channel)) {
            // 清除中断标志
            dma_hw->ints0 = 1u << channel;
            
            // 检查通道是否已注册回调
            if (dma_channels[channel].active && dma_channels[channel].callback) {
                // 检查DMA传输是否成功完成
                bool success = !dma_channel_is_busy(channel);
                
                // 调用回调函数
                dma_channels[channel].callback(success);
            }
        }
    }
}

/**
 * DMA_IRQ_1中断处理函数
 * 处理DMA通道8-11的中断
 */
static void dma_irq1_handler(void) {
    // 读取中断状态寄存器
    uint32_t ints = dma_hw->ints1;
    
    // 遍历通道8-11
    for (uint8_t channel = 8; channel < MAX_DMA_CHANNELS; channel++) {
        uint8_t bit_pos = channel - 8;
        if (ints & (1u << bit_pos)) {
            // 清除中断标志
            dma_hw->ints1 = 1u << bit_pos;
            
            // 检查通道是否已注册回调
            if (dma_channels[channel].active && dma_channels[channel].callback) {
                // 检查DMA传输是否成功完成
                bool success = !dma_channel_is_busy(channel);
                
                // 调用回调函数
                dma_channels[channel].callback(success);
            }
        }
    }
}

void global_irq_init(void) {
    if (global_irq_initialized) {
        return; // 已经初始化过了
    }
    
    // 清零所有通道信息
    memset(dma_channels, 0, sizeof(dma_channels));
    
    // 注册DMA中断处理函数
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_irq1_handler);
    
    // 启用DMA中断
    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_enabled(DMA_IRQ_1, true);
    
    global_irq_initialized = true;
}

void global_irq_deinit(void) {
    if (!global_irq_initialized) {
        return;
    }
    
    // 禁用DMA中断
    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(DMA_IRQ_1, false);
    
    // 清零所有通道信息
    memset(dma_channels, 0, sizeof(dma_channels));
    
    global_irq_initialized = false;
}

bool global_irq_register_dma_callback(uint8_t channel, dma_callback_func_t callback) {
    // 检查参数有效性
    if (channel >= MAX_DMA_CHANNELS || callback == NULL) {
        return false;
    }
    
    // 确保全局中断系统已初始化
    if (!global_irq_initialized) {
        global_irq_init();
    }
    
    // 注册回调信息
    dma_channels[channel].callback = callback;
    dma_channels[channel].user_data = NULL;
    dma_channels[channel].active = true;
    
    // 为对应的DMA通道启用中断
    if (channel < 8) {
        // 通道0-7使用DMA_IRQ_0
        dma_channel_set_irq0_enabled(channel, true);
    } else {
        // 通道8-11使用DMA_IRQ_1
        dma_channel_set_irq1_enabled(channel, true);
    }
    
    return true;
}

void global_irq_unregister_dma_callback(uint8_t channel) {
    // 检查参数有效性
    if (channel >= MAX_DMA_CHANNELS) {
        return;
    }
    
    // 禁用对应的DMA通道中断
    if (channel < 8) {
        // 通道0-7使用DMA_IRQ_0
        dma_channel_set_irq0_enabled(channel, false);
    } else {
        // 通道8-11使用DMA_IRQ_1
        dma_channel_set_irq1_enabled(channel, false);
    }
    
    // 清除回调信息
    dma_channels[channel].callback = NULL;
    dma_channels[channel].user_data = NULL;
    dma_channels[channel].active = false;
}

bool global_irq_is_dma_callback_registered(uint8_t channel) {
    if (channel >= MAX_DMA_CHANNELS) {
        return false;
    }
    
    return dma_channels[channel].active && (dma_channels[channel].callback != NULL);
}

void global_irq_trigger_dma_callback(uint8_t channel, bool success) {
    // 检查参数有效性
    if (channel >= MAX_DMA_CHANNELS) {
        return;
    }
    
    // 检查通道是否已注册回调
    if (dma_channels[channel].active && dma_channels[channel].callback) {
        // 调用回调函数
        dma_channels[channel].callback(success);
    }
}