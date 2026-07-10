#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 全局DMA中断管理模块
 * 统一管理所有DMA通道的中断处理
 * 避免在各个HAL库中重复注册DMA中断
 */

// DMA回调函数类型定义
typedef void (*dma_callback_func_t)(bool success);

// DMA通道回调信息结构体
typedef struct {
    dma_callback_func_t callback;  // 回调函数指针
    void* user_data;               // 用户数据指针（可选）
    bool active;                   // 通道是否激活
} dma_channel_info_t;

/**
 * 初始化全局DMA中断管理系统
 * 注册DMA_IRQ_0和DMA_IRQ_1中断处理函数
 * 必须在main函数中调用
 */
void global_irq_init(void);

/**
 * 反初始化全局DMA中断管理系统
 * 禁用DMA中断并清理资源
 */
void global_irq_deinit(void);

/**
 * 注册DMA通道回调函数
 * @param channel DMA通道号 (0-11)
 * @param callback 回调函数指针
 * @return true 注册成功，false 注册失败
 */
bool global_irq_register_dma_callback(uint8_t channel, dma_callback_func_t callback);

/**
 * 注销DMA通道回调函数
 * @param channel DMA通道号 (0-11)
 */
void global_irq_unregister_dma_callback(uint8_t channel);

/**
 * 检查DMA通道是否已注册回调
 * @param channel DMA通道号 (0-11)
 * @return true 已注册，false 未注册
 */
bool global_irq_is_dma_callback_registered(uint8_t channel);

/**
 * 手动触发DMA通道回调（用于测试或特殊情况）
 * @param channel DMA通道号 (0-11)
 * @param success 传递给回调函数的成功标志
 */
void global_irq_trigger_dma_callback(uint8_t channel, bool success);

#ifdef __cplusplus
}
#endif