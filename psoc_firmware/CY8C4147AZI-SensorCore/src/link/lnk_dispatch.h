/*******************************************************************************
 * lnk_dispatch.h —— 信封 -> 业务
 *
 * 处于业务层之上: 它包含各业务模块的头文件, 反过来没有任何模块包含它(线级只按声明回调
 * lnk_deliver)。命令 switch、tag 去重、响应入队都在这里, 与线级彻底分开。
 ******************************************************************************/
#ifndef LNK_DISPATCH_H
#define LNK_DISPATCH_H

#include <stdint.h>

/* 线级取到一整合法帧后的投递入口(lnk_wire.h 里以回调声明给出同一个符号)。 */
void lnk_deliver(const uint8_t * rx);
/* 启动时清空去重缓存。 */
void lnk_dispatch_reset(void);

#endif /* LNK_DISPATCH_H */
