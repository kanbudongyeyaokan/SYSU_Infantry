#ifndef _BSP_RTT_H
#define _BSP_RTT_H

#include <stdint.h>

/**
 * @brief 极致极简版 RTT 打印函数 (随处可用，自动初始化，支持浮点)
 * @param channel 端口号 (0: 默认终端, 1~3: 自定义通道，如VOFA等)
 * @param fmt 格式化字符串，用法和 printf 完全一样
 */
void Rtt_Printf(uint8_t channel, const char *fmt, ...);

#endif // _BSP_RTT_H