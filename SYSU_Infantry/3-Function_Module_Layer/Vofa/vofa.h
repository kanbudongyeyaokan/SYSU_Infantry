/**
 * @file    vofa.h
 * @brief   VOFA+ 示波器 FireWater 协议便捷发送库
 * @author  SYSU电控组 (Based on User Request)
 * @date    2025-02-01
 * @note    支持 1-8 个变量自动识别，自动格式化为 "float, float\n"
 */

#ifndef _BSP_VOFA_H
#define _BSP_VOFA_H

#include "bsp_usart.h" // 确保包含你的串口库，以便调用 Uart_printf

// ============================================================
//   底层宏定义 (请勿直接调用，使用下方的 VOFA_Send)
// ============================================================

// 1个变量
#define _VOFA_SEND_1(uart, v1) \
    Uart_printf(uart, "%.2f\n", (float)(v1))

// 2个变量
#define _VOFA_SEND_2(uart, v1, v2) \
    Uart_printf(uart, "%.2f,%.2f\n", (float)(v1), (float)(v2))

// 3个变量
#define _VOFA_SEND_3(uart, v1, v2, v3) \
    Uart_printf(uart, "%.2f,%.2f,%.2f\n", (float)(v1), (float)(v2), (float)(v3))

// 4个变量
#define _VOFA_SEND_4(uart, v1, v2, v3, v4) \
    Uart_printf(uart, "%.2f,%.2f,%.2f,%.2f\n", (float)(v1), (float)(v2), (float)(v3), (float)(v4))

// 5个变量
#define _VOFA_SEND_5(uart, v1, v2, v3, v4, v5) \
    Uart_printf(uart, "%.2f,%.2f,%.2f,%.2f,%.2f\n", (float)(v1), (float)(v2), (float)(v3), (float)(v4), (float)(v5))

// 6个变量 (预留)
#define _VOFA_SEND_6(uart, v1, v2, v3, v4, v5, v6) \
    Uart_printf(uart, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", (float)(v1), (float)(v2), (float)(v3), (float)(v4), (float)(v5), (float)(v6))

// ============================================================
//   宏重载选择器 (Magic Macros)
// ============================================================

// 这个宏用于计算参数个数，并返回对应的宏名称
#define _GET_VOFA_MACRO(_1, _2, _3, _4, _5, _6, NAME, ...) NAME

// ============================================================
//   用户调用接口
// ============================================================

/**
 * @brief VOFA+ FireWater协议 发送函数
 * @param uart 串口实例指针 (如 test_uart)
 * @param ...  需要查看的变量 (1到6个), 支持整型、浮点型
 * * @example VOFA_Send(test_uart, target, measure);
 * @example VOFA_Send(test_uart, target, measure, output, current);
 */
#define VOFA_Send(uart, ...) \
    _GET_VOFA_MACRO(__VA_ARGS__, \
                    _VOFA_SEND_6, \
                    _VOFA_SEND_5, \
                    _VOFA_SEND_4, \
                    _VOFA_SEND_3, \
                    _VOFA_SEND_2, \
                    _VOFA_SEND_1)(uart, __VA_ARGS__)


/**
 * @brief VOFA+ 带标签发送 (格式: "label:ch0,ch1\n")
 * @param uart  串口实例指针
 * @param label 字符串标签 (不要带冒号)
 * @param ...   变量列表
 */
#define VOFA_Send_Label(uart, label, ...) do { \
    Uart_printf(uart, "%s:", label);           \
    VOFA_Send(uart, __VA_ARGS__);              \
} while(0)

#endif // _BSP_VOFA_H