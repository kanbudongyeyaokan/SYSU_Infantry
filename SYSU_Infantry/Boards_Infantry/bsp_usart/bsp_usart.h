/**
******************************************************************************
 * @file	bsp_usart.h
 * @brief   串口库，提供使用DMA来发送，接收数据，数据包通信等功能
 *          不使用面向对象封装，使用起来更为简洁。
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include <stdarg.h>
#include "usart.h"


// 串口通信协议定义，后续改为CRC校验
#pragma pack(push, 1)
typedef struct {
    uint8_t header;     // 0xA5
    uint16_t cx;        // 黄色色块X坐标
    uint16_t cy;        // 黄色色块Y坐标
    uint8_t checksum;   // 固定为0x00
    uint8_t tail;       // 0x5A
} packet;
#pragma pack(pop)

// 初始化函数
void USART_Init(void);

// 核心功能函数
void USART_Printf(UART_HandleTypeDef* huart, const char* fmt, ...);
void USART_SendData(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);
void USART_SendPacket(UART_HandleTypeDef* huart, packet* frame);

/*
使用手册：
1.如要使用多个串口需要定义该串口对应的发送/接收缓冲区
// 串口1资源
 uint8_t uart1_tx_buf[TX_BUF_SIZE];
 uint8_t uart1_rx_buf[RX_BUF_SIZE];
 volatile bool uart1_tx_busy = false;
并在USART_Printf和USART_Send函数增加对应的分支
2.接收：需要main函数外部引用对应的接收缓冲区，并手动调用HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, RX_BUF_SIZE);用于接收
3.发送：直接调用USART_Printf和USART_Sendpacket即可，数据包可以自行定义，目前只支持发单包，若要发多包，需要自行增加函数。

*/

#endif