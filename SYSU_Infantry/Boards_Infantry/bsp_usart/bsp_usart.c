/**
******************************************************************************
 * @file	bsp_usart.c
 * @brief   串口库，提供使用DMA来发送/接收数据以及数据包通信等功能
 *          不使用面向对象封装，使用起来更为简洁。
 */
#include "bsp_usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "usart.h"

#define TX_BUF_SIZE 128
#define RX_BUF_SIZE 128

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

// 串口1缓冲区
 uint8_t uart1_tx_buf[TX_BUF_SIZE];
 uint8_t uart1_rx_buf[RX_BUF_SIZE];
 volatile bool uart1_tx_busy = false;

// 串口6缓冲区
 uint8_t uart6_tx_buf[TX_BUF_SIZE];
 uint8_t uart6_rx_buf[RX_BUF_SIZE];
volatile bool uart6_tx_busy = false;

// 初始化两个串口的DMA接收
void USART_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buf, RX_BUF_SIZE);
}

/* printf函数 */
void USART_Printf(UART_HandleTypeDef* huart, const char* fmt, ...) {
    volatile bool* tx_busy = NULL;
    uint8_t* tx_buf = NULL;

    if (huart->Instance == USART1) {
        tx_busy = &uart1_tx_busy;
        tx_buf = uart1_tx_buf;
    } else if (huart->Instance == USART6) {
        tx_busy = &uart6_tx_busy;
        tx_buf = uart6_tx_buf;
    } else {
        return;
    }

    // 检查发送状态
    if (*tx_busy) return;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char*)tx_buf, TX_BUF_SIZE, fmt, args);
    va_end(args);

    if (len > 0) {
        *tx_busy = true;
        HAL_UART_Transmit_DMA(huart, tx_buf, len);
    }
}

/* DMA数据发送 */
void USART_SendData(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    volatile bool* tx_busy = NULL;
    uint8_t* tx_buf = NULL;

    if (huart->Instance == USART1) {
        tx_busy = &uart1_tx_busy;
        tx_buf = uart1_tx_buf;
    } else if (huart->Instance == USART6) {
        tx_busy = &uart6_tx_busy;
        tx_buf = uart6_tx_buf;
    } else {
        return;
    }

    // 长度限制
    len = (len > TX_BUF_SIZE) ? TX_BUF_SIZE : len;

    // 检查发送状态
    if (*tx_busy) return;

    memcpy(tx_buf, data, len);
    *tx_busy = true;
    HAL_UART_Transmit_DMA(huart, tx_buf, len);
}

/* 发送数据包*/
void USART_SendPacket(UART_HandleTypeDef* huart, packet* frame) {
    // 填充固定协议头尾
    frame->header = 0xA5;
    frame->tail = 0x5A;
    frame->checksum = 0x00; // 固定校验值

    USART_SendData(huart, (uint8_t*)frame, sizeof(packet));
}

/* 发送完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        uart1_tx_busy = false;
    } else if (huart->Instance == USART6) {
        uart6_tx_busy = false;
    }
}

/* 精简接收处理 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {

    if(huart == &huart1){
         //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
    // 协议处理 (单包检测)
    if (size == sizeof(packet)) {
        packet* frame = (packet*)uart1_rx_buf;

        // 仅校验头尾 (免校验和计算)
        if (frame->header == 0xA5 && frame->tail == 0x5A&&frame->checksum == 0x00) {
            // 这里添加您的应用处理代码


           // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, RX_BUF_SIZE);
    }
    /*另一种解包方式，先找帧头
 if(huart == &huart1) {
        // 遍历整个接收缓冲区寻找帧头
        for(int i = 0; i <= (size - sizeof(JY_Frame)); i++) {
            // 检查帧头0x55和类型0x53
            if(uart1_rx_buf[i] == 0x55 && uart1_rx_buf[i+1] == 0x53) {
                JY_Frame* frame = (JY_Frame*)(uart1_rx_buf + i);

                // 验证校验和
                if(JY_CalculateChecksum(frame) == frame->checksum) {
                    Get_Angle(frame, &jy_gyro1);
                }
                break; // 处理完一个帧后跳出
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, RX_BUF_SIZE);
        */
}