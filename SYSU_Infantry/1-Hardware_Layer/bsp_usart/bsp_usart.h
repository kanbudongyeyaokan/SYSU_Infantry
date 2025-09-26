#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "usart.h"

//发送/接收缓冲区大小
#define TX_BUF_SIZE 128
#define RX_BUF_SIZE 128

//串口实例数量
#define UART_MAX_COUNT 4


// 模块回调函数,用于解析协议
typedef void (*uart_receive_callback)();

//串口结构体
typedef struct {
    UART_HandleTypeDef *uart_handle;//串口句柄
    uint8_t tx_buffer[TX_BUF_SIZE];
    uint8_t rx_buffer[RX_BUF_SIZE];
    uint16_t rx_buf_length;//接收缓冲区长度
    uart_receive_callback receive_callback;
}Uart_instance_t;

//串口注册
Uart_instance_t* Uart_register(UART_HandleTypeDef *register_huart,uart_receive_callback receive_callback);

//打印调试信息
void Uart_printf(Uart_instance_t *uart_instance,const char* fmt, ...);

//发送数据/数据包
void Uart_sendData(Uart_instance_t *uart_instance,uint8_t* data,uint16_t length);

// 全局调试串口实例
extern Uart_instance_t* debug_uart;


#endif //_BSP_USART_H