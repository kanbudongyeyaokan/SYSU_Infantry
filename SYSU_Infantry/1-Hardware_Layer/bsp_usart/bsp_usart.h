#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "usart.h"
#include "cmsis_os.h"

// 缓冲区大小 (必须是2的幂，方便优化，建议 512, 1024, 2048)
// 越大越能抗瞬间爆发的数据量，但占用 RAM
#define UART_FIFO_SIZE 1024

// 最大支持的串口实例数
#define UART_MAX_COUNT 4

// 接收回调函数类型
typedef void (*uart_receive_callback)();

// 串口实例结构体
typedef struct {
    UART_HandleTypeDef *uart_handle;

    // 发送部分 (RingBuffer)
    uint8_t  tx_fifo[UART_FIFO_SIZE]; // 环形缓冲区
    uint16_t fifo_write_pos;          // 写指针 (Head)
    uint16_t fifo_read_pos;           // 读指针 (Tail)
    volatile uint8_t is_sending;      // DMA 是否正在忙的标志位
    osMutexId fifo_mutex;             // 互斥锁，保护写指针和数据拷贝

    // 接收部分
    uint8_t  rx_buffer[128];          // 接收缓冲区 (DMA 乒乓或空闲中断用)
    uint16_t rx_buf_length;           // 接收缓冲区长度
    uint16_t rx_data_len;             // 本次接收到的实际长度
    uart_receive_callback receive_callback; // 接收回调

} Uart_instance_t;

// 注册串口
Uart_instance_t* Uart_register(UART_HandleTypeDef *huart, uart_receive_callback cb);

// 格式化打印 (非阻塞)
void Uart_printf(Uart_instance_t *inst, const char* fmt, ...);

// 发送数据 (非阻塞)
void Uart_sendData(Uart_instance_t *inst, uint8_t* data, uint16_t length);


#endif // _BSP_USART_H