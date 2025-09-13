#include "bsp_usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stdbool.h"
#include "bsp_dwt.h"

//串口实例序号
static uint8_t uart_ix = 0;
static UartInstance_t* uart_instance[UART_MAX_COUNT] = {0};

//初始化串口实例
static void Uart_init(UartInstance_t* instance,UART_HandleTypeDef *huart) {
    //安全检查
    if (instance == NULL || huart == NULL) {
        return;
    }
    //正常初始化
    memset(instance,0,sizeof(UartInstance_t));
    instance->uart_handle = huart;
    instance->rx_buf_length = RX_BUF_SIZE;
    //接收初始化
    HAL_UARTEx_ReceiveToIdle_DMA(instance->uart_handle, instance->rx_buffer, instance->rx_buf_length);
    //关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(instance->uart_handle->hdmarx, DMA_IT_HT);
}
//串口注册
UartInstance_t* Uart_register(UART_HandleTypeDef *register_huart,uart_receive_callback receive_callback)
{
    //安全检查
    if (uart_ix >= UART_MAX_COUNT) // 超过最大实例数
        return NULL;
    for (uint8_t i = 0; i < uart_ix; i++) // 检查是否已经注册过
        if (uart_instance[i]->uart_handle == register_huart)
            return NULL;
    //正常，进行注册
    UartInstance_t *instance = (UartInstance_t *)malloc(sizeof(UartInstance_t));
    memset(instance, 0, sizeof(UartInstance_t));
    Uart_init(instance,register_huart);
    instance->receive_callback = receive_callback;
    //记录该串口实例
    uart_instance[uart_ix++] = instance;
    return instance;
}

//打印调试信息
void Uart_printf(UartInstance_t *uart_instance,const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char*)uart_instance->tx_buffer, TX_BUF_SIZE, fmt, args);
    va_end(args);

    uint8_t wait_time = HAL_GetTick();
    while (HAL_GetTick() - wait_time < 1) {
    };

    //发送调试字符串信息
    if (len > 0) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buffer, len);
    }
}

//发送数据/数据包
void Uart_sendData(UartInstance_t *uart_instance,uint8_t* data,uint16_t length) {
    // 长度限制
    length = (length > TX_BUF_SIZE) ? TX_BUF_SIZE : length;
    //发送数据
    memcpy(uart_instance->tx_buffer, data, length);
    HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buffer, length);
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数据量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    //检索已经注册的串口实例，调用回调函数并开启DMA空闲中断接收
    for (uint8_t i = 0; i < uart_ix; ++i)
    {
        if (huart == uart_instance[i]->uart_handle)
        {
            if (uart_instance[i]->receive_callback != NULL)
            {
                uart_instance[i]->receive_callback();
                memset(uart_instance[i]->rx_buffer, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instance[i]->uart_handle,
                uart_instance[i]->rx_buffer, uart_instance[i]->rx_buf_length);
            //禁用DMA半传输中断
            __HAL_DMA_DISABLE_IT(uart_instance[i]->uart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}
