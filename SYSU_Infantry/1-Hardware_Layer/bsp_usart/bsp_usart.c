#include "bsp_usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"

// 管理所有注册的串口实例
static Uart_instance_t* uart_instances[UART_MAX_COUNT] = {NULL};
static uint8_t uart_cnt = 0;

static Uart_instance_t uart_instances_pool[UART_MAX_COUNT];

// ============================================================
// 核心内部函数：尝试启动 DMA 发送
// ============================================================
static void Uart_Try_Transmit(Uart_instance_t *inst)
{
    if (inst->is_sending) {
        return;
    }
    // 检查缓冲区是否为空 (读指针 == 写指针)
    if (inst->fifo_read_pos == inst->fifo_write_pos) {
        return;
    }

    // 计算本次 DMA 需要发送的长度
    // RingBuffer 可能发生回绕 (Wrap Around)
    // 情况 A: [ ... Tail ... Head ... ]  -> 发送长度 = Head - Tail
    // 情况 B: [ ... Head ... Tail ... ]  -> 先发送 Tail 到 End，中断后再发 0 到 Head
    uint16_t head = inst->fifo_write_pos;
    uint16_t tail = inst->fifo_read_pos;
    uint16_t send_len = 0;

    if (head > tail) {
        // 线性段，未回绕
        send_len = head - tail;
    } else {
        // 回绕了，先发尾巴那一段
        send_len = UART_FIFO_SIZE - tail;
    }

    // 启动 DMA 发送
    // 标记忙状态，防止其他任务再次触发
    inst->is_sending = 1;

    if (HAL_UART_Transmit_DMA(inst->uart_handle, &inst->tx_fifo[tail], send_len) != HAL_OK) {
        // 如果发送失败，清除标志位让下次重试
        inst->is_sending = 0;
    }
}

// ============================================================
// 初始化与注册
// ============================================================
static void Uart_init(Uart_instance_t* inst, UART_HandleTypeDef *huart) {
    if(!inst || !huart) return;
    memset(inst, 0, sizeof(Uart_instance_t));

    inst->uart_handle = huart;
    inst->rx_buf_length = 128; // 接收缓冲区大小，与 rx_buffer[128] 保持一致

    // 创建互斥锁
    osMutexDef(uart_mutex);
    inst->fifo_mutex = osMutexCreate(osMutex(uart_mutex));

    // 启动空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(inst->uart_handle, inst->rx_buffer, inst->rx_buf_length);
    __HAL_DMA_DISABLE_IT(inst->uart_handle->hdmarx, DMA_IT_HT); // 关闭半传输中断
}

Uart_instance_t* Uart_register(UART_HandleTypeDef *huart, uart_receive_callback cb) {
    if (uart_cnt >= UART_MAX_COUNT) return NULL;

    // 查重
    for(int i=0; i<uart_cnt; i++) {
        if (uart_instances[i]->uart_handle == huart) return uart_instances[i];
    }

    Uart_instance_t *inst = &uart_instances_pool[uart_cnt];
    
    if (inst == NULL) return NULL;

    Uart_init(inst, huart);
    inst->receive_callback = cb;
    uart_instances[uart_cnt++] = inst;

    return inst;
}

// ============================================================
// 发送接口
// ============================================================

// 发送二进制数据
void Uart_sendData(Uart_instance_t *inst, uint8_t* data, uint16_t length)
{
    if (!inst || !data || length == 0) return;

    // 获取锁：保证多任务写 FIFO 时指针不会乱
    if (osMutexWait(inst->fifo_mutex, 10) != osOK) return;

    // 检查剩余空间是否足够
    uint16_t free_space = 0;
    if (inst->fifo_read_pos > inst->fifo_write_pos) {
        free_space = inst->fifo_read_pos - inst->fifo_write_pos - 1;
    } else {
        free_space = (UART_FIFO_SIZE - inst->fifo_write_pos) + inst->fifo_read_pos - 1;
    }
    if (length > free_space) {
        // 空间不足，放弃发送
        osMutexRelease(inst->fifo_mutex);
        return;
    }

    // 拷贝数据到 FIFO (处理回绕)
    for (uint16_t i = 0; i < length; i++) {
        inst->tx_fifo[inst->fifo_write_pos] = data[i];
        inst->fifo_write_pos++;

        // 处理回绕
        if (inst->fifo_write_pos >= UART_FIFO_SIZE) {
            inst->fifo_write_pos = 0;
        }
    }
    // 4. 释放锁
    osMutexRelease(inst->fifo_mutex);

    // 尝试触发发送 (在临界区保护下检查，防止中断竞争)
    // 这里的 taskENTER_CRITICAL 是为了防止 Uart_Try_Transmit 执行判断到一半被中断打断
    taskENTER_CRITICAL();
    Uart_Try_Transmit(inst);
    taskEXIT_CRITICAL();
}

// 格式化打印
void Uart_printf(Uart_instance_t *inst, const char* fmt, ...)
{
    if (!inst) return;

    // 使用任务栈上的临时 buffer
    // 注意：确保任务堆栈够大 (建议 > 512 Bytes)
    char temp_buf[256];

    va_list args;
    va_start(args, fmt);
    // vsnprintf 安全地格式化字符串，防止溢出
    int len = vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);
    va_end(args);

    if (len > 0) {
        // 调用通用发送函数
        Uart_sendData(inst, (uint8_t*)temp_buf, (uint16_t)len);
    }
}

// ============================================================
// 中断回调函数
// ============================================================

// 发送完成回调 (DMA发完一段后触发)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < uart_cnt; ++i)
    {
        if (huart == uart_instances[i]->uart_handle)
        {
            Uart_instance_t *inst = uart_instances[i];

            // 标记为空闲
            inst->is_sending = 0;

            // 更新读指针 (Tail)
            // huart->TxXferSize 记录了刚才 DMA 实际请求发送的长度
            inst->fifo_read_pos += huart->TxXferSize;

            // 处理回绕
            if (inst->fifo_read_pos >= UART_FIFO_SIZE) {
                inst->fifo_read_pos -= UART_FIFO_SIZE; // 归位
            }

            // 继续尝试发送剩余数据 (如果有的话)
            Uart_Try_Transmit(inst);

            return;
        }
    }
}

// 接收回调 (Idle 中断)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < uart_cnt; ++i)
    {
        if (huart == uart_instances[i]->uart_handle)
        {
            uart_instances[i]->rx_data_len = Size; // 记录长度

            if (uart_instances[i]->receive_callback != NULL)
            {
                uart_instances[i]->receive_callback();
            }

            // 重新开启接收
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instances[i]->uart_handle,
                uart_instances[i]->rx_buffer, uart_instances[i]->rx_buf_length);
            __HAL_DMA_DISABLE_IT(uart_instances[i]->uart_handle->hdmarx, DMA_IT_HT);

            return;
        }
    }
}