# BSP USART 串口通信模块使用指南
## 1. 模块功能介绍

主要功能包括：

- 串口实例注册与初始化
- DMA 异步发送数据
- FIFO 缓冲区管理
- 支持二进制数据发送接口
- 支持格式化字符串打印接口
- 基于 Idle 中断的接收机制
- 接收完成回调函数通知上层
- 支持多 UART 外设（USART1/2/3 等）
---

## 2. 数据结构说明
每一个 UART 对应一个 `Uart_instance_t` 实例，主要包含：

- `UART_HandleTypeDef *uart_handle`：绑定的 UART 句柄
- `uint8_t tx_fifo[UART_FIFO_SIZE]`：发送 FIFO 缓冲区
- `uint16_t fifo_write_pos`：FIFO 写指针（Head）
- `uint16_t fifo_read_pos`：FIFO 读指针（Tail）
- `uint8_t is_sending`：当前是否正在 DMA 发送
- `osMutexId fifo_mutex`：FIFO 互斥锁（线程安全）
- `uint8_t rx_buffer[]`：接收缓冲区
- `uint16_t rx_buf_length`：接收缓冲区长度
- `uint16_t rx_data_len`：本次接收的数据长度
- `uart_receive_callback receive_callback`：接收完成回调函数
---

## 3. 使用流程

### 3.1 注册串口实例

在系统初始化阶段注册串口实例：

```c
void Uart1_Receive_Callback(void);

Uart_instance_t *uart1 = Uart_register(&huart1, Uart1_Receive_Callback);
```
自动分配并初始化串口实例对象,启动 Idle + DMA 接收,注册接收回调函数,支持最多 UART_MAX_COUNT 个串口实例

### 3.2 发送二进制数据
```c
uint8_t data[] = {0x01, 0x02, 0x03};
Uart_sendData(uart1, data, sizeof(data));
```

### 3.3 格式化字符串发送
```c
Uart_printf(uart1, "Roll=%.2f Pitch=%.2f\r\n", roll, pitch);
```

### 3.4 接收数据处理
```c
void Uart1_Receive_Callback(void)
{
    uint16_t len = uart1->rx_data_len;
    uint8_t *buf = uart1->rx_buffer;

    //Parse_Command(buf, len);
}
```
## 4. DMA 发送机制说明
### 4.1 FIFO 发送流程

用户调用 Uart_sendData,数据写入 FIFO,调用 Uart_Try_Transmit,若 UART 空闲，启动 DMA。
DMA 完成触发 HAL_UART_TxCpltCallback,更新 FIFO 读指针,若 FIFO 仍有数据，继续发送
实现连续不间断发送。

### 4.2 FIFO 回绕处理

支持 RingBuffer 回绕：
Head > Tail：线性发送
Head < Tail：分两段发送
保证 FIFO 连续输出。

## 5.使用示例
```c
void Uart1_Callback(void)
{
    Process_PC_Command(uart1->rx_buffer, uart1->rx_data_len);
}

Uart_instance_t *uart1 = Uart_register(&huart1, Uart1_Callback);

Uart_printf(uart1, "System Init OK\r\n");

uint8_t msg[] = {0xAA, 0x55};
Uart_sendData(uart1, msg, 2);
```
