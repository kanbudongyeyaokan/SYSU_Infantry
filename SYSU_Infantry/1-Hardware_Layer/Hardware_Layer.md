# Hardware Layer 硬件抽象层说明

硬件抽象层位于 `1-Hardware_Layer/`，也就是 BSP 层。它负责把 STM32 内部外设封装成相对稳定的接口，让设备驱动层和功能模块不直接依赖 HAL 细节。

这一层只处理“MCU 怎么收发数据、怎么计时、怎么管理外设实例”，不处理某个具体设备的业务含义。

## 目录总览

| 目录 | 主要接口 | 职责 |
| --- | --- | --- |
| `bsp_can` | `Can_init`、`Can_device_init`、`Can_send_data` | CAN 过滤器、设备注册、ID 查表分发、发送 |
| `bsp_usart` | `Uart_register`、`Uart_sendData`、`Uart_printf` | UART DMA 空闲接收、TX FIFO 非阻塞发送 |
| `bsp_spi` | `Spi_device_init`、`Spi_swap_data_block`、`Spi_swap_data_dma` | SPI 设备对象、片选管理、阻塞/DMA 交换 |
| `bsp_iic` | `HAL_I2C_MemRxCpltCallback` | I2C DMA 回调分发入口 |
| `bsp_usb` | `Usb_Init`、`Usb_Send`、`Usb_Printf` | USB CDC 虚拟串口收发 |
| `bsp_dwt` | `DWT_Init`、`DWT_GetDeltaT`、`DWT_GetTimeline_s` | Cortex-M DWT 高精度计时 |
| `bsp_wdg` | `Watchdog_register`、`Watchdog_feed`、`Watchdog_control_all` | 软件看门狗和在线状态检测 |
| `bsp_rtt` | RTT 相关封装 | SEGGER RTT 调试输出 |

## 初始化顺序

主流程位于 `Core/Src/main.c`：

```text
HAL_Init()
SystemClock_Config()
MX_GPIO/DMA/ADC/CAN/TIM/I2C/SPI/USART/... 初始化
Can_init()
DWT_Init(168)
HAL_TIM_PWM_Start(...)
MX_USB_DEVICE_Init()
MX_FREERTOS_Init()
osKernelStart()
```

注意：

- `Can_init()` 必须早于 DJI 电机、功率计、超级电容等 CAN 设备注册后的正常通信。
- `DWT_Init()` 必须早于 PID、INS、视觉时间戳等使用 DWT 的模块。
- USB CDC 由 CubeMX 生成的 `USB_DEVICE` 与 `bsp_usb` 协作使用。

## CAN BSP

接口：

```c
void Can_init(void);
Can_controller_t *Can_device_init(Can_init_t *can_config);
uint8_t Can_send_data(Can_controller_t *Can_dev, uint8_t *tx_buff);
```

实现特点：

- `Can_init()` 配置全局过滤器并启动 CAN1/CAN2。
- CAN1 使用 FIFO0，CAN2 当前配置到 FIFO1，避免回调冲突。
- 设备注册后进入快速查找表 `can1_rx_lut/can2_rx_lut`。
- 接收中断中按 `StdId` 直接查表，命中后复制 8 字节数据并调用设备回调。
- 发送失败、邮箱满、接收失败都会通过错误系统限频上报。

设备驱动注册示例：

```c
Can_init_t cfg = {
    .can_handle = &hcan1,
    .can_id = 0x200,
    .tx_id = 1,
    .rx_id = 0x201,
    .receive_callback = Decode_djimotor,
    .context = motor,
};

Can_controller_t *can = Can_device_init(&cfg);
```

## USART BSP

接口：

```c
Uart_instance_t *Uart_register(UART_HandleTypeDef *huart, uart_receive_callback cb);
void Uart_sendData(Uart_instance_t *inst, uint8_t *data, uint16_t length);
void Uart_printf(Uart_instance_t *inst, const char *fmt, ...);
```

实现特点：

- TX 使用 `UART_FIFO_SIZE` 环形缓冲区，默认 1024 字节。
- 发送使用 DMA，调用方无需阻塞等待。
- RX 使用 `HAL_UARTEx_ReceiveToIdle_DMA()`，收到空闲中断后记录 `rx_data_len` 并调用用户回调。
- 使用互斥锁保护多任务写 FIFO。
- UART 错误回调中会自动重启 DMA 接收。

使用建议：

- 注册串口后保存 `Uart_instance_t *`，不要直接操作 HAL DMA。
- 接收回调中只做轻量解析或入队，不要做耗时逻辑。
- `Uart_printf` 内部使用约 256 字节栈空间，调用任务栈不要过小。

## SPI BSP

接口：

```c
Spi_device_t *Spi_device_init(Spi_init_config_t *config);
HAL_StatusTypeDef Spi_swap_data_block(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef Spi_swap_data_dma(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
```

设计目标：

- 为 BMI088 等 SPI 设备管理总线句柄和片选引脚。
- 阻塞接口用于初始化阶段寄存器读写。
- DMA 接口用于周期性传感器读取。

DMA 注意事项：`tx_data` 和 `rx_data` 必须指向静态或全局内存，不要传入栈上临时数组。

## I2C BSP

当前 `bsp_iic` 主要提供全局 DMA 接收完成回调分发入口。HWT101/HWT606 等 I2C IMU 驱动通过这一层接收 HAL 回调。

如果新增 I2C 设备，需要注意：

- 不要在多个驱动里重复定义 HAL 弱回调。
- 在 BSP 回调中按 `I2C_HandleTypeDef *` 分发到具体驱动。
- DMA 模式下同一条 I2C 总线要避免多设备同时发起传输。

## USB BSP

接口：

```c
void Usb_Init(usb_rx_callback cb);
void Usb_Send(uint8_t *data, uint16_t len);
void Usb_Printf(const char *fmt, ...);
```

当前视觉通信模块在 `vision_comm.h` 中定义 `USE_VISION_USB`，默认通过 USB CDC 与视觉主机通信。

`bsp_usb` 使用 FIFO 非阻塞发送，收发完成钩子由 `USB_DEVICE/App/usbd_cdc_if.c` 调用。

## DWT 计时

接口：

```c
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
void DWT_Delay_us(uint32_t delay_us);
```

用途：

- PID 控制器自动计算 `dt`。
- INS 更新计算采样间隔。
- 视觉通信发送微秒级时间戳。
- 短时间精确延时。

使用前必须初始化，当前主频为 168 MHz。

## 软件看门狗

接口：

```c
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config);
void Watchdog_feed(Watchdog_device_t *instance);
void Watchdog_control_all(void);
uint8_t Watchdog_is_online(Watchdog_device_t *instance);
```

工作方式：

1. 设备驱动注册看门狗并设置 `reload_count`、离线回调和名称。
2. 收到设备数据时调用 `Watchdog_feed()`。
3. `Watchdog_control_task` 周期调用 `Watchdog_control_all()`。
4. 计数耗尽时触发离线回调。

当前 DJI 电机驱动、遥控器解析等模块都使用该机制做在线检测。

## 新增 BSP 建议

新增 BSP 时请遵守：

1. 只封装 MCU 内部外设，不写具体机器人业务。
2. 给每个可注册对象保留 `context` 指针和回调接口，方便设备驱动挂载。
3. 高频错误必须限频上报。
4. 与 HAL 弱回调相关的函数只能有一个统一入口。
5. 写清楚初始化时机和是否可在 ISR 中调用。
