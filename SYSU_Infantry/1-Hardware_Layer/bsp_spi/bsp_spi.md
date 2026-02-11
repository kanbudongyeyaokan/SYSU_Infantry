# BSP SPI 设备管理与通信模块使用指南
## 1. 模块功能介绍

主要功能包括：

- SPI 设备对象初始化与注册
- SPI 片选（CS）自动控制
- SPI 阻塞式数据交换接口
- SPI DMA 异步数据交换接口
- SPI 总线占用与释放管理
- DMA 传输完成中断回调分发
- 支持 SPI1 / SPI2 多总线扩展
---

## 2. 数据结构说明
### 2.1 Spi_device_t（SPI 设备对象）

每一个 SPI 外设对应一个 `Spi_device_t` 对象，内部包含：

- `SPI_HandleTypeDef *hspi`：绑定的 SPI 外设句柄（如 `&hspi1`）
- `GPIO_TypeDef *cs_port`：片选端口
- `uint16_t cs_pin`：片选引脚
- `void *context`：用户上下文指针
- `void (*callback)(void *)`：DMA 完成后的回调函数
- `uint8_t *tx_buffer`：发送缓冲区指针
- `uint8_t *rx_buffer`：接收缓冲区指针
- `uint16_t data_len`：数据长度
---
### 2.2 Spi_init_config_t（初始化配置结构体）

```c
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    void *context;
    void (*callback)(void *);
} Spi_init_config_t;
```
---
## 3. 使用流程
### 3.1 初始化 SPI 设备对象
每个 SPI 外设调用一次初始化函数：
```c
Spi_init_config_t imu_spi_cfg = {
    .hspi = &hspi1,
    .cs_port = GPIOA,
    .cs_pin = GPIO_PIN_4,
    .context = NULL,
    .callback = Imu_Spi_Callback
};

Spi_device_t *imu_spi = Spi_device_init(&imu_spi_cfg);
```
### 3.2 阻塞式 SPI 通信
```c
uint8_t tx_buf[2] = {0x80, 0x00};
uint8_t rx_buf[2];

Spi_swap_data_block(imu_spi, tx_buf, rx_buf, 2, 100);
```
支持三种模式：发送 + 接收,仅发送,仅接收
### 3.3 DMA 异步通信
```c
Spi_swap_data_dma(imu_spi, tx_buf, rx_buf, 14);
```
## 4. SPI 总线仲裁机制
流程：
    1.发起 DMA 传输时登记当前设备为 owner;
    2.DMA 完成回调中释放 owner;
    3.其他设备才能再次使用该 SPI 总线.
## 5. 使用示例
```c
void Imu_Spi_Callback(void *context)
{
    Imu_Parse_Data();
}

Spi_init_config_t imu_cfg = {
    .hspi = &hspi1,
    .cs_port = GPIOA,
    .cs_pin = GPIO_PIN_4,
    .context = NULL,
    .callback = Imu_Spi_Callback
};

Spi_device_t *imu_spi = Spi_device_init(&imu_cfg);

uint8_t tx[14], rx[14];
Spi_swap_data_dma(imu_spi, tx, rx, 14);
```