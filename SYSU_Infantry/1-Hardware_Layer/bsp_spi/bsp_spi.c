#include "bsp_spi.h"
#include <string.h>

// --- 静态内存池---
static Spi_device_t spi_devices_pool[SPI_DEVICE_MAX_COUNT];
static uint8_t spi_dev_idx = 0;

// --- 总线仲裁 ---
// 记录当前 SPI1 和 SPI2 正在被哪个设备对象独占
// NULL 表示总线空闲
static Spi_device_t* current_spi1_owner = NULL;
static Spi_device_t* current_spi2_owner = NULL;

// 引用外部 SPI 句柄
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


static inline void Spi_cs_enable(Spi_device_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void Spi_cs_disable(Spi_device_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// ================= SPI通信接口 =================

Spi_device_t* Spi_device_init(Spi_init_config_t *config)
{
    if (spi_dev_idx >= SPI_DEVICE_MAX_COUNT) return NULL;
    
    // 从池中分配一个对象
    Spi_device_t *dev = &spi_devices_pool[spi_dev_idx++];
    
    // 绑定参数
    dev->hspi     = config->hspi;
    dev->cs_port  = config->cs_port;
    dev->cs_pin   = config->cs_pin;
    dev->context  = config->context;
    dev->callback = config->callback;
    
    // 初始化 CS 引脚状态 (默认拉高，不选中)
    Spi_cs_disable(dev);
    
    return dev;
}

HAL_StatusTypeDef Spi_swap_data_block(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t timeout_ms)
{
    if (!dev) return HAL_ERROR;

    // 拉低片选
    Spi_cs_enable(dev);

    // 执行交换
    HAL_StatusTypeDef status;
    
    if (tx_data != NULL && rx_data != NULL)
    {
        // 既发又收
        status = HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, len, timeout_ms);
    }
    else if (tx_data != NULL)
    {
        // 只发
        status = HAL_SPI_Transmit(dev->hspi, tx_data, len, timeout_ms);
    }
    else if (rx_data != NULL)
    {
        // 只收
        status = HAL_SPI_Receive(dev->hspi, rx_data, len, timeout_ms);
    }
    else
    {
        status = HAL_ERROR; // 参数错误
    }

    // 拉高片选
    Spi_cs_disable(dev);

    return status;
}

HAL_StatusTypeDef Spi_swap_data_dma(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    if (!dev) return HAL_ERROR;

    // 检查总线是否忙碌 (硬件层面)
    if (dev->hspi->State != HAL_SPI_STATE_READY) {
        return HAL_BUSY;
    }

    // 总线仲裁 & 抢占
    // 如果你有 SPI2/3，请依照 SPI1 的逻辑往下加
    if (dev->hspi == &hspi1) 
    {
        if (current_spi1_owner != NULL) return HAL_BUSY; // 软件层面双重保险
        current_spi1_owner = dev; // 登记：SPI1 归我了
    }

    else if (dev->hspi == &hspi2)
    {
        if (current_spi2_owner != NULL) return HAL_BUSY;
        current_spi2_owner = dev;
    }

    else
    {
        return HAL_ERROR; // 未知的 SPI 句柄
    }

    // 记录缓冲区信息 (可选，方便调试)
    dev->tx_buffer = tx_data;
    dev->rx_buffer = rx_data;
    dev->data_len  = len;

    // 拉低片选 (Start)
    Spi_cs_enable(dev);

    // 启动 DMA
    // 注意：即使单向传输，建议统一用 TransmitReceive，未使用的buffer可指向 dummy
    HAL_StatusTypeDef status;
    if(tx_data && rx_data)
        status = HAL_SPI_TransmitReceive_DMA(dev->hspi, tx_data, rx_data, len);
    else if(tx_data)
        status = HAL_SPI_Transmit_DMA(dev->hspi, tx_data, len);
    else if(rx_data)
        status = HAL_SPI_Receive_DMA(dev->hspi, rx_data, len);
    else 
        status = HAL_ERROR;

    // 如果启动失败，需要立即释放资源
    if (status != HAL_OK)
    {
        Spi_cs_disable(dev);
        if (dev->hspi == &hspi1) current_spi1_owner = NULL;
        if (dev->hspi == &hspi2) current_spi2_owner = NULL;
    }

    return status;
}

// ================= 中断回调处理 (核心逻辑) =================

/**
 * @brief HAL库 SPI 发送/接收完成回调 (在 DMA 中断中被调用)
 * @note  这个函数是 HAL 库弱定义的重写
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Spi_device_t *active_dev = NULL;

    // 查找是哪个总线触发的中断，并找回对应的设备对象
    if (hspi == &hspi1) 
    {
        active_dev = current_spi1_owner;
        current_spi1_owner = NULL; // 解锁总线
    }

    else if (hspi == &hspi2)
    {
        active_dev = current_spi2_owner;
        current_spi2_owner = NULL;
    }

    // 执行回调逻辑
    if (active_dev != NULL) 
    {
        Spi_cs_disable(active_dev);
        if (active_dev->callback != NULL) 
        {
            active_dev->callback(active_dev->context);
        }
    }
}

// 兼容单向传输完成的回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxRxCpltCallback(hspi);
}