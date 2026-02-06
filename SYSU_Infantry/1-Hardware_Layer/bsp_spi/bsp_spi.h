#ifndef _BSP_SPI_H
#define _BSP_SPI_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// 最大支持的 SPI 设备实例数量 (根据实际需要修改)
#define SPI_DEVICE_MAX_COUNT 8

/**
 * @brief SPI回调函数类型定义
 * @param context 用户上下文指针 (通常传入设备驱动对象本身，如 Bmi088_t*)
 */
typedef void (*SpiCallback_t)(void* context);

/**
 * @brief SPI 初始化配置结构体
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;       // 归属的 SPI 总线句柄 (如 &hspi1)
    GPIO_TypeDef      *cs_port;    // 片选引脚 Port (如 GPIOA)
    uint16_t           cs_pin;     // 片选引脚 Pin  (如 GPIO_PIN_4)

    void* context;                 // 上下文指针 (传给回调函数用)
    SpiCallback_t      callback;   // DMA传输完成后的回调函数 (不需要则填 NULL)
} Spi_init_config_t;

/**
 * @brief SPI 设备对象结构体
 */
typedef struct
{
    // --- 硬件属性 ---
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    // --- 运行时数据 (DMA用) ---
    uint8_t *tx_buffer;           // 当前正在发送的数据指针
    uint8_t *rx_buffer;           // 当前正在接收的数据指针
    uint16_t data_len;            // 数据长度

    // --- 回调机制 ---
    void* context;                // 上下文
    SpiCallback_t callback;       // 完成回调
}Spi_device_t;

// ================= API 接口 =================

/**
 * @brief  注册/初始化一个 SPI 设备对象
 * @param  config 初始化配置结构体指针
 * @return SpiDevice_t* 返回创建的对象指针，如果失败返回 NULL
 */
Spi_device_t* Spi_device_init(Spi_init_config_t *config);

/**
 * @brief  [阻塞模式] 交换数据
 * @note   适用于初始化阶段，如写寄存器配置。CPU会等待传输完成。
 * @param  dev      SPI设备对象
 * @param  tx_data  发送数据指针 (如果只想读，填 NULL 或 dummy 数组)
 * @param  rx_data  接收数据指针 (如果只想写，填 NULL)
 * @param  len      数据长度
 * @param  timeout_ms 超时时间
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Spi_swap_data_block(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief  [DMA模式] 异步交换数据
 * @note   适用于周期性读取传感器。函数立即返回。
 * 传输开始时自动拉低 CS，传输完成(进入中断)后自动拉高 CS 并调用 callback。
 * @param  dev      SPI设备对象
 * @param  tx_data  发送数据指针 (必须指向静态或全局内存，不可指向栈内存!)
 * @param  rx_data  接收数据指针 (必须指向静态或全局内存!)
 * @param  len      数据长度
 * @return HAL_StatusTypeDef (如果总线忙，返回 HAL_BUSY)
 */
HAL_StatusTypeDef Spi_swap_data_dma(Spi_device_t *dev, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

#endif // _BSP_SPI_H