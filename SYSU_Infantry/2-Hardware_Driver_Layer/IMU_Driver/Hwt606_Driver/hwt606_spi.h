/**
* @file    hwt606_spi.h
 * @brief   HWT606 6轴IMU SPI驱动
 * @author  SYSU电控组
 */

#ifndef HWT606_SPI_H
#define HWT606_SPI_H

#include "ins.h"       // 包含标准 INS 接口定义
#include "main.h"      // 包含 SPI 句柄定义

// ================= 硬件定义 =================
// 对应 RoboMaster C板 SPI2 引脚
#define HWT_CS_GPIO_Port GPIOB
#define HWT_CS_Pin       GPIO_PIN_12

// ================= API 接口 =================

/**
 * @brief 获取 HWT606 的 SPI 驱动接口实例
 * @param spi_handle 指向 SPI 句柄的指针 (如 &hspi2)
 */
const Ins_driver_interface_t* HWT606_SPI_Get_Driver(SPI_HandleTypeDef *spi_handle);

#endif // HWT606_SPI_H