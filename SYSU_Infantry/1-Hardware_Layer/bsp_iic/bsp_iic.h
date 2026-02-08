/**
 * @file    bsp_iic.h
 * @brief   板级支持包 I2C 管理 (BSP Layer)
 * @details 负责统一管理 HAL 库的 I2C 中断回调，分发给具体的传感器驱动
 */

#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "main.h" 

/**
 * @brief  HAL库 I2C 接收完成回调 (重写弱定义)
 * @note   这是全局唯一的 I2C DMA 接收完成入口
 * @param  hi2c: I2C 句柄指针
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif // BSP_IIC_H