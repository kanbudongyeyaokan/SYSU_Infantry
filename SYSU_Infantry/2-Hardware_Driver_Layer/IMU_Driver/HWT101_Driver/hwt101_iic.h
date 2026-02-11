/**
* @file    hwt101_iic.h
 * @brief   HWT101 I2C 驱动 (DMA版，适配 INS 层)
 * @author  SYSU电控组
 */

#ifndef HWT101_IIC_H
#define HWT101_IIC_H

#include "ins.h"       // 包含标准 INS 接口定义
#include "main.h"      // 包含 I2C 句柄定义

// HWT101 默认 I2C 地址 (7-bit: 0x50)
// HAL库需要左移一位: 0x50 << 1 = 0xA0
#define HWT101_I2C_ADDR (0x50 << 1)

// ================= API 接口 =================

/**
 * @brief 获取 HWT101 的标准驱动接口实例
 * @param i2c_handle 指向 I2C 句柄的指针 (如 &hi2c2)
 * @return 返回符合 Ins_driver_interface_t 的指针，可直接传给 Ins_init()
 */
const Ins_driver_interface_t* HWT101_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle);

#endif // HWT101_IIC_H