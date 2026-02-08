/**
* @file    hwt606_iic.h
 * @brief   HWT606 6轴 IMU 驱动 (I2C Burst Read 版)
 * @note    支持读取三轴角速度 + 三轴角度
 */

#ifndef HWT606_IIC_H
#define HWT606_IIC_H

#include "ins.h"       
#include "main.h"     

// HWT606 默认 I2C 地址 (0x50) -> HAL库左移: 0xA0
#define HWT606_IIC_ADDR (0x50 << 1)

// ================= API 接口 =================
/**
 * @brief 获取 HWT606 驱动实例
 * @param i2c_handle I2C句柄指针 (如 &hi2c2)
 */
const Ins_driver_interface_t* HWT606_Get_Driver(I2C_HandleTypeDef *i2c_handle);

#endif // HWT606_IIC_H