/**
 * @file    hwt606_iic.h
 * @brief   HWT606 6轴 IMU 驱动 (DMA版)
 * @note    支持读取三轴角速度 + 三轴角度，配合 bsp_iic.c 使用
 */

#ifndef HWT606_IIC_H
#define HWT606_IIC_H

#include "ins.h"      
#include "main.h"      

#define HWT606_IIC_ADDR (0x50 << 1)

// ================= API 接口 =================

/**
 * @brief 获取 HWT606 驱动实例
 * @param i2c_handle I2C句柄指针 (如 &hi2c2)
 */
const Ins_driver_interface_t* HWT606_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle);

/**
 * @brief  DMA接收完成回调 (供 bsp_iic.c 调用)
 * @note   不要在用户代码中手动调用
 */
void HWT606_RxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif // HWT606_IIC_H