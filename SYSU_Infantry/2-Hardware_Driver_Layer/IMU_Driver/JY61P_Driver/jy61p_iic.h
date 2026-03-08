/**
 * @file    jy61p_iic.h
 * @brief   JY61P 的 I2C 驱动（适配 INS 抽象层）
 */

#ifndef JY61P_IIC_H
#define JY61P_IIC_H

#include "ins.h"
#include "main.h"

/* 默认 7-bit 地址是 0x50，HAL 需要左移后的 8-bit 地址。 */
#define JY61P_IIC_ADDR (0x50u << 1)

/**
 * @brief  获取 JY61P 的 INS 驱动接口
 * @param  i2c_handle I2C 句柄（例如 &hi2c2）
 * @retval 可直接传给 Ins_init() 的驱动接口指针
 */
const Ins_driver_interface_t *JY61P_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle);

#endif /* JY61P_IIC_H */
