//
// Created by 26524 on 2025/9/16.
//

/**
 * @file    gimval.c
 * @brief   云台功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    云台电机初始化
 */

#ifndef SYSU_INFANTRY_GIMBAL_H
#define SYSU_INFANTRY_GIMBAL_H
#include "dji_motor.h"

/**
 * @brief 云台初始化
 */
void Gimbal_motor_init(void);

/**
 * @brief 获取yaw轴电机指针
 * @return yaw轴电机指针
 */
Djimotor_device_t* Get_yaw_motor(void);

/**
 * @brief 获取pitch轴电机指针
 * @return pitch轴电机指针
 */
Djimotor_device_t* Get_pitch_motor(void) ;
#endif //SYSU_INFANTRY_GIMBAL_H