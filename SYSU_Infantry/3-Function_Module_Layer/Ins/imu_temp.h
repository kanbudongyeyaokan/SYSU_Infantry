#ifndef IMU_TEMP_H
#define IMU_TEMP_H
/**
 * @Author  SYSU电控组
 * @file    imu_temp.c
 * @brief   IMU恒温控制模块
 * @note    基于RoboMaster开发板C型：加热电阻连接在 TIM10_CH1 (PF6)
 *
 */
#include <stdint.h>

/**
 * @brief 初始化IMU恒温控制模块
 * @note  配置PID参数并启动TIM10 PWM
 */
void Imu_Temp_Init(void);

/**
 * @brief IMU恒温闭环控制函数
 * @param current_temp 当前测量到的IMU温度(°C)
 * @note  需定期调用
 */
void Imu_Temp_Control(float current_temp);

#endif // IMU_TEMP_H