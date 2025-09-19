/**
 * @file    motor_chassis_interface.h
 * @brief   Motor Task 和 Chassis Task 之间的数据传递接口
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    定义两个模块之间的静态全局变量接口
 */

#ifndef __MOTOR_CHASSIS_INTERFACE_H__
#define __MOTOR_CHASSIS_INTERFACE_H__

#include "main.h"
#include "robot_definitions.h"

/**
 * @brief 底盘电机速度目标值结构体
 */
typedef struct {
    float chassis_motor_speed[4];  // 四个底盘电机的速度目标值 (rpm)
    uint8_t speed_updated;         // 速度更新标志位，1表示有新的速度值
} Chassis_motor_speed_t;

/**
 * @brief 底盘电机控制配置结构体
 */
typedef struct {
    chassis_mode_e chassis_mode;           // 底盘控制模式
    uint8_t config_updated;                // 配置更新标志位，1表示需要更新配置
} Chassis_motor_config_t;

// 全局变量声明，在motor_chassis_interface.c中定义
extern Chassis_motor_speed_t g_chassis_motor_speed;
extern Chassis_motor_config_t g_chassis_motor_config;

/**
 * @brief 初始化Motor和Chassis接口
 */
void Motor_chassis_interface_init(void);

/**
 * @brief Chassis Task设置电机速度目标值
 * @param motor_speeds 四个电机的速度数组
 */
void Chassis_set_motor_speeds(float motor_speeds[4]);

/**
 * @brief Chassis Task设置电机控制配置
 * @param chassis_mode 底盘控制模式
 */
void Chassis_set_motor_config(chassis_mode_e chassis_mode);

/**
 * @brief Motor Task检查并获取电机速度目标值
 * @param motor_speeds 输出四个电机的速度数组
 * @return uint8_t 1表示有新的速度值，0表示无更新
 */
uint8_t Motor_get_chassis_speeds(float motor_speeds[4]);

/**
 * @brief Motor Task检查并获取电机控制配置
 * @param chassis_mode 输出底盘控制模式
 * @return uint8_t 1表示有新的配置，0表示无更新
 */
uint8_t Motor_get_chassis_config(chassis_mode_e *chassis_mode);

#endif // __MOTOR_CHASSIS_INTERFACE_H__