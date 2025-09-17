/**
 * @file    motor_control.h
 * @brief   电机控制模块头文件
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    提供电机控制所需的功能接口
 */

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "main.h"
#include "dji_motor.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "robot_definitions.h"
#include "message_center.h"
#include "decision_making.h"

/**
 * @brief 电机控制模块初始化
 * @note  初始化电机设备和消息中心通信
 */
void Motor_control_init(void);

/**
 * @brief 处理底盘控制指令
 * @note  订阅底盘控制指令并控制底盘电机
 */
void Motor_control_handle_chassis_cmd(void);

/**
 * @brief 处理云台控制指令
 * @note  订阅云台控制指令并控制云台电机
 */
void Motor_control_handle_gimbal_cmd(void);

/**
 * @brief 处理发射机构控制指令
 * @note  订阅发射控制指令并控制发射机构电机
 */
void Motor_control_handle_shoot_cmd(void);

/**
 * @brief 收集电机反馈信息并发布
 * @note  收集所有电机的反馈数据并通过消息中心发布
 */
void Motor_control_collect_feedback(void);

/**
 * @brief 电机控制执行
 * @note  执行所有电机控制
 */
static inline void Motor_control_execute(void);

/**
 * @brief 获取底盘电机指针数组
 * @return 指向底盘电机数组的指针
 */
static inline Djimotor_device_t** Motor_control_get_chassis_motors(void);

/**
 * @brief 获取云台电机指针数组
 * @return 指向云台电机数组的指针
 */
static inline Djimotor_device_t** Motor_control_get_gimbal_motors(void);

/**
 * @brief 获取发射机构电机指针数组
 * @return 指向发射机构电机数组的指针
 */
static inline Djimotor_device_t** Motor_control_get_shoot_motors(void);

// 外部声明电机数组，用于inline函数
extern Djimotor_device_t *chassis_motors[4];
extern Djimotor_device_t *gimbal_motors[2];
extern Djimotor_device_t *shoot_motors[3];

/**
 * @brief 电机控制执行的内联实现
 */
static inline void Motor_control_execute(void)
{
    Djimotor_control_all();
}

/**
 * @brief 获取底盘电机指针数组的内联实现
 */
static inline Djimotor_device_t** Motor_control_get_chassis_motors(void)
{
    return chassis_motors;
}

/**
 * @brief 获取云台电机指针数组的内联实现
 */
static inline Djimotor_device_t** Motor_control_get_gimbal_motors(void)
{
    return gimbal_motors;
}

/**
 * @brief 获取发射机构电机指针数组的内联实现
 */
static inline Djimotor_device_t** Motor_control_get_shoot_motors(void)
{
    return shoot_motors;
}

#endif // __MOTOR_CONTROL_H__
