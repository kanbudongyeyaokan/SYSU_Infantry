/**
 * @file    motor_task.h
 * @brief   电机控制任务头文件
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    纯CAN后台发送任务：不接收Chassis Task target，
 *          由各模块调用 Djimotor_set_target() 写入静态缓冲区，
 *          本任务仅周期性调用 Djimotor_control_all() 聚合发送。
 */

#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 电机控制任务函数
 * @param argument 任务参数（未使用）
 * @note 频率1000Hz，仅调用 Djimotor_control_all() 发送
 */
void Motor_control_task(void const *argument);

#endif // __MOTOR_TASK_H__
