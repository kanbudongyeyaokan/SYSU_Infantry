/**
 * @file    motor_task.h
 * @brief   电机控制任务头文件
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    负责直接控制电机的目标值
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
 * @note 频率1000Hz，调用功能模块层接口控制电机
 */
void Motor_control_task(void const *argument);

#endif // __MOTOR_TASK_H__
