/**
 * @file    Chassis_task.h
 * @brief   底盘控制任务头文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    负责底盘运动学解算，并直接通过 Djimotor_set_target()
 *          写入 dji_motor.c 的静态缓冲区；不再向 Motor Task 发送 target。
 */

#ifndef SYSU_INFANTRY_CHASSIS_TASK_H
#define SYSU_INFANTRY_CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 底盘控制任务函数
 * @param argument 任务参数（未使用）
 * @note 频率500Hz，接收决策层指令，进行底盘解算，直接设置电机目标值（静态缓冲区）
 */
void Chassis_control_task(void const *argument);

#endif //SYSU_INFANTRY_CHASSIS_TASK_H