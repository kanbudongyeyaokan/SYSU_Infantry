/**
 * @file    watchdog_task.h
 * @brief   看门狗控制任务头文件
 * @author  SYSU电控组
 * @date    2025-09-24
 * @version 1.0
 * 
 * @note    看门狗控制
 */

#ifndef WATCHDOG_TASK_H
#define WATCHDOG_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 看门狗控制任务函数
 * @param argument 任务参数（未使用）
 * @note 频率100Hz，仅调用 Watchdog_control_all() 发送
 */
void Watchdog_control_task(void const *argument);

#endif 
