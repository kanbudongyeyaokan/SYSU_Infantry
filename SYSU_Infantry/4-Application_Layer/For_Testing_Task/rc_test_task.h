/**
 * @file    rc_test_task.h
 * @brief   遥控器测试任务头文件
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 1.0
 * 
 * @note    用于单独测试遥控器功能
 */

#ifndef __RC_TEST_TASK_H__
#define __RC_TEST_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 遥控器测试任务函数
 * @param argument 任务参数（未使用）
 */
void Rc_test_task(void const *argument);

#endif // __RC_TEST_TASK_H__
