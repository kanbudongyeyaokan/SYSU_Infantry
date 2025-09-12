/**
 * @file    bmi088_test_task.h
 * @brief   BMI088测试任务头文件
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 1.0
 * 
 * @note    用于测试BMI088传感器功能
 */

#ifndef __BMI088_TEST_TASK_H__
#define __BMI088_TEST_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief BMI088测试任务函数
 * @param argument 任务参数（未使用）
 */
void Bmi088_test_task(void const *argument);

#endif // __BMI088_TEST_TASK_H__
