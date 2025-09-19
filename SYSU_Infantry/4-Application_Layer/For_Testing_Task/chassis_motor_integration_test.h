/**
 * @file    chassis_motor_integration_test.h
 * @brief   底盘电机集成测试任务头文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    用于测试底盘->话题数据->电机驱动的完整数据流框架
 */

#ifndef __CHASSIS_MOTOR_INTEGRATION_TEST_H__
#define __CHASSIS_MOTOR_INTEGRATION_TEST_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 底盘电机集成测试任务函数
 * @param argument 任务参数（未使用）
 * @note 测试底盘解算、消息传递、电机控制的完整链路
 */
void Chassis_motor_integration_test_task(void const *argument);

#endif // __CHASSIS_MOTOR_INTEGRATION_TEST_H__