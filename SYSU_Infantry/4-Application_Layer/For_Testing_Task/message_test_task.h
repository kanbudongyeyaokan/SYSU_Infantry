/**
 * @file    message_test_task.h
 * @brief   消息中心发布订阅模型测试任务头文件
 * @author  SYSU电控组
 * @date    2025-09-16
 * @version 1.0
 * 
 * @note    用于测试发布订阅模型的各种场景，包括：
 *          1. 一对一发布订阅
 *          2. 多发布者对一订阅者
 *          3. 多发布者对多订阅者
 *          4. 一发布者对多订阅者
 */

#ifndef __MESSAGE_TEST_TASK_H__
#define __MESSAGE_TEST_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief 消息中心测试任务函数
 * @param argument 任务参数（未使用）
 */
void Message_test_task(void const *argument);

#endif // __MESSAGE_TEST_TASK_H__
