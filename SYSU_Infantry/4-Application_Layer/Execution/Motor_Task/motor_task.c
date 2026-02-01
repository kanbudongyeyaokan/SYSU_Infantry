/**
 * @file    motor_task.c
 * @brief   电机控制任务源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    负责直接控制电机的目标值，运行频率1000Hz，纯CAN发送任务
 */

#include "motor_task.h"
#include "dji_motor.h"
#include <stdio.h>
#include "robot_task.h"
#include "FreeRTOS.h"
#include "task.h"



/**
 * @brief 电机控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Motor_control_task(void const *argument)
{
    // 获取当前时间 tick
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    const uint32_t TimeIncrement = 1; // 1ms

    for (;;)
    {
        // 后台发送CAN报文，实现算发分离
        Djimotor_Send_All_Bus();
        //printf("HELLO\r\n");
        // 使用绝对延时，保证严格的 1kHz 节拍
        vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
    }
}
