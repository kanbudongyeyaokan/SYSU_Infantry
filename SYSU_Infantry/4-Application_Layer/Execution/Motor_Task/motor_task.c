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
 * @note  该任务是电机控制系统的【执行末端】，运行频率严格恒定为 1kHz (1ms)。
 *        它不负责控制算法的计算（PID计算在各个功能任务中完成），
 *        而是负责将计算好的电流值 (out_current) 打包成 CAN 报文，
 *        并同步发送给所有挂载的大疆电机。
 *        这种设计实现了 控制解算 与 底层通信 的分离，保证了通信的实时性。
 */
void Motor_control_task(void const *argument)
{
    // 获取当前时间 tick
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    const uint32_t TimeIncrement = 1; // 1ms
    osDelay(2000); // 启动后短暂延时，等待系统稳定
    for (;;)
    {
        // 后台发送CAN报文，实现算发分离
        Djimotor_Send_All_Bus();
        // 使用绝对延时，保证严格的 1kHz 节拍
        vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
    }
}
