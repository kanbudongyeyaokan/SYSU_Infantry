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

/**
 * @brief 电机控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Motor_control_task(void const *argument)
{
    // 纯发送任务，不做设备初始化（各模块自行完成）
    
    // 任务主循环
    for (;;)
    {
        // 纯CAN后台发送任务：所有目标值由各功能/应用模块实时更新到 dji_motor.c 的静态缓冲区
        // 仅负责聚合并发送
        Djimotor_control_all();
        
        // 任务延时1ms，保持1000Hz的运行频率
        osDelay(1);
    }
}