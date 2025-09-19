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
#include "motor_control.h"
#include "motor_chassis_interface.h"
#include <stdio.h>

/**
 * @brief 电机控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Motor_control_task(void const *argument)
{
    // 初始化电机控制模块
    Motor_control_init();
    
    // 任务主循环
    for (;;)
    {
        // 处理底盘电机控制（从Chassis Task获取目标值）
        Motor_control_handle_chassis_motors();
        
        // 处理云台控制指令（从决策层获取）
        Motor_control_handle_gimbal_cmd();
        
        // 处理发射机构控制指令（从决策层获取）
        Motor_control_handle_shoot_cmd();
        
        // 执行所有电机控制
        Motor_control_execute();
        
        // 收集和发布反馈信息
        Motor_control_collect_feedback();
        
        // 任务延时1ms，保持1000Hz的运行频率
        osDelay(1);
    }
}