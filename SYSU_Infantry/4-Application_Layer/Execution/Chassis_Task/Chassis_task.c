/**
 * @file    Chassis_task.c
 * @brief   底盘控制任务源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    调用来自方法层中的底盘接口进行底盘控制
 */

#include "Chassis_task.h"

#include <stdio.h>

#include "chassis.h"
#include "chassis_power_control.h"
#include "robot_task.h"
#include "supercap_comm.h"

/**
 * @brief 底盘控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Chassis_control_task(void const *argument)
{
    // 初始化底盘任务
    Chassis_task_init();
    // 本地变量，用于接收队列数据
    Chassis_cmd_send_t cmd_recv;
    // 任务主循环
    for (;;)
    {
        //超电发送
       // Send2SuperCap();

        if (xQueueReceive(Chassis_cmd_queue_handle, &cmd_recv, 100) == pdTRUE)
        {
            // === 正常接收到指令 ===
            // 调用逻辑层函数进行解算和设定目标
            Chassis_Update_Control(&cmd_recv);
           // printf("vx: %f,vy:%f\r\n", cmd_recv.vx,cmd_recv.vy);
        }
        else
        {
            // === 超时未收到指令 (安全保护) ===
            // 决策层卡死或通信断开，底盘必须急停防止疯跑
            cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
            cmd_recv.vx = 0;
            cmd_recv.vy = 0;
            cmd_recv.wz = 0;

            Chassis_Update_Control(&cmd_recv);
        }
    }
}