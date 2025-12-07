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
#include "chassis.h"


/**
 * @brief 底盘控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Chassis_control_task(void const *argument)
{
    // 初始化底盘任务
    Chassis_task_init();
    
    // 任务主循环
    for (;;)
    {
        // 处理底盘控制指令
        Chassis_handle_command();
        
        // 任务延时2ms，保持500Hz的运行频率
        osDelay(5);
    }
}