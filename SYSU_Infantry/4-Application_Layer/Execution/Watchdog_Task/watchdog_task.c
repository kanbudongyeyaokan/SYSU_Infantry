/**
 * @file    watchdog_task.h
 * @brief   看门狗控制任务头文件
 * @author  SYSU电控组
 * @date    2025-09-24
 * @version 1.0
 * 
 * @note    看门狗控制
 */

#include "watchdog_task.h"
#include "bsp_wdg.h"

void Watchdog_control_task(void const *argument)
{

     // 任务主循环
    for (;;)
    {
        Watchdog_control_all();
        
        // 任务延时10ms，保持100Hz的运行频率
        osDelay(5);
    }

}