/**
* @file    Gimbal_task.c
 * @brief   云台控制任务源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    调用来自方法层中的云台接口进行云台控制
 */

#include "Gimbal_task.h"
#include "gimbal.h"
#include "cmsis_os.h"


/**
 * @brief 云台控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Gimbal_control_task(void const *argument) {
    //任务初始化
    Gimbal_task_init();

    for (;;) {
        //处理控制指令
        Gimbal_handle_command();

        //修改控制频率，保持200Hz
        osDelay(5);
    }
}