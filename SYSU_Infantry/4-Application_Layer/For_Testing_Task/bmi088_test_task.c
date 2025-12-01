/**
 * @file    bmi088_test_task.c
 * @brief   BMI088测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 1.0
 * 
 * @note    用于测试BMI088传感器功能
 */

#include "bmi088_test_task.h"

// #include "bmi088.h"
// #include "bsp_log.h"

#include "bmi088.h"

#include "bsp_usart.h"
#include <stdio.h>

//extern UartInstance_t* uart_instance;

/**
 * @brief BMI088测试任务函数
 * @param argument 任务参数（未使用）
 */
void Bmi088_test_task(void const *argument)
{




    for (;;)
    {


        
        // 任务延时10ms（100Hz更新频率）
        osDelay(10);
    }

}
