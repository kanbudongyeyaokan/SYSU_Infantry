// /**
//  * @file    can_motors_test_task.c
//  * @brief   HWT606陀螺仪测试任务源文件
//  * @author  SYSU电控组
//  * @date    2025-09-12
//  * @version 3.1
//  * @note    用于测试HWT606陀螺仪数据读取功能
//  */
//
// #include "can_motors_test_task.h"
// #include "main.h"
// #include <stdio.h>
// #include <string.h>
// #include "hwt606_uart.h"
//
// // 外部UART句柄声明
// extern UART_HandleTypeDef huart1;
//
// // HWT606数据指针
// static HWT606_Data_t *hwt606_data = NULL;
//
// void Can_motors_test_task(void const *argument)
// {
//     // 等待系统稳定
//     osDelay(500);
//
//     // 初始化HWT606陀螺仪 (使用UART1)
//     hwt606_data = HWT606_Init(&huart1);
//
//     if (hwt606_data == NULL) {
//
//         for (;;) {
//             osDelay(1000);
//         }
//     }
//
//     uint32_t tick = 0;
//     uint32_t last_update_count = 0;
//
//     for (;;)
//     {
//         // 每100ms打印一次数据
//         if (tick % 20 == 0) {  // 5ms * 20 = 100ms
//             // 获取最新数据
//         }
//
//         tick++;
//         osDelay(5);
//     }
// }
