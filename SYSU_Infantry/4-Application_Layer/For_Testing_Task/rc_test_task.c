/**
 * @file    rc_test_task.c
 * @brief   DWT 专用测试任务 (原遥控器任务临时修改)
 * @author  SYSU电控组
 * @note    用于验证 bsp_dwt 库的延时和计时精度
 */

#include "rc_test_task.h"
#include "bsp_dwt.h"
#include <stdio.h>

/**
 * @brief DWT 测试任务
 */
void Rc_test_task(void const *argument)
{
    // 初始化 DWT 计数变量
    static uint32_t last_cnt = 0;
    float dt = 0.0f;
    float timeline = 0.0f;

    // 确保系统稳定
    osDelay(100);

    printf("\r\n==================================\r\n");
    printf("      DWT Library Test Start      \r\n");
    printf("==================================\r\n");

    // 1. 初始化 last_cnt (获取当前时刻作为基准)
    DWT_GetDeltaT(&last_cnt);

    for (;;)
    {
        printf("\r\n[Round Start] Timeline: %.4f s\r\n", DWT_GetTimeline_s());

        // --- 测试 1: 毫秒级延时 (DWT_Delay_ms) ---
        // 目标: 延时 10ms
        // 验证: 使用 DWT_GetDeltaT 测量实际经过的时间

        DWT_GetDeltaT(&last_cnt); // 重置计时起点
        DWT_Delay_ms(10);         // 执行死循环延时
        dt = DWT_GetDeltaT(&last_cnt); // 获取实际消耗时间

        printf("Test 1 (ms) | Target: 10ms   | Measured: %.6f s | Error: %.2f%%\r\n",
               dt, (dt - 0.01f) / 0.01f * 100.0f);


        // --- 测试 2: 微秒级延时 (DWT_Delay_us) ---
        // 目标: 延时 500us (0.5ms)

        DWT_GetDeltaT(&last_cnt);
        DWT_Delay_us(500);
        dt = DWT_GetDeltaT(&last_cnt);

        printf("Test 2 (us) | Target: 500us  | Measured: %.6f s | Error: %.2f%%\r\n",
               dt, (dt - 0.0005f) / 0.0005f * 100.0f);


        // --- 测试 3: 秒级浮点延时 (DWT_Delay) ---
        // 目标: 延时 0.002s (2ms)

        DWT_GetDeltaT(&last_cnt);
        DWT_Delay(0.002f);
        dt = DWT_GetDeltaT(&last_cnt);

        printf("Test 3 (s)  | Target: 0.002s | Measured: %.6f s | Error: %.2f%%\r\n",
               dt, (dt - 0.002f) / 0.002f * 100.0f);


        // --- 测试 4: 时间轴更新 (DWT_SysTimeUpdate) ---
        // 手动更新一次，确保长时间运行不溢出
        DWT_SysTimeUpdate();
        timeline = DWT_GetTimeline_s();
        printf("Sys Update  | Current Time: %.4f s\r\n", timeline);

        // 使用 osDelay 挂起任务，方便观察打印数据，不占用 CPU
        osDelay(1000);
    }
}