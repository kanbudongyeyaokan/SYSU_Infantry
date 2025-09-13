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
#include <stdio.h>

/**
 * @brief BMI088测试任务函数
 * @param argument 任务参数（未使用）
 */
void Bmi088_test_task(void const *argument)
{
    // 初始化BMI088实例和数据结构
    // static Bmi088_data_t bmi088_data;
    
    // TODO: 根据实际硬件配置初始化BMI088
    // static Bmi088_instance_t *bmi088_instance;
    // Bmi088_init_config_s bmi088_config = {
    //     // 配置参数需要根据实际硬件设置
    // };
    // bmi088_instance = Bmi088_register(&bmi088_config);
    
    for (;;)
    {
        // TODO: 读取BMI088数据并输出
        // if (Bmi088_acquire(bmi088_instance, &bmi088_data))
        // {
        //     // 输出加速度计数据
        //     printf("ACC: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
        //            bmi088_data.acc[0], bmi088_data.acc[1], bmi088_data.acc[2]);
        //     
        //     // 输出陀螺仪数据
        //     printf("GYRO: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
        //            bmi088_data.gyro[0], bmi088_data.gyro[1], bmi088_data.gyro[2]);
        //     
        //     // 输出温度数据
        //     printf("TEMP: %.2f°C\r\n", bmi088_data.temperature);
        // }
        
        // 暂时输出测试信息
        printf("BMI088 Test Task Running...\r\n");
        
        // 任务延时100ms
        osDelay(100);
    }
}
