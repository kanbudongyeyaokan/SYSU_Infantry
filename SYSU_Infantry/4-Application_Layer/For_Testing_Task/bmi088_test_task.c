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

extern UartInstance_t* uart_instance;

/**
 * @brief BMI088测试任务函数
 * @param argument 任务参数（未使用）
 */
void Bmi088_test_task(void const *argument)
{
    // 初始化BMI088实例和数据结构
    static Bmi088_data_t bmi088_data;
    static Acc_raw_data_t acc_data;
    static Gyro_raw_data_t gyro_data;
    static float temperature;
    

    // TODO: 根据实际硬件配置初始化BMI088
    // static Bmi088_instance_t *bmi088_instance;
    // Bmi088_init_config_s bmi088_config = {
    //     // 配置参数需要根据实际硬件设置
    // };
    // bmi088_instance = Bmi088_register(&bmi088_config);




    // 初始化BMI088
    Bmi088_error_e error = Bmi088_init();
    if (error != NO_ERROR) {
        // Uart_printf(debug_uart, "BMI088 初始化失败，错误码：0x%02X\r\n", error);
        printf("BMI088 初始化失败，错误码：0x%02X\r\n", error);
    } else {
        // Uart_printf(debug_uart, "BMI088 初始化成功\r\n");
        printf("BMI088 初始化成功\r\n");
    }
    

    for (;;)
    {
        // 读取BMI088数据并输出
        Read_acc_data(&acc_data);
        Read_gyro_data(&gyro_data);
        Read_acc_temperature(&temperature);
        

        // 暂时输出测试信息
       // printf("BMI088 Test Task Running...\r\n");
        Uart_printf(uart_instance,"Hello World\r\n");


        // 输出加速度计数据
        // Uart_printf(debug_uart, "ACC: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
        //        acc_data.x, acc_data.y, acc_data.z);
        printf("ACC: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
               acc_data.x, acc_data.y, acc_data.z);

        // // 输出陀螺仪数据
        // Uart_printf(debug_uart, "GYRO: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\r\n", 
        //        gyro_data.roll, gyro_data.pitch, gyro_data.yaw);
        printf("GYRO: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\r\n", 
               gyro_data.roll, gyro_data.pitch, gyro_data.yaw);

        // // 输出温度数据
        // Uart_printf(debug_uart, "TEMP: %.2f°C\r\n", temperature);
        printf("TEMP: %.2f°C\r\n", temperature);
        

        // 任务延时100ms
        osDelay(100);
    }
}
