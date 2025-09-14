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
    
    // 初始化BMI088实例和数据结构
    static Bmi088_data_t bmi088_data;
    static Acc_raw_data_t acc_data;
    static Gyro_raw_data_t gyro_data;
    static float temperature;

    // 创建BMI088配置
    Bmi088_config_t bmi088_config = {
        .spi_handle = &hspi1,
        .accel_cs_gpio_port = GPIOA,
        .accel_cs_gpio_pin = GPIO_PIN_4,
        .gyro_cs_gpio_port = GPIOB,
        .gyro_cs_gpio_pin = GPIO_PIN_0,
        .enable_accel_self_test = true,
        .enable_gyro_self_test = true
    };
    
    // 初始化BMI088设备
    Bmi088_device_t* bmi088 = Bmi088_device_init(&bmi088_config);
    if (bmi088 == NULL || bmi088->last_error != NO_ERROR) {
        printf("BMI088 初始化失败，错误码：0x%02X\r\n", 
               bmi088 ? bmi088->last_error : 0xFF);
    } else {
        printf("BMI088 初始化成功\r\n");
    }


    for (;;)
    {
        if (bmi088 != NULL) {
            // 读取BMI088数据并输出
            Acc_raw_data_t *acc_data_ptr = Read_acc_data(bmi088);
            Gyro_raw_data_t *gyro_data_ptr = Read_gyro_data(bmi088);
            float *temp_ptr = Read_acc_temperature(bmi088);
            
            // 暂时输出测试信息
            printf("BMI088 Test Task Running...\r\n");
            //  Uart_printf(uart_instance,"Hello World\r\n");

            // 输出加速度计数据
            printf("ACC: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
                acc_data_ptr->x, acc_data_ptr->y, acc_data_ptr->z);

            // 输出陀螺仪数据
            printf("GYRO: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\r\n", 
                gyro_data_ptr->roll, gyro_data_ptr->pitch, gyro_data_ptr->yaw);

            // 输出温度数据
            printf("TEMP: %.2f°C\r\n", *temp_ptr);
        }
        
        // 任务延时100ms
        osDelay(100);
    }

}
