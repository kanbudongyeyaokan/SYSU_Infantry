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
    
    // 创建EKF配置
    Ekf_config_t ekf_config = {
        .process_noise_q = 0.01f,
        .measurement_noise_r = 0.1f,
        .gyro_bias_noise = 0.001f,
        .dt = 0.001f, // 1ms
        .enable_bias_correction = true,
        .static_threshold = 0.2f
    };
    
    // 初始化BMI088设备
    Bmi088_device_t* bmi088 = Bmi088_device_init(&bmi088_config);
    if (bmi088 == NULL || bmi088->last_error != NO_ERROR) {
        printf("BMI088 初始化失败，错误码：0x%02X\r\n", 
               bmi088 ? bmi088->last_error : 0xFF);
    } else {
        printf("BMI088 初始化成功\r\n");
    }
    
    // 初始化EKF
    if (bmi088 != NULL) {
        Bmi088_error_e ekf_error = Bmi088_ekf_init(bmi088, &ekf_config);
        if (ekf_error != NO_ERROR) {
            printf("EKF 初始化失败，错误码：0x%02X\r\n", ekf_error);
        } else {
            printf("EKF 初始化成功\r\n");
        }
    }

    for (;;)
    {
        if (bmi088 != NULL) {
            // 更新EKF（自动读取传感器数据并进行姿态估计）
            Bmi088_error_e update_error = Bmi088_ekf_update(bmi088);
            
            if (update_error == NO_ERROR) {
                // 获取原始传感器数据
                Acc_raw_data_t *acc_data_ptr = Read_acc_data(bmi088);
                Gyro_raw_data_t *gyro_data_ptr = Read_gyro_data(bmi088);
                float *temp_ptr = Read_acc_temperature(bmi088);
                
                // 获取EKF姿态估计结果
                Quaternion_t *quaternion = Bmi088_get_quaternion(bmi088);
                Euler_angles_t *euler = Bmi088_get_euler_angles(bmi088);
                
                // 输出原始传感器数据
                printf("=== BMI088 Raw Data ===\r\n");
                printf("ACC: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
                    acc_data_ptr->x, acc_data_ptr->y, acc_data_ptr->z);
                printf("GYRO: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\r\n", 
                    gyro_data_ptr->roll, gyro_data_ptr->pitch, gyro_data_ptr->yaw);
                printf("TEMP: %.2f°C\r\n", *temp_ptr);
                
                // 输出EKF估计结果
                if (quaternion != NULL && euler != NULL) {
                    printf("=== EKF Attitude Estimation ===\r\n");
                    printf("Quaternion: q0=%.4f, q1=%.4f, q2=%.4f, q3=%.4f\r\n",
                        quaternion->q0, quaternion->q1, quaternion->q2, quaternion->q3);
                    printf("Euler Angles: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\r\n",
                        euler->roll, euler->pitch, euler->yaw);
                    
                    // 输出零偏信息（如果可访问）
                    Ekf_state_t* ekf_state = &bmi088->data.ekf_state;
                    printf("Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f\r\n",
                        ekf_state->gyro_bias[0], ekf_state->gyro_bias[1], ekf_state->gyro_bias[2]);
                    printf("Static State: %s\r\n", ekf_state->is_static ? "YES" : "NO");
                }
                
                printf("----------------------------\r\n");
            } else {
                printf("EKF 更新失败，错误码：0x%02X\r\n", update_error);
            }
        }
        
        // 任务延时10ms（100Hz更新频率）
        osDelay(10);
    }

}
