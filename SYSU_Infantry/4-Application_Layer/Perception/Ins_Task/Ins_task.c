#include "Ins_task.h"

#include <stdio.h>

#include "cmsis_os.h"
#include "ins.h"
#include "bmi088.h"
#include "main.h"

// BMI088设备实例
static Bmi088_device_t *bmi088_device;

/**
 * @brief          ins_task
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Ins_task(void const *argument)
{
    // 初始化BMI088配置
    Bmi088_config_t bmi088_config = {
        .spi_handle = &hspi1,
        .accel_cs_gpio_port = GPIOA,
        .accel_cs_gpio_pin = GPIO_PIN_4,
        .gyro_cs_gpio_port = GPIOB,
        .gyro_cs_gpio_pin = GPIO_PIN_0,
        .enable_accel_self_test = true,
        .enable_gyro_self_test = true};

    // 初始化EKF配置
    Ekf_config_t ekf_config = {
        .process_noise_q = 0.01f,
        .measurement_noise_r = 0.1f,
        .gyro_bias_noise = 0.001f,
        .dt = 0.001f, // 1ms, 1000Hz
        .enable_bias_correction = true,
        .static_threshold = 0.2f};

    // 初始化BMI088设备
    bmi088_device = Bmi088_device_init(&bmi088_config);
    if (bmi088_device != NULL)
    {
        // 初始化EKF
        Bmi088_ekf_init(bmi088_device, &ekf_config);
    }

    for (;;)
    {
        if (bmi088_device != NULL)
        {
            // 更新EKF（此函数内部会自动读取传感器数据）
            Bmi088_ekf_update(bmi088_device);

            // 获取更新后的数据
            Acc_raw_data_t *acc_data = Read_acc_data(bmi088_device);
            Gyro_raw_data_t *gyro_data = Read_gyro_data(bmi088_device);
            Euler_angles_t *euler_angles = Bmi088_get_euler_angles(bmi088_device);

            // 更新到ins模块
            if (acc_data && gyro_data && euler_angles)
            {
                update_attitude_data(acc_data, gyro_data, euler_angles);
            }
            
            // 调试打印 - 欧拉角
            printf("euler_yaw:%f\r\n", euler_angles->yaw);
            
            // 调试打印 - 原始陀螺仪数据
            printf("gyro_raw: x:%f, y:%f, z:%f\r\n", 
                   gyro_data->roll, gyro_data->pitch, gyro_data->yaw);
            
            // 调试打印 - 四元数
            printf("quaternion: q0:%f, q1:%f, q2:%f, q3:%f\r\n", 
                   bmi088_device->data.ekf_state.quaternion.q0,
                   bmi088_device->data.ekf_state.quaternion.q1,
                   bmi088_device->data.ekf_state.quaternion.q2,
                   bmi088_device->data.ekf_state.quaternion.q3);
            
            // 调试打印 - 陀螺仪零偏
            printf("gyro_bias: x:%f, y:%f, z:%f\r\n", 
                   bmi088_device->data.ekf_state.gyro_bias[0],
                   bmi088_device->data.ekf_state.gyro_bias[1],
                   bmi088_device->data.ekf_state.gyro_bias[2]);
            
            // 调试打印 - 静态状态
            printf("is_static:%d, static_count:%d\r\n", 
                   bmi088_device->data.ekf_state.is_static,
                   bmi088_device->data.ekf_state.static_count);
                   
            printf("===================\r\n");

        }
        // 1000Hz
        osDelay(1);
    }
}