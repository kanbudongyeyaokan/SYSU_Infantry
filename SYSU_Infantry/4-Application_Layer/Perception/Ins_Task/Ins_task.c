#include "Ins_task.h"

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "cmsis_os.h"
#include "ins.h"
#include "bmi088.h"
#include "main.h"
#include "bsp_dwt.h"
#include "algorithm_ekf.h"

// BMI088设备实例
static Bmi088_device_t *bmi088_device;

#define INS_EKF_STATIC_THRESHOLD     200U
#define INS_EKF_UPDATE_THRESHOLD     500U

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
        .enable_gyro_self_test = true,
        // 修正后的旋转矩阵：符合右手坐标系 (X前, Y右, Z下)
        .accel_rotation = {
            {1.0f, 0.0f, 0.0f},  // 机体X(前) = -IMU_X (IMU_X指向机体后方)
            {0.0f, 1.0f, 0.0f},   // 机体Y(右) = IMU_Y (IMU_Y指向机体右侧)
            {0.0f, 0.0f, 1.0f}    // 机体Z(下) = IMU_Z (IMU_Z指向机体下方)
        },
        .gyro_rotation = {
            {1.0f, 0.0f, 0.0f},  // 机体Roll(绕X轴) = -IMU_Roll
            {0.0f, 1.0f, 0.0f},   // 机体Pitch(绕Y轴) = IMU_Pitch  
            {0.0f, 0.0f, 1.0f}    // 机体Yaw(绕Z轴) = IMU_Yaw
        }
    };

    // 初始化EKF配置 - 使用优化的配置来修复yaw轴漂移
    Ekf_config_t ekf_config = {
        .process_noise_q = 10.0f,                // 过程噪声（参考QuaternionEKF）
        .measurement_noise_r = 1000.0f,          // 测量噪声，针对yaw漂移优化（比原1000000小一些）
        .gyro_bias_noise = 0.001f,               // 陀螺仪零偏噪声
        .dt = 0.001f,                            // 1ms, 1000Hz
        .enable_bias_correction = true,          // 启用零偏校正
        .static_threshold = 0.3f,                // 静态检测阈值（参考QuaternionEKF）
        .fading_factor = 0.9996f,                // 关键参数！渐减因子，防止零偏过度收敛

        // 新增温度补偿配置
        .enable_temp_compensation = true,        // 启用温度补偿
        .temp_coeff_x = 0.01f * (3.14159265f / 180.0f), // X轴温度系数 0.01°/s/°C
        .temp_coeff_y = 0.01f * (3.14159265f / 180.0f), // Y轴温度系数 0.01°/s/°C  
        .temp_coeff_z = 0.015f * (3.14159265f / 180.0f),// Z轴温度系数 0.015°/s/°C（稍大一些）
        .baseline_temp = 25.0f                   // 基准温度25°C
    };

    // 原始配置（已优化为上面的配置）
    // Ekf_config_t ekf_config = {
    //     .process_noise_q = 0.01f,
    //     .measurement_noise_r = 0.1f,
    //     .gyro_bias_noise = 0.001f,
    //     .dt = 0.001f, // 1ms, 1000Hz
    //     .enable_bias_correction = true,
    //     .static_threshold = 0.2f};

    // 跃鹿 EKF 的配置（已整合到上面的优化配置中）
    // Ekf_config_t ekf_config = {
    //     .process_noise_q = 10.0f,                // 增大过程噪声，提高响应性（参考QuaternionEKF用10）
    //     .measurement_noise_r = 1000000.0f,       // 减小测量噪声的影响（参考QuaternionEKF用1000000）
    //     .gyro_bias_noise = 0.001f,               // 保持零偏噪声不变
    //     .dt = 0.001f,                            // 1ms, 1000Hz
    //     .enable_bias_correction = true,
    //     .static_threshold = 0.3f                 // 提高静态阈值，减少误判（参考QuaternionEKF用0.3）
    // };

    // 初始化BMI088设备
    bmi088_device = Bmi088_device_init(&bmi088_config);
    if (bmi088_device != NULL)
    {
        // 初始化EKF
        Bmi088_error_e ekf_init_result = Bmi088_ekf_init(bmi088_device, &ekf_config);
        if (ekf_init_result == NO_ERROR) {
            // printf("INS EKF initialized successfully with optimized parameters!\n");
            // printf("Yaw drift fix configuration:\n");
            // printf("   - Process noise: %.1f\n", ekf_config.process_noise_q);
            // printf("   - Measurement noise: %.1f\n", ekf_config.measurement_noise_r);
            // printf("   - Fading factor: %.6f (KEY parameter!)\n", ekf_config.fading_factor);
            // printf("   - Static threshold: %.1f rad/s\n", ekf_config.static_threshold);
        } else {
            // printf("EKF initialization failed! Error: %d\n", ekf_init_result);
        }
    } else {
        // printf("BMI088 initialization failed!\n");
    }

    Imu_state_e published_state = (bmi088_device != NULL) ? bmi088_device->data.state : IMU_STATE_INIT;
    update_attitude_data(NULL, NULL, NULL, NULL, published_state);

    for (;;)
    {
        if (bmi088_device == NULL)
        {
            if (published_state != IMU_STATE_ERROR) {
                published_state = IMU_STATE_ERROR;
                update_attitude_data(NULL, NULL, NULL, NULL, published_state);
                // printf("INS device not initialized\n");
            }
            osDelay(10);
            continue;
        }

        Imu_state_e current_state = bmi088_device->data.state;

        if (bmi088_device->last_error != NO_ERROR) {
            if (current_state != IMU_STATE_ERROR) {
                // printf("INS sensor error detected (code=%d)\n", bmi088_device->last_error);
            }
            bmi088_device->data.state = IMU_STATE_ERROR;
            if (published_state != IMU_STATE_ERROR) {
                update_attitude_data(NULL, NULL, NULL, NULL, IMU_STATE_ERROR);
                published_state = IMU_STATE_ERROR;
            }
            osDelay(1);
            continue;
        }

        if (current_state == IMU_STATE_INIT) {
            if (published_state != IMU_STATE_INIT) {
                update_attitude_data(NULL, NULL, NULL, NULL, IMU_STATE_INIT);
                published_state = IMU_STATE_INIT;
            }
            osDelay(1);
            continue;
        }

        // 温度补偿：读取BMI088的温度数据并设置到EKF
        float current_temp = Bmi088_get_temperature(bmi088_device);
        Ekf_set_temperature(&bmi088_device->data.ekf_state, current_temp);

        // 更新EKF（此函数内部会自动读取传感器数据）
        Bmi088_error_e update_result = Bmi088_ekf_update(bmi088_device);

        if (update_result == NO_ERROR) {
            // 获取更新后的数据
            Acc_raw_data_t *acc_data = Read_acc_data(bmi088_device);
            Gyro_raw_data_t *gyro_data = Read_gyro_data(bmi088_device);
            Euler_angles_t *euler_angles = Bmi088_get_euler_angles(bmi088_device);

            // 更新到ins模块
            if (acc_data && gyro_data && euler_angles)
            {
                Acc_raw_data_t acc_body = {0};
                Gyro_raw_data_t gyro_body = {0};

                const float (*acc_rot)[3] = bmi088_device->config.accel_rotation;
                const float (*gyro_rot)[3] = bmi088_device->config.gyro_rotation;

                acc_body.x = acc_rot[0][0] * acc_data->x + acc_rot[0][1] * acc_data->y + acc_rot[0][2] * acc_data->z;
                acc_body.y = acc_rot[1][0] * acc_data->x + acc_rot[1][1] * acc_data->y + acc_rot[1][2] * acc_data->z;
                acc_body.z = acc_rot[2][0] * acc_data->x + acc_rot[2][1] * acc_data->y + acc_rot[2][2] * acc_data->z;

                gyro_body.roll = gyro_rot[0][0] * gyro_data->roll + gyro_rot[0][1] * gyro_data->pitch + gyro_rot[0][2] * gyro_data->yaw;
                gyro_body.pitch = gyro_rot[1][0] * gyro_data->roll + gyro_rot[1][1] * gyro_data->pitch + gyro_rot[1][2] * gyro_data->yaw;
                gyro_body.yaw = gyro_rot[2][0] * gyro_data->roll + gyro_rot[2][1] * gyro_data->pitch + gyro_rot[2][2] * gyro_data->yaw;

                Imu_state_e new_state = IMU_STATE_CALIBRATING;
                bool ekf_converged = false;
                if (bmi088_device->data.ekf_state.update_count >= INS_EKF_UPDATE_THRESHOLD &&
                    bmi088_device->data.ekf_state.static_count >= INS_EKF_STATIC_THRESHOLD) {
                    ekf_converged = true;
                }

                if (ekf_converged) {
                    new_state = IMU_STATE_READY;
                    if (current_state != IMU_STATE_READY) {
                        // printf("INS ready: static_count=%u, update_count=%lu\n",
                               // (unsigned)bmi088_device->data.ekf_state.static_count,
                               // (unsigned long)bmi088_device->data.ekf_state.update_count);
                    }
                }

                bmi088_device->data.state = new_state;
                update_attitude_data(&acc_body,
                                     &gyro_body,
                                     euler_angles,
                                     (new_state == IMU_STATE_READY) ? &bmi088_device->data.ekf_state.yaw_total_angle : NULL,
                                     new_state);
                published_state = new_state;
            }

            // 高频输出 - 原有的调试打印（可选择启用/禁用）
            // printf("[quaternionl]q0/q1/q2/q3 [euler]pitch/roll/yaw [temp]:%f,%f,%f,%f,%f,%f,%f,%f\r\n",
            //       bmi088_device->data.ekf_state.quaternion.q0,
            //       bmi088_device->data.ekf_state.quaternion.q1,
            //       bmi088_device->data.ekf_state.quaternion.q2,
            //       bmi088_device->data.ekf_state.quaternion.q3,
            //       euler_angles->pitch,
            //       euler_angles->roll,
            //       euler_angles->yaw,
            //       current_temp
            //     //   gyro_data->pitch,
            //     //   gyro_data->roll,
            //     //   gyro_data->yaw
            //     );

        } else {
            // EKF更新失败，输出错误信息
            static uint32_t error_count = 0;
            if (++error_count % 1000 == 0) {
                // printf("EKF update error count: %d\n", error_count);
            }
            if (current_state != IMU_STATE_ERROR) {
                // printf("INS readiness revoked: EKF update error\n");
            }
            bmi088_device->data.state = IMU_STATE_ERROR;
            update_attitude_data(NULL, NULL, NULL, NULL, IMU_STATE_ERROR);
            published_state = IMU_STATE_ERROR;
        }
        // 1000Hz
        osDelay(1);
    }
}