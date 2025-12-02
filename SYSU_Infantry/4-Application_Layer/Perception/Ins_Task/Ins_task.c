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

static Bmi088_device_t *bmi088_device;

#define INS_EKF_STATIC_THRESHOLD     200U
#define INS_EKF_UPDATE_THRESHOLD     500U

// 简单的向量运算，用于初始化
static void vec_cross(float a[3], float b[3], float res[3]) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

static float vec_dot(float a[3], float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void vec_norm(float v[3]) {
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len > 1e-6f) {
        v[0] /= len; v[1] /= len; v[2] /= len;
    }
}

// [新增] 初始化四元数函数
// 读取加速度计数据，计算初始倾角，直接对齐重力方向
static void Ins_init_quaternion_from_accel(Bmi088_device_t* bmi088) {
    Acc_raw_data_t *acc_data;
    float acc_avg[3] = {0};
    float count = 0;

    // 1. 读取100次数据取平均
    for (int i = 0; i < 100; i++) {
        acc_data = Read_acc_data(bmi088);
        if (acc_data) {
            // 应用旋转矩阵转换到机体坐标系
            float acc_body[3];
            const float (*acc_rot)[3] = bmi088->config.accel_rotation;
            acc_body[0] = acc_rot[0][0] * acc_data->x + acc_rot[0][1] * acc_data->y + acc_rot[0][2] * acc_data->z;
            acc_body[1] = acc_rot[1][0] * acc_data->x + acc_rot[1][1] * acc_data->y + acc_rot[1][2] * acc_data->z;
            acc_body[2] = acc_rot[2][0] * acc_data->x + acc_rot[2][1] * acc_data->y + acc_rot[2][2] * acc_data->z;

            acc_avg[0] += acc_body[0];
            acc_avg[1] += acc_body[1];
            acc_avg[2] += acc_body[2];
            count++;
        }
        HAL_Delay(2);
    }

    if (count > 0) {
        acc_avg[0] /= count;
        acc_avg[1] /= count;
        acc_avg[2] /= count;
        vec_norm(acc_avg);

        // 2. 计算从测量到的加速度向量旋转到 [0,0,1] 的四元数
        // 注意：这里我们算的是如何把机体水平面转到当前姿态，还是反过来？
        // 目标：Body Frame 的 Gravity (acc_avg) -> Navigation Frame Gravity (0,0,1)
        // 旋转轴 axis = acc_avg X [0,0,1]
        float gravity_ref[3] = {0, 0, 1};
        float axis[3];
        vec_cross(acc_avg, gravity_ref, axis);
        vec_norm(axis);

        float angle = acosf(vec_dot(acc_avg, gravity_ref));

        // 3. 构建四元数 [cos(theta/2), sin(theta/2)*x, sin(theta/2)*y, sin(theta/2)*z]
        float half_angle = angle * 0.5f;
        float sin_half = sinf(half_angle);

        bmi088->data.ekf_state.quaternion.q0 = cosf(half_angle);
        bmi088->data.ekf_state.quaternion.q1 = axis[0] * sin_half;
        bmi088->data.ekf_state.quaternion.q2 = axis[1] * sin_half;
        bmi088->data.ekf_state.quaternion.q3 = axis[2] * sin_half;

        // 4. 更新一下Euler以便立即查看
        Ekf_quaternion_to_euler(&bmi088->data.ekf_state.quaternion, &bmi088->data.ekf_state.euler);
    }
}

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
        // 保持你原有的旋转矩阵设置
        .accel_rotation = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        },
        .gyro_rotation = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        }
    };

    // [关键修改] 使用成功代码(QuaternionEKF)的参数配置
    Ekf_config_t ekf_config = {
        .process_noise_q = 10.0f,            // Q1
        .gyro_bias_noise = 0.001f,           // Q2
        .measurement_noise_r = 1000000.0f,   // R: 非常大，信任陀螺仪
        .dt = 0.001f,
        .fading_factor = 0.9996f,            // Lambda

        .enable_bias_correction = true,
        .static_threshold = 0.3f,
        .enable_temp_compensation = false,
    };

    bmi088_device = Bmi088_device_init(&bmi088_config);
    if (bmi088_device != NULL)
    {
        Bmi088_ekf_init(bmi088_device, &ekf_config);

        // [新增] 在EKF初始化后，手动计算一次初始姿态
        // 这样上电时如果板子是斜的，Roll/Pitch也能立刻对齐，不需要慢慢收敛
        Ins_init_quaternion_from_accel(bmi088_device);
    }

    Imu_state_e published_state = (bmi088_device != NULL) ? bmi088_device->data.state : IMU_STATE_INIT;
    update_attitude_data(NULL, NULL, NULL, NULL, published_state);

    for (;;)
    {
        if (bmi088_device == NULL)
        {
            osDelay(10);
            continue;
        }

        Imu_state_e current_state = bmi088_device->data.state;

        if (bmi088_device->last_error != NO_ERROR) {
            bmi088_device->data.state = IMU_STATE_ERROR;
            osDelay(1);
            continue;
        }

        // 核心更新逻辑
        Bmi088_error_e update_result = Bmi088_ekf_update(bmi088_device);

        if (update_result == NO_ERROR) {
            Acc_raw_data_t *acc_data = Read_acc_data(bmi088_device);
            Gyro_raw_data_t *gyro_data = Read_gyro_data(bmi088_device);
            Euler_angles_t *euler_angles = Bmi088_get_euler_angles(bmi088_device);

            if (acc_data && gyro_data && euler_angles)
            {
                // 数据转换部分保持不变
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

                Imu_state_e new_state = IMU_STATE_READY;
                if(bmi088_device->data.ekf_state.update_count < INS_EKF_UPDATE_THRESHOLD) {
                    new_state = IMU_STATE_CALIBRATING;
                }

                bmi088_device->data.state = new_state;
                update_attitude_data(&acc_body,
                                     &gyro_body,
                                     euler_angles,
                                     (new_state == IMU_STATE_READY) ? &bmi088_device->data.ekf_state.yaw_total_angle : NULL,
                                     new_state);

                // ================= [调试打印区] =================
                static uint32_t print_count = 0;
                // 降低打印频率，防止串口缓冲区溢出导致阻塞
                 {
                     printf("IMU: Roll:%.2f, Pitch:%.2f, Yaw:%.2f\r\n",
                            euler_angles->roll,
                            euler_angles->pitch,
                            euler_angles->yaw);
                }
                // ==============================================
            }
        } else {
            bmi088_device->data.state = IMU_STATE_ERROR;
            update_attitude_data(NULL, NULL, NULL, NULL, IMU_STATE_ERROR);
        }
        osDelay(1);
    }
}