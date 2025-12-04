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
#include "imu_temp.h"

static Bmi088_device_t *bmi088_device;

#define INS_EKF_STATIC_THRESHOLD     200U
#define INS_EKF_UPDATE_THRESHOLD     500U


static void vec_cross(float a[3], float b[3], float res[3]) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

//向量点积
static float vec_dot(float a[3], float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void vec_norm(float v[3]) {
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len > 1e-6f) {
        v[0] /= len; v[1] /= len; v[2] /= len;
    }
}

/**
 * @brief 预热BMI088，使其达到39.5度才开始姿态解算
 * @note  此函数会在 ins_task 开始时被调用。它会死循环检查温度，
 * 并持续调用温控函数，直到温度达标或超时。
 * 这能确保姿态解算开始时，IMU 已经处于恒温状态，从而获得极高的零偏稳定性。
 */
static void Imu_Wait_For_Temp(Bmi088_device_t* bmi088) {
    //目标温度：39.5度
    const float TARGET_TEMP = 39.5f;
    // 超时时间：60000毫秒 (60秒)。
    const uint32_t TIMEOUT_MS = 60000;
    // 记录开始预热的时间戳 (ms)
    uint32_t start_time = HAL_GetTick();
    uint32_t print_tick = 0;

    while (1) {
        float current_temp = Bmi088_get_temperature(bmi088);
        Imu_Temp_Control(current_temp);

        if (current_temp >= TARGET_TEMP) {
            break;
        }
        // 判断是否超时了，温度仍然没有升上来
        if ((HAL_GetTick() - start_time) > TIMEOUT_MS) {
            break;
        }

        if (HAL_GetTick() - print_tick > 500) {
            print_tick = HAL_GetTick();
        }

        osDelay(10);
    }
    osDelay(1000);
}

static void Ins_calibrate_and_init(Bmi088_device_t* bmi088) {
    Acc_raw_data_t *acc_data;
    Gyro_raw_data_t *gyro_data;

    float acc_avg[3] = {0};
    float gyro_z_sum = 0.0f;
    float count = 0;

    const int CALI_SAMPLES = 2000;

    for (int i = 0; i < CALI_SAMPLES; i++) {
        float temp = Bmi088_get_temperature(bmi088);
        Imu_Temp_Control(temp);

        acc_data = Read_acc_data(bmi088);
        gyro_data = Read_gyro_data(bmi088);

        if (acc_data && gyro_data) {
            float acc_body[3];
            const float (*acc_rot)[3] = bmi088->config.accel_rotation;
            acc_body[0] = acc_rot[0][0] * acc_data->x + acc_rot[0][1] * acc_data->y + acc_rot[0][2] * acc_data->z;
            acc_body[1] = acc_rot[1][0] * acc_data->x + acc_rot[1][1] * acc_data->y + acc_rot[1][2] * acc_data->z;
            acc_body[2] = acc_rot[2][0] * acc_data->x + acc_rot[2][1] * acc_data->y + acc_rot[2][2] * acc_data->z;

            acc_avg[0] += acc_body[0];
            acc_avg[1] += acc_body[1];
            acc_avg[2] += acc_body[2];

            const float (*gyro_rot)[3] = bmi088->config.gyro_rotation;
            float gyro_z_body = gyro_rot[2][0] * gyro_data->roll +
                                gyro_rot[2][1] * gyro_data->pitch +
                                gyro_rot[2][2] * gyro_data->yaw;
            gyro_z_sum += gyro_z_body;

            count++;
        }
        osDelay(2);
    }

    if (count > 0) {
        acc_avg[0] /= count;
        acc_avg[1] /= count;
        acc_avg[2] /= count;
        vec_norm(acc_avg);

        float gravity_ref[3] = {0, 0, 1};
        float axis[3];
        vec_cross(acc_avg, gravity_ref, axis);
        vec_norm(axis);
        float angle = acosf(vec_dot(acc_avg, gravity_ref));
        float half_angle = angle * 0.5f;
        float sin_half = sinf(half_angle);

        bmi088->data.ekf_state.quaternion.q0 = cosf(half_angle);
        bmi088->data.ekf_state.quaternion.q1 = axis[0] * sin_half;
        bmi088->data.ekf_state.quaternion.q2 = axis[1] * sin_half;
        bmi088->data.ekf_state.quaternion.q3 = axis[2] * sin_half;

        Ekf_quaternion_to_euler(&bmi088->data.ekf_state.quaternion, &bmi088->data.ekf_state.euler);

        float gyro_z_avg = gyro_z_sum / count;
        bmi088->data.ekf_state.gyro_bias[2] = gyro_z_avg;
    }
}

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
    Imu_Temp_Init();

    Bmi088_config_t bmi088_config = {
        .spi_handle = &hspi1,
        .accel_cs_gpio_port = GPIOA,
        .accel_cs_gpio_pin = GPIO_PIN_4,
        .gyro_cs_gpio_port = GPIOB,
        .gyro_cs_gpio_pin = GPIO_PIN_0,
        .enable_accel_self_test = true,
        .enable_gyro_self_test = true,
        .accel_rotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        .gyro_rotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
    };

    Ekf_config_t ekf_config = {
        .process_noise_q = 10.0f,
        .gyro_bias_noise = 0.001f,
        .measurement_noise_r = 1000000.0f,
        .dt = 0.001f,
        .fading_factor = 0.9996f,
        .enable_bias_correction = true,
        .static_threshold = 0.3f,
        .enable_temp_compensation = false,
    };

    bmi088_device = Bmi088_device_init(&bmi088_config);
    if (bmi088_device != NULL)
    {
        Bmi088_ekf_init(bmi088_device, &ekf_config);
       // Imu_Wait_For_Temp(bmi088_device);
        Ins_calibrate_and_init(bmi088_device);
    }

    Imu_state_e published_state = (bmi088_device != NULL) ? bmi088_device->data.state : IMU_STATE_INIT;
    update_attitude_data(NULL, NULL, NULL, NULL, published_state);

    uint32_t loop_count = 0;
    uint32_t static_consistent_count = 0;

    // 误差累积器：用于平滑零偏修正
    // 只有当累积误差足够大时，才去修改真正的 bias
    float bias_accumulator = 0.0f;

    for (;;)
    {
        loop_count++;

        if (bmi088_device == NULL) { osDelay(10); continue; }

        if (loop_count % 20 == 0) {
            float current_temp = Bmi088_get_temperature(bmi088_device);
            Imu_Temp_Control(current_temp);
        }

        if (bmi088_device->last_error != NO_ERROR) {
            bmi088_device->data.state = IMU_STATE_ERROR;
            osDelay(1);
            continue;
        }

        Bmi088_error_e update_result = Bmi088_ekf_update(bmi088_device);

        if (update_result == NO_ERROR) {
            Acc_raw_data_t *acc_data = Read_acc_data(bmi088_device);
            Gyro_raw_data_t *gyro_data = Read_gyro_data(bmi088_device);
            Euler_angles_t *euler_angles = Bmi088_get_euler_angles(bmi088_device);

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

                // ================== [关键优化] 自适应动态零偏追踪 ==================
                float gyro_xy = sqrtf(gyro_body.roll * gyro_body.roll + gyro_body.pitch * gyro_body.pitch);
                float acc_diff = fabsf(sqrtf(acc_body.x*acc_body.x + acc_body.y*acc_body.y + acc_body.z*acc_body.z) - 9.81f);

                // 1. 严格的静态判定
                if (gyro_xy < 0.01f && acc_diff < 0.8f) {
                    static_consistent_count++;
                } else {
                    static_consistent_count = 0;
                    // 如果动了，清空累积器，防止把运动期间的偏差带入
                    bias_accumulator = 0.0f;
                }

                // 2. 连续静止 1 秒后开始计算
                if (static_consistent_count > 1000) {
                    float current_bias = bmi088_device->data.ekf_state.gyro_bias[2];
                    float raw_z = gyro_body.yaw;
                    float diff = raw_z - current_bias;

                    // 3. 核心：死区 + 累积修正
                    // 只有漂移超过“底噪阈值”(0.0005)时才开始累积
                    // 这个阈值比之前的小，因为我们现在有了累积器，不怕噪声
                    if (fabsf(diff) > 0.0005f && fabsf(diff) < 0.05f) {

                        // 将偏差累积起来
                        bias_accumulator += diff;

                        // 累积器阈值 (ACC_THRESHOLD)
                        // 只有当累积的误差超过一定值（比如 0.05）时，说明这是一个持续的偏差，不是噪声
                        // 此时才对 bias 进行一次微小的修正
                        const float ACC_THRESHOLD = 0.05f;
                        const float FIX_STEP = 0.0001f; // 每次修正的最小步长

                        if (bias_accumulator > ACC_THRESHOLD) {
                            bmi088_device->data.ekf_state.gyro_bias[2] += FIX_STEP;
                            bias_accumulator = 0.0f; // 清空累积器
                        }
                        else if (bias_accumulator < -ACC_THRESHOLD) {
                            bmi088_device->data.ekf_state.gyro_bias[2] -= FIX_STEP;
                            bias_accumulator = 0.0f; // 清空累积器
                        }
                    }
                }
                // ======================================================================

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

                static uint32_t print_count = 0;
                 {
                     float temp = Bmi088_get_temperature(bmi088_device);
                     // 打印 BiasZ 以便观察
                    /*
                     printf("IMU: R:%.2f P:%.2f Y:%.2f Temp:%.1f BiasZ:%.5f\r\n",
                            euler_angles->roll,
                            euler_angles->pitch,
                            euler_angles->yaw,
                            temp,
                            bmi088_device->data.ekf_state.gyro_bias[2]);
                            */
                }
            }
        } else {
            bmi088_device->data.state = IMU_STATE_ERROR;
            update_attitude_data(NULL, NULL, NULL, NULL, IMU_STATE_ERROR);
        }
        osDelay(1);
    }
}