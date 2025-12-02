#include "ins.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "main.h"
#include "bsp_dwt.h"
#include "math_lib.h"

// 双缓冲姿态数据
static attitude_t g_attitude_buffer[2];
static volatile uint8_t g_active_buffer_index = 0U;

static float g_yaw_total_deg = 0.0f;
static int32_t g_yaw_round_count = 0;
static bool g_yaw_total_valid = false;
static uint64_t g_last_update_timestamp_us = 0ULL;
static Imu_state_e g_ins_state = IMU_STATE_INIT;

/**
 * @brief 获取最新的姿态数据
 * @return 返回一个指向全局姿态数据结构体的常量指针
 */
attitude_t* get_attitude_data(void)
{
    return (attitude_t *)&g_attitude_buffer[g_active_buffer_index];
}

/**
 * @brief 更新姿态数据（由Ins_task调用）
 * @param acc 最新的加速度数据
 * @param gyro 最新的陀螺仪数据
 * @param euler 最新的欧拉角数据
 */
void update_attitude_data(const Acc_raw_data_t* acc,
                          const Gyro_raw_data_t* gyro,
                          const Euler_angles_t* euler,
                          const float* yaw_total_angle,
                          Imu_state_e state)
{
    uint8_t inactive_index = g_active_buffer_index ^ 1U;
    attitude_t *target = &g_attitude_buffer[inactive_index];
    float dt = 0.001f; // 默认1ms
    uint64_t now_us;

    g_ins_state = state;
    target->state = g_ins_state;

    if (g_ins_state == IMU_STATE_INIT || g_ins_state == IMU_STATE_ERROR) {
        g_yaw_total_valid = false;
    }

    now_us = DWT_GetTimeline_s();
    if (g_last_update_timestamp_us != 0ULL) {
        uint64_t delta_us = now_us - g_last_update_timestamp_us;
        if (delta_us < 1000000ULL) {
            dt = (float)delta_us * 1.0e-6f;
        }
    }
    g_last_update_timestamp_us = now_us;

    if (acc) {
        memcpy(&target->accel_raw, acc, sizeof(Acc_raw_data_t));
    }
    if (gyro) {
        target->gyro_raw.roll = RAD_TO_DEG(gyro->roll);
        target->gyro_raw.pitch = RAD_TO_DEG(gyro->pitch);
        target->gyro_raw.yaw = RAD_TO_DEG(gyro->yaw);
        target->yaw_rate_dps = target->gyro_raw.yaw;
    }
    if (euler) {
        float current_yaw;
        bool has_external_total;

        memcpy(&target->euler_angles, euler, sizeof(Euler_angles_t));

        current_yaw = euler->yaw;
        has_external_total = (yaw_total_angle != NULL) && isfinite(*yaw_total_angle);

        if (has_external_total) {
            g_yaw_total_deg = *yaw_total_angle;
            g_yaw_round_count = (int32_t)floorf((g_yaw_total_deg + 180.0f) / 360.0f);
            g_yaw_total_valid = true;
        } else if (!g_yaw_total_valid) {
            g_yaw_total_deg = current_yaw;
            g_yaw_round_count = (int32_t)floorf((g_yaw_total_deg + 180.0f) / 360.0f);
        }
    }

    target->yaw_round_count = g_yaw_round_count;
    target->yaw_total_angle = g_yaw_total_deg;
    if (!gyro) {
        target->yaw_rate_dps = target->gyro_raw.yaw;
    }

    target->state = g_ins_state;

    __disable_irq();
    g_active_buffer_index = inactive_index;
    __enable_irq();
}

Imu_state_e ins_get_state(void)
{
    return g_ins_state;
}