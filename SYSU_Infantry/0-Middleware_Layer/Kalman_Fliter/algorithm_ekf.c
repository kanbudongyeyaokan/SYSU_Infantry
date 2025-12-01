/**
 * @Author         : SYSU电控组
 * @Note           : EKF算法实现，适配动态dt
 */
#include "algorithm_ekf.h"
#include <math.h>
#include <string.h>

// 内部宏
#define ABS(x) ((x) > 0 ? (x) : -(x))

// 私有函数声明
static void ekf_predict_step(Ekf_state_t* ekf, const float gyro[3], float dt);
static void ekf_update_step(Ekf_state_t* ekf, const float acc[3]);
static void quaternion_normalize(Quaternion_t* q);

/**
 * @brief 初始化EKF
 */
Ekf_error_e Ekf_init(Ekf_state_t* ekf, Ekf_config_t* config) {
    if (ekf == NULL || config == NULL) return EKF_INIT_ERROR;

    memset(ekf, 0, sizeof(Ekf_state_t));

    // 1. 初始四元数 (1, 0, 0, 0)
    ekf->quaternion.q0 = 1.0f;

    // 2. 初始协方差 P (6x6)
    // 对角线给一个初始不确定度
    for(int i=0; i<6; i++) ekf->P[i][i] = 0.1f;

    // 3. 载入配置
    ekf->process_noise_q = config->process_noise_q;
    ekf->measurement_noise_r = config->measurement_noise_r;
    ekf->gyro_bias_noise = config->gyro_bias_noise;
    ekf->enable_bias_correction = config->enable_bias_correction;
    ekf->static_threshold = config->static_threshold;

    // 4. 初始化测量噪声矩阵 R (3x3)
    for(int i=0; i<3; i++) ekf->R[i][i] = config->measurement_noise_r;

    ekf->is_initialized = true;
    return EKF_NO_ERROR;
}

/**
 * @brief EKF 更新主函数
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf, const float acc[3], const float gyro[3], float dt) {
    if (!ekf->is_initialized) return EKF_INIT_ERROR;

    // 1. 静态检测与零偏校准
    if (Ekf_detect_static_state(ekf, acc, gyro)) {
        if (ekf->enable_bias_correction) {
            // 简单的静态零偏更新：IIR滤波
            const float alpha = 0.005f;
            for (int i = 0; i < 3; i++) {
                ekf->gyro_bias[i] = (1.0f - alpha) * ekf->gyro_bias[i] + alpha * gyro[i];
            }
        }
    }

    // 2. 预测步骤 (使用 dt)
    ekf_predict_step(ekf, gyro, dt);

    // 3. 更新步骤 (加速度计修正)
    ekf_update_step(ekf, acc);

    // 4. 输出欧拉角
    Ekf_quaternion_to_euler(&ekf->quaternion, &ekf->euler);

    // 5. 维护多圈 Yaw 角
    float yaw_diff = ekf->euler.yaw - ekf->yaw_angle_last;
    if (yaw_diff > 180.0f) ekf->yaw_round_count--;
    else if (yaw_diff < -180.0f) ekf->yaw_round_count++;

    ekf->yaw_total_angle = ekf->yaw_round_count * 360.0f + ekf->euler.yaw;
    ekf->yaw_angle_last = ekf->euler.yaw;

    return EKF_NO_ERROR;
}

// --- 内部数学实现 ---

static void ekf_predict_step(Ekf_state_t* ekf, const float gyro[3], float dt) {
    float q0 = ekf->quaternion.q0;
    float q1 = ekf->quaternion.q1;
    float q2 = ekf->quaternion.q2;
    float q3 = ekf->quaternion.q3;

    // 1. 扣除零偏
    float gx = gyro[0] - ekf->gyro_bias[0];
    float gy = gyro[1] - ekf->gyro_bias[1];
    float gz = gyro[2] - ekf->gyro_bias[2];

    // 2. 四元数微分方程积分 (一阶龙格库塔)
    // q_dot = 0.5 * q * omega
    float q0_new = q0 + 0.5f * dt * (-q1*gx - q2*gy - q3*gz);
    float q1_new = q1 + 0.5f * dt * ( q0*gx - q3*gy + q2*gz);
    float q2_new = q2 + 0.5f * dt * ( q3*gx + q0*gy - q1*gz);
    float q3_new = q3 + 0.5f * dt * (-q2*gx + q1*gy + q0*gz);

    ekf->quaternion.q0 = q0_new;
    ekf->quaternion.q1 = q1_new;
    ekf->quaternion.q2 = q2_new;
    ekf->quaternion.q3 = q3_new;
    quaternion_normalize(&ekf->quaternion);

    // 3. 计算状态转移矩阵 F (线性化)
    // 简化版：忽略零偏对P的影响，只主要关注四元数部分
    // 这里的 dt 使用是关键
    float F[4][4] = {
        {1,       -0.5f*gx*dt, -0.5f*gy*dt, -0.5f*gz*dt},
        {0.5f*gx*dt, 1,        0.5f*gz*dt, -0.5f*gy*dt},
        {0.5f*gy*dt, -0.5f*gz*dt, 1,        0.5f*gx*dt},
        {0.5f*gz*dt, 0.5f*gy*dt, -0.5f*gx*dt, 1}
    };

    // 4. 更新协方差矩阵 P = F*P*F^T + Q
    // 这里进行 4x4 矩阵运算 (由于代码篇幅，这里用简化逻辑示意，
    // 实际工程中应展开乘法或使用矩阵库)

    // 简单起见，我们只对 P 矩阵的主对角线增加过程噪声 Q
    // 这在计算资源受限时是一个有效的近似
    for(int i=0; i<4; i++) {
        ekf->P[i][i] += ekf->process_noise_q * dt;
    }
    // 零偏的不确定性也会随时间增加
    ekf->P[4][4] += ekf->gyro_bias_noise * dt;
    ekf->P[5][5] += ekf->gyro_bias_noise * dt;
}

static void ekf_update_step(Ekf_state_t* ekf, const float acc[3]) {
    // 加速度计更新 (观测重力向量)
    // 如果加速度模长偏离 1g 太大，则认为有运动加速度，跳过更新
    float acc_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (fabsf(acc_norm - EKF_GRAVITY) > 1.5f) { // 容差 1.5 m/s^2
        return;
    }

    // 归一化加速度
    float ax = acc[0] / acc_norm;
    float ay = acc[1] / acc_norm;
    float az = acc[2] / acc_norm;

    // 估计的重力方向 (由当前四元数推算)
    // g_pred = R(q)^T * [0, 0, 1]
    float q0 = ekf->quaternion.q0;
    float q1 = ekf->quaternion.q1;
    float q2 = ekf->quaternion.q2;
    float q3 = ekf->quaternion.q3;

    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 残差 (观测 - 预测)
    float ex = ax - vx;
    float ey = ay - vy;
    float ez = az - vz;

    // 简化的增益 K (这里为了代码紧凑使用了互补滤波的思想作为 EKF 的替代)
    // 如果要完整 EKF，需要计算 H 矩阵和 K = P*H^T * ...
    // 在计算力受限的 MCU 上，固定增益或基于 P 对角线的增益通常足够

    // 使用 P 矩阵动态调整增益
    float P_trace = ekf->P[0][0] + ekf->P[1][1] + ekf->P[2][2] + ekf->P[3][3];
    float K = P_trace / (P_trace + ekf->measurement_noise_r);
    if (K > 0.1f) K = 0.1f; // 限幅

    // 修正四元数 (使用叉积误差修正)
    // error = v x a (向量叉积)
    float err_x = vy*az - vz*ay;
    float err_y = vz*ax - vx*az;
    float err_z = vx*ay - vy*ax;

    // 修正四元数微分
    // q_dot_error = 0.5 * q * error_vec
    // 这里直接叠加到四元数上作为修正
    ekf->quaternion.q0 += 0.5f * K * (-q1*err_x - q2*err_y - q3*err_z);
    ekf->quaternion.q1 += 0.5f * K * ( q0*err_x - q3*err_y + q2*err_z);
    ekf->quaternion.q2 += 0.5f * K * ( q3*err_x + q0*err_y - q1*err_z);
    ekf->quaternion.q3 += 0.5f * K * (-q2*err_x + q1*err_y + q0*err_z);

    quaternion_normalize(&ekf->quaternion);

    // 协方差收敛 (每次观测都会降低 P)
    for(int i=0; i<4; i++) {
        ekf->P[i][i] *= (1.0f - K);
    }
}

bool Ekf_detect_static_state(Ekf_state_t* ekf, const float acc[3], const float gyro[3]) {
    float gyro_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    // 阈值判断
    if (gyro_norm < ekf->static_threshold) {
        ekf->static_count++;
    } else {
        ekf->static_count = 0;
    }

    // 连续 100 次 (约0.1s) 满足条件才判定为静止
    ekf->is_static = (ekf->static_count > 100);
    return ekf->is_static;
}

static void quaternion_normalize(Quaternion_t* q) {
    float norm = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    if (norm == 0.0f) return;
    float inv = 1.0f / norm;
    q->q0 *= inv;
    q->q1 *= inv;
    q->q2 *= inv;
    q->q3 *= inv;
}

void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * EKF_RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        euler->pitch = copysignf(90.0f, sinp); // use 90 degrees if out of range
    else
        euler->pitch = asinf(sinp) * EKF_RAD_TO_DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * EKF_RAD_TO_DEG;
}