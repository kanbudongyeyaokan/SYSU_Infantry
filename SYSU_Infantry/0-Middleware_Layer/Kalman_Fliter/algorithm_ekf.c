/**
 * @Author         : GitHub Copilot
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-17
 * @Note           : 扩展卡尔曼滤波算法实现
 * @Copyright(c)   : SYSU Copyright
 */
#include "algorithm_ekf.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// EKF相关常数定义
#define EKF_DEG_TO_RAD (3.14159265f / 180.0f)
#define EKF_RAD_TO_DEG (180.0f / 3.14159265f)
#define EKF_GRAVITY 9.80665f
#define EKF_STATIC_THRESHOLD_DEFAULT 0.2f
#define EKF_STATIC_COUNT_THRESHOLD 100

// 矩阵运算辅助函数声明
static void matrix_multiply_3x3(float a[3][3], float b[3][3], float result[3][3]);
static void matrix_multiply_7x7(float a[7][7], float b[7][7], float result[7][7]);
static void matrix_transpose_3x3(float src[3][3], float dst[3][3]);
static void matrix_inverse_3x3(float src[3][3], float dst[3][3]);
static void quaternion_normalize(Quaternion_t* q);
static void ekf_predict_step(Ekf_state_t* ekf_state, const float gyro[3]);
static void ekf_update_step(Ekf_state_t* ekf_state, const float acc[3]);

/**
 * @brief 初始化EKF参数和状态
 */
Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config) {
    if (ekf_state == NULL || ekf_config == NULL) {
        return EKF_INIT_ERROR;
    }
    
    // 初始化四元数为单位四元数
    ekf_state->quaternion.q0 = 1.0f;
    ekf_state->quaternion.q1 = 0.0f;
    ekf_state->quaternion.q2 = 0.0f;
    ekf_state->quaternion.q3 = 0.0f;
    
    // 初始化欧拉角为0
    ekf_state->euler.roll = 0.0f;
    ekf_state->euler.pitch = 0.0f;
    ekf_state->euler.yaw = 0.0f;
    
    // 初始化陀螺仪零偏为0
    ekf_state->gyro_bias[0] = 0.0f;
    ekf_state->gyro_bias[1] = 0.0f;
    ekf_state->gyro_bias[2] = 0.0f;
    
    // 初始化状态协方差矩阵P
    memset(ekf_state->P, 0, sizeof(ekf_state->P));
    for (int i = 0; i < 4; i++) {
        ekf_state->P[i][i] = 1.0f;  // 初始四元数部分不确定性
    }
    for (int i = 4; i < 7; i++) {
        ekf_state->P[i][i] = 0.01f;  // 初始零偏不确定性
    }
    
    // 初始化过程噪声协方差矩阵Q
    memset(ekf_state->Q, 0, sizeof(ekf_state->Q));
    for (int i = 0; i < 4; i++) {
        ekf_state->Q[i][i] = ekf_config->process_noise_q;
    }
    for (int i = 4; i < 7; i++) {
        ekf_state->Q[i][i] = ekf_config->gyro_bias_noise;
    }
    
    // 初始化测量噪声协方差矩阵R
    memset(ekf_state->R, 0, sizeof(ekf_state->R));
    for (int i = 0; i < 3; i++) {
        ekf_state->R[i][i] = ekf_config->measurement_noise_r;
    }
    
    // 初始化其他状态变量
    ekf_state->is_initialized = true;
    ekf_state->static_count = 0;
    ekf_state->is_static = false;
    
    return EKF_NO_ERROR;
}

/**
 * @brief 使用新的传感器数据更新EKF
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    if (ekf_state == NULL || acc == NULL || gyro == NULL) {
        return EKF_UPDATE_ERROR;
    }
    
    // 检查EKF是否已初始化
    if (!ekf_state->is_initialized) {
        return EKF_INIT_ERROR;
    }
    
    // 静态检测和零偏校准
    if (Ekf_detect_static_state(ekf_state, acc, gyro)) {
        // 在静态状态下更新零偏
        for (int i = 0; i < 3; i++) {
            ekf_state->gyro_bias[i] = 0.95f * ekf_state->gyro_bias[i] + 0.05f * gyro[i];
        }
    }
    
    // EKF预测步骤
    ekf_predict_step(ekf_state, gyro);
    
    // EKF更新步骤（使用加速度计数据）
    ekf_update_step(ekf_state, acc);
    
    // 更新欧拉角
    Ekf_quaternion_to_euler(&ekf_state->quaternion, &ekf_state->euler);
    
    return EKF_NO_ERROR;
}

/**
 * @brief 检测静态状态
 */
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    
    // 根据加速度模长和角速度模长判断是否处于静态状态
    float gravity_error = fabsf(acc_norm - EKF_GRAVITY);
    float threshold = EKF_STATIC_THRESHOLD_DEFAULT;
    
    if (gravity_error < threshold && gyro_norm < threshold) {
        ekf_state->static_count++;
        if (ekf_state->static_count > EKF_STATIC_COUNT_THRESHOLD) {
            ekf_state->is_static = true;
        }
    } else {
        ekf_state->static_count = 0;
        ekf_state->is_static = false;
    }
    
    return ekf_state->is_static;
}

/**
 * @brief 四元数转欧拉角
 */
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler) {
    // 归一化四元数
    quaternion_normalize(q);
    
    // 根据四元数计算欧拉角
    float q0q0 = q->q0 * q->q0;
    float q0q1 = q->q0 * q->q1;
    float q0q2 = q->q0 * q->q2;
    float q0q3 = q->q0 * q->q3;
    float q1q1 = q->q1 * q->q1;
    float q1q2 = q->q1 * q->q2;
    float q1q3 = q->q1 * q->q3;
    float q2q2 = q->q2 * q->q2;
    float q2q3 = q->q2 * q->q3;
    float q3q3 = q->q3 * q->q3;
    
    // 计算欧拉角（ZYX顺序）
    euler->roll = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3) * EKF_RAD_TO_DEG;
    euler->pitch = asinf(-2.0f * (q1q3 - q0q2)) * EKF_RAD_TO_DEG;
    euler->yaw = atan2f(2.0f * (q0q3 + q1q2), q0q0 + q1q1 - q2q2 - q3q3) * EKF_RAD_TO_DEG;
}

/**
 * @brief 归一化四元数
 */
static void quaternion_normalize(Quaternion_t* q) {
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        q->q0 *= inv_norm;
        q->q1 *= inv_norm;
        q->q2 *= inv_norm;
        q->q3 *= inv_norm;
    }
}

/**
 * @brief EKF预测步骤
 */
static void ekf_predict_step(Ekf_state_t* ekf_state, const float gyro[3]) {
    // 提取四元数
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;
    
    // 提取陀螺仪零偏
    float bx = ekf_state->gyro_bias[0];
    float by = ekf_state->gyro_bias[1];
    float bz = ekf_state->gyro_bias[2];
    
    // 计算陀螺仪补偿后的角速度
    float wx = gyro[0] - bx;
    float wy = gyro[1] - by;
    float wz = gyro[2] - bz;
    
    // 计算四元数导数
    float dq0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    float dq1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    float dq2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    float dq3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);
    
    // 使用Euler积分更新四元数
    float dt = 0.001f; // 假设采样周期为1ms，实际应该通过配置参数获取
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    
    // 归一化四元数
    float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= qnorm;
    q1 /= qnorm;
    q2 /= qnorm;
    q3 /= qnorm;
    
    // 更新四元数
    ekf_state->quaternion.q0 = q0;
    ekf_state->quaternion.q1 = q1;
    ekf_state->quaternion.q2 = q2;
    ekf_state->quaternion.q3 = q3;
    
    // 更新状态协方差矩阵P (此处简化处理)
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ekf_state->P[i][j] += ekf_state->Q[i][j] * dt;
        }
    }
}

/**
 * @brief EKF更新步骤
 */
static void ekf_update_step(Ekf_state_t* ekf_state, const float acc[3]) {
    // 提取四元数
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;
    
    // 计算重力在机体坐标系下的预测值
    float ax_pred = 2.0f * (q1 * q3 - q0 * q2) * (-EKF_GRAVITY);
    float ay_pred = 2.0f * (q0 * q1 + q2 * q3) * (-EKF_GRAVITY);
    float az_pred = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (-EKF_GRAVITY);
    
    // 计算测量残差
    float ex = acc[0] - ax_pred;
    float ey = acc[1] - ay_pred;
    float ez = acc[2] - az_pred;
    
    // 简化的观测矩阵H和卡尔曼增益K
    // 实际应该计算完整的H矩阵和卡尔曼增益K
    // 这里使用简化版本进行示例
    float Kx = 0.01f; // 简化的卡尔曼增益系数
    float Ky = 0.01f;
    float Kz = 0.01f;
    
    // 应用测量更新
    q0 += Kx * ex;
    q1 += Ky * ey;
    q2 += Kz * ez;
    q3 += 0.0f;
    
    // 归一化四元数
    float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= qnorm;
    q1 /= qnorm;
    q2 /= qnorm;
    q3 /= qnorm;
    
    // 更新四元数
    ekf_state->quaternion.q0 = q0;
    ekf_state->quaternion.q1 = q1;
    ekf_state->quaternion.q2 = q2;
    ekf_state->quaternion.q3 = q3;
    
    // 此处应该更新状态协方差矩阵P，但这里简化处理
}

/**
 * @brief 矩阵乘法 3x3
 */
static void matrix_multiply_3x3(float a[3][3], float b[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 矩阵乘法 7x7
 */
static void matrix_multiply_7x7(float a[7][7], float b[7][7], float result[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 矩阵转置 3x3
 */
static void matrix_transpose_3x3(float src[3][3], float dst[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dst[j][i] = src[i][j];
        }
    }
}

/**
 * @brief 矩阵求逆 3x3
 */
static void matrix_inverse_3x3(float src[3][3], float dst[3][3]) {
    float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
                src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
                src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);
    
    float inv_det = 1.0f / det;
    
    dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) * inv_det;
    dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) * inv_det;
    dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) * inv_det;
    dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) * inv_det;
    dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) * inv_det;
    dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) * inv_det;
    dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) * inv_det;
    dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) * inv_det;
    dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) * inv_det;
}