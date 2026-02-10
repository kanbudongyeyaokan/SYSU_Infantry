/**
 * @Author         : SYSU电控组
 * @Date           : 2025-09-29
 * @Note           : 适配现有接口
 * 核心策略：6状态EKF (4四元数+2零偏)，Z轴零偏不估计，卡方检验抗干扰
 */
#include "algorithm_ekf.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 常量定义
#define EKF_GRAVITY 9.80665f
#define EKF_DEG_TO_RAD (3.14159265f / 180.0f)
#define EKF_RAD_TO_DEG (180.0f / 3.14159265f)

// 移植自QuaternionEKF的参数
#define CHI_SQUARE_THRESHOLD_DEFAULT 1e-8f
#define ERROR_COUNT_MAX 50

// 私有函数声明
static float inv_sqrt(float x);
static void matrix_multiply(float* A, float* B, float* C, int m, int n, int p);
static void matrix_transpose(float* A, float* B, int m, int n);
static int matrix_inverse_3x3(float* src, float* dst);

// 状态转移矩阵F (6x6) 的初始值
const float F_init[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

// 初始协方差矩阵P (6x6)
// 前4个对角元素(四元数)设置为大值100000,表示初始姿态高度不确定
// 后2个对角元素(零偏)设置为100,表示初始零偏有一定不确定性
const float P_init[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100
};

/**
 * @brief 初始化EKF
 */
Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config) {
    if (ekf_state == NULL || ekf_config == NULL) {
        return EKF_INIT_ERROR;
    }

    // 1. 初始化参数
    ekf_state->Q1 = ekf_config->process_noise_q;      // 四元数过程噪声 (建议 10)
    ekf_state->Q2 = ekf_config->gyro_bias_noise;      // 零偏过程噪声 (建议 0.001)
    ekf_state->R_val = ekf_config->measurement_noise_r; // 测量噪声 (建议 1000000)
    ekf_state->lambda = ekf_config->fading_factor;    // 渐减因子 (建议 0.9996)

    // 2. 初始化状态
    // 单位四元数表示初始姿态为无旋转(机体系与导航系重合)
    ekf_state->quaternion.q0 = 1.0f;
    ekf_state->quaternion.q1 = 0.0f;
    ekf_state->quaternion.q2 = 0.0f;
    ekf_state->quaternion.q3 = 0.0f;

    ekf_state->gyro_bias[0] = 0.0f;
    ekf_state->gyro_bias[1] = 0.0f;
    ekf_state->gyro_bias[2] = 0.0f;

    ekf_state->euler.roll = 0.0f;
    ekf_state->euler.pitch = 0.0f;
    ekf_state->euler.yaw = 0.0f;

    // 3. 初始化矩阵
    memcpy(ekf_state->P, P_init, sizeof(P_init));

    // 4. 初始化标志位
    ekf_state->converge_flag = false;
    ekf_state->stable_flag = false;
    ekf_state->error_count = 0;
    ekf_state->update_count = 0;
    ekf_state->chi_square = 0.0f;
    // 航向角连续化相关变量,用于处理±180°跳变
    ekf_state->yaw_round_count = 0;
    ekf_state->yaw_angle_last = 0.0f;
    ekf_state->yaw_total_angle = 0.0f;

    ekf_state->is_initialized = true;

    return EKF_NO_ERROR;
}

/**
 * @brief EKF更新函数 (核心算法移植)
 *  使用陀螺仪角速度预测下一时刻姿态和协方差
 *  使用加速度计测量值修正预测姿态
 *  卡方检验判断测量可靠性,自适应调整卡尔曼增益
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt) {
    if (!ekf_state->is_initialized) return EKF_INIT_ERROR;

    float acc_norm_val, gyro_norm_val;
    float half_T = 0.5f * dt;

    // [关键修改] 删除"强制去除Z轴零偏"的代码
    // 我们将通过Ins_task在启动时计算Z轴静态零偏，并填入 gyro_bias[2]
    // 之后该值保持不变（因为dx[3]和dx[5]逻辑不更新它），从而持续扣除静态漂移
    // ekf_state->gyro_bias[2] = 0.0f; <--- 删除这行

    // 1. 去除零偏后的角速度
    float gx = gyro[0] - ekf_state->gyro_bias[0];
    float gy = gyro[1] - ekf_state->gyro_bias[1];
    float gz = gyro[2] - ekf_state->gyro_bias[2];

    // 2. 计算模长用于判断运动状态
    gyro_norm_val = sqrtf(gx*gx + gy*gy + gz*gz);
    acc_norm_val = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

    // 判断稳定性 (用于卡方检验逻辑)
    // 满足条件认为设备静止,此时加速度计测量最可靠
    if (gyro_norm_val < 0.3f && acc_norm_val > 9.3f && acc_norm_val < 10.3f) {
        ekf_state->stable_flag = true;
    } else {
        ekf_state->stable_flag = false;
    }

    // 归一化加速度 (量测值)
    float norm_acc[3];
    if (acc_norm_val > 1e-4f) {
        float inv_norm = 1.0f / acc_norm_val;
        norm_acc[0] = acc[0] * inv_norm;
        norm_acc[1] = acc[1] * inv_norm;
        norm_acc[2] = acc[2] * inv_norm;
    } else {
        return EKF_UPDATE_ERROR;
    }

    // ================== 预测步骤 (Predict) ==================

    // 3. 构建状态转移矩阵 F (6x6)
    // 左上角 4x4 是四元数运动学方程的离散化
    float F[36];
    memcpy(F, F_init, sizeof(F_init)); // 初始化为单位阵+零矩阵

    float hgx = -gx * half_T;
    float hgy = -gy * half_T;
    float hgz = -gz * half_T;

    // F[0~3][0~3]
    F[0*6+1] = hgx; F[0*6+2] = hgy; F[0*6+3] = hgz;
    F[1*6+0] = -hgx; F[1*6+2] = hgz; F[1*6+3] = -hgy;
    F[2*6+0] = -hgy; F[2*6+1] = -hgz; F[2*6+3] = hgx;
    F[3*6+0] = -hgz; F[3*6+1] = hgy; F[3*6+2] = -hgx;

    // F[0~3][4~5] (四元数对零偏的偏导，注意这里只推导了x,y零偏)
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;

    F[0*6+4] = q1 * half_T; F[0*6+5] = q2 * half_T;
    F[1*6+4] = -q0 * half_T; F[1*6+5] = q3 * half_T;
    F[2*6+4] = -q3 * half_T; F[2*6+5] = -q0 * half_T;
    F[3*6+4] = q2 * half_T; F[3*6+5] = -q1 * half_T;

    // 4. 状态预测 x_k = F * x_{k-1}
    // 这里使用简单的四元数积分：q_new = q + 0.5 * q * omega * dt
    float q_new[4];
    q_new[0] = q0 + (-q1*gx - q2*gy - q3*gz) * half_T;
    q_new[1] = q1 + ( q0*gx - q3*gy + q2*gz) * half_T;
    q_new[2] = q2 + ( q3*gx + q0*gy - q1*gz) * half_T;
    q_new[3] = q3 + (-q2*gx + q1*gy + q0*gz) * half_T;

    // 归一化预测四元数
    float q_norm = inv_sqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    ekf_state->quaternion.q0 = q_new[0] * q_norm;
    ekf_state->quaternion.q1 = q_new[1] * q_norm;
    ekf_state->quaternion.q2 = q_new[2] * q_norm;
    ekf_state->quaternion.q3 = q_new[3] * q_norm;

    // 更新本地变量供后续使用
    q0 = ekf_state->quaternion.q0;
    q1 = ekf_state->quaternion.q1;
    q2 = ekf_state->quaternion.q2;
    q3 = ekf_state->quaternion.q3;

    // 5. 协方差预测 P = F*P*F' + Q
    float P[36];
    memcpy(P, ekf_state->P, sizeof(P));

    float FP[36]; // F * P
    matrix_multiply(F, P, FP, 6, 6, 6);

    float FT[36]; // F'
    matrix_transpose(F, FT, 6, 6);

    float P_pred[36]; // FP * FT
    matrix_multiply(FP, FT, P_pred, 6, 6, 6);

    // 加上过程噪声 Q
    // Q1对四元数，Q2对零偏
    float dtQ1 = ekf_state->Q1 * dt;
    float dtQ2 = ekf_state->Q2 * dt;

    P_pred[0*6+0] += dtQ1; P_pred[1*6+1] += dtQ1; P_pred[2*6+2] += dtQ1; P_pred[3*6+3] += dtQ1;
    P_pred[4*6+4] += dtQ2; P_pred[5*6+5] += dtQ2;

    // [移植点 2] Fading Filter (防止零偏方差过度收敛)
    P_pred[4*6+4] /= ekf_state->lambda;
    P_pred[5*6+5] /= ekf_state->lambda;

    // 限幅
    if(P_pred[4*6+4] > 10000.0f) P_pred[4*6+4] = 10000.0f;
    if(P_pred[5*6+5] > 10000.0f) P_pred[5*6+5] = 10000.0f;


    // ================== 更新步骤 (Update) ==================

    // 6. 观测矩阵 H (3x6)
    // h(x)是将重力向量[0,0,1]旋转到机体坐标系
    float H[18] = {0}; // 3x6
    float dx0 = 2*q0, dx1 = 2*q1, dx2 = 2*q2, dx3 = 2*q3;

    H[0*6+0] = -dx2; H[0*6+1] = dx3;  H[0*6+2] = -dx0; H[0*6+3] = dx1;
    H[1*6+0] = dx1;  H[1*6+1] = dx0;  H[1*6+2] = dx3;  H[1*6+3] = dx2;
    H[2*6+0] = dx0;  H[2*6+1] = -dx1; H[2*6+2] = -dx2; H[2*6+3] = dx3;
    // 后两列对零偏的偏导为0，因为加速度计测量模型与陀螺仪零偏无直接关系

    // 7. 计算残差 y = z - h(x)
    float g_pred[3];
    g_pred[0] = 2*(q1*q3 - q0*q2);
    g_pred[1] = 2*(q0*q1 + q2*q3);
    g_pred[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float y[3];
    y[0] = norm_acc[0] - g_pred[0];
    y[1] = norm_acc[1] - g_pred[1];
    y[2] = norm_acc[2] - g_pred[2];

    // 8. 计算 S = H*P*H' + R
    float HP[18]; // 3x6
    matrix_multiply(H, P_pred, HP, 3, 6, 6);

    float HT[18]; // 6x3
    matrix_transpose(H, HT, 3, 6);

    float S[9]; // 3x3
    matrix_multiply(HP, HT, S, 3, 6, 3);
    // 加上测量噪声R(对角阵,三个轴相同)
    S[0] += ekf_state->R_val;
    S[4] += ekf_state->R_val;
    S[8] += ekf_state->R_val;

    // 9. 计算 S逆
    float S_inv[9];
    if (matrix_inverse_3x3(S, S_inv) != 0) {
        return EKF_UPDATE_ERROR; // 矩阵奇异
    }

    // [移植点 3] 卡方检验 (Chi-Square Test)
    // chi = y' * S_inv * y
    float S_inv_y[3];
    matrix_multiply(S_inv, y, S_inv_y, 3, 3, 1); // 3x3 * 3x1 = 3x1

    float chi_square = 0.0f;
    for(int i=0; i<3; i++) chi_square += y[i] * S_inv_y[i];
    ekf_state->chi_square = chi_square;

    // 自适应增益系数
    float gain_scale = 1.0f;
    float threshold = CHI_SQUARE_THRESHOLD_DEFAULT;

    if (chi_square < 0.5f * threshold) {
        ekf_state->converge_flag = true;
    }

    if (chi_square > threshold && ekf_state->converge_flag) {
        if (ekf_state->stable_flag) {
            ekf_state->error_count++;
        } else {
            ekf_state->error_count = 0;
        }

        if (ekf_state->error_count > ERROR_COUNT_MAX) {
            // 滤波器发散，重置收敛标志，允许更新
            ekf_state->converge_flag = false;
        } else {
            // 误差过大但未发散，拒绝本次测量更新，仅保留预测结果
            memcpy(ekf_state->P, P_pred, sizeof(P));
            return EKF_NO_ERROR;
        }
    } else {
        ekf_state->error_count = 0;
        if (chi_square > 0.1f * threshold && ekf_state->converge_flag) {
            // 介于阈值之间，进行增益衰减
            gain_scale = (threshold - chi_square) / (0.9f * threshold);
            if (gain_scale < 0.0f) gain_scale = 0.0f; // 保护
        }
    }

    // 10. 计算卡尔曼增益 K = P*H'*S_inv
    float PHT[18]; // 6x3
    matrix_multiply(P_pred, HT, PHT, 6, 6, 3);

    float K[18]; // 6x3
    matrix_multiply(PHT, S_inv, K, 6, 3, 3);

    // [移植点 4] 应用自适应增益和方向余弦修正
    for(int i=0; i<18; i++) K[i] *= gain_scale;
    // 当重力矢量与某轴夹角较大时,该轴的加速度计信息较弱,降低增益
    for(int i=4; i<6; i++) { // 对零偏部分(x,y)进行修正
        for(int j=0; j<3; j++) {
             float cos_val = fabsf(g_pred[i-4]);
             float angle = acosf(cos_val);
             K[i*3+j] *= angle / 1.5707963f;
        }
    }

    // 11. 更新状态 x = x + K*y
    float dx[6];
    matrix_multiply(K, y, dx, 6, 3, 1);

    // 限制零偏修正幅度
    if (ekf_state->converge_flag) {
        float limit = 1e-2f * dt;
        if (dx[4] > limit) dx[4] = limit;
        if (dx[4] < -limit) dx[4] = -limit;
        if (dx[5] > limit) dx[5] = limit;
        if (dx[5] < -limit) dx[5] = -limit;
    }

    // 不更新Z轴零偏 (仅依靠外部校准)
    dx[3] = 0.0f;

    ekf_state->quaternion.q0 += dx[0];
    ekf_state->quaternion.q1 += dx[1];
    ekf_state->quaternion.q2 += dx[2];
    ekf_state->quaternion.q3 += dx[3];

    ekf_state->gyro_bias[0] += dx[4];
    ekf_state->gyro_bias[1] += dx[5];
    // Z轴零偏不更新

    // 再次归一化
    q_norm = inv_sqrt(ekf_state->quaternion.q0*ekf_state->quaternion.q0 +
                      ekf_state->quaternion.q1*ekf_state->quaternion.q1 +
                      ekf_state->quaternion.q2*ekf_state->quaternion.q2 +
                      ekf_state->quaternion.q3*ekf_state->quaternion.q3);
    ekf_state->quaternion.q0 *= q_norm;
    ekf_state->quaternion.q1 *= q_norm;
    ekf_state->quaternion.q2 *= q_norm;
    ekf_state->quaternion.q3 *= q_norm;

    // 12. 更新协方差 P = (I - KH) * P_pred
    float KH[36];
    matrix_multiply(K, H, KH, 6, 3, 6);

    float I_KH[36];
    for(int i=0; i<36; i++) I_KH[i] = -KH[i];
    for(int i=0; i<6; i++) I_KH[i*6+i] += 1.0f;

    matrix_multiply(I_KH, P_pred, ekf_state->P, 6, 6, 6);

    // ================== 输出转换 ==================
    // 将四元数转换为欧拉角输出
    Ekf_quaternion_to_euler(&ekf_state->quaternion, &ekf_state->euler);
    // 更新连续航向角(处理±180°跳变)
    Ekf_update_yaw_continuity(ekf_state);

    ekf_state->update_count++;

    return EKF_NO_ERROR;
}

// 辅助函数：检测静态
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    // 移植后的算法使用内部的 stable_flag，此函数仅作为接口保留
    return ekf_state->stable_flag;
}

// 辅助函数：四元数转欧拉角
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;

    // ZYX顺序
    euler->yaw = atan2f(2.0f * (q0*q3 + q1*q2), 2.0f * (q0*q0 + q1*q1) - 1.0f) * EKF_RAD_TO_DEG;
    euler->pitch = atan2f(2.0f * (q0*q1 + q2*q3), 2.0f * (q0*q0 + q3*q3) - 1.0f) * EKF_RAD_TO_DEG;
    euler->roll = asinf(-2.0f * (q1*q3 - q0*q2)) * EKF_RAD_TO_DEG;
}

void Ekf_update_yaw_continuity(Ekf_state_t* ekf_state) {
    float yaw = ekf_state->euler.yaw;
    float last_yaw = ekf_state->yaw_angle_last;

    if (yaw - last_yaw > 180.0f) {
        ekf_state->yaw_round_count--;
    } else if (yaw - last_yaw < -180.0f) {
        ekf_state->yaw_round_count++;
    }

    ekf_state->yaw_total_angle = 360.0f * ekf_state->yaw_round_count + yaw;
    ekf_state->yaw_angle_last = yaw;
}

// 矩阵运算辅助函数
// 矩阵乘法: C = A * B
static void matrix_multiply(float* A, float* B, float* C, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += A[i*n + k] * B[k*p + j];
            }
            C[i*p + j] = sum;
        }
    }
}
// 矩阵转置: B = A'
static void matrix_transpose(float* A, float* B, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            B[j*m + i] = A[i*n + j];
        }
    }
}
// 3×3矩阵求逆
static int matrix_inverse_3x3(float* src, float* dst) {
    float det = src[0]*(src[4]*src[8] - src[5]*src[7]) -
                src[1]*(src[3]*src[8] - src[5]*src[6]) +
                src[2]*(src[3]*src[7] - src[4]*src[6]);

    if (fabsf(det) < 1e-8f) return -1;

    float inv_det = 1.0f / det;

    dst[0] = (src[4]*src[8] - src[5]*src[7]) * inv_det;
    dst[1] = (src[2]*src[7] - src[1]*src[8]) * inv_det;
    dst[2] = (src[1]*src[5] - src[2]*src[4]) * inv_det;
    dst[3] = (src[5]*src[6] - src[3]*src[8]) * inv_det;
    dst[4] = (src[0]*src[8] - src[2]*src[6]) * inv_det;
    dst[5] = (src[2]*src[3] - src[0]*src[5]) * inv_det;
    dst[6] = (src[3]*src[7] - src[4]*src[6]) * inv_det;
    dst[7] = (src[1]*src[6] - src[0]*src[7]) * inv_det;
    dst[8] = (src[0]*src[4] - src[1]*src[3]) * inv_det;

    return 0;
}
// 快速平方根倒数算法
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 占位函数，保持接口兼容
void Ekf_set_temperature(Ekf_state_t* ekf_state, float current_temp) { (void)ekf_state; (void)current_temp; }
void Ekf_calculate_temp_compensation(const Ekf_state_t* ekf_state, float temp_compensation[3]) {
    temp_compensation[0] = temp_compensation[1] = temp_compensation[2] = 0.0f;
}