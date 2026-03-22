/**
 * @Author         : 优化版 EKF
 * @Note           : 基于统计信号处理的姿态最优估计
 */

// ================= 强制最高级优化指令 =================
#pragma GCC optimize ("O3")
#pragma GCC optimize ("-ffast-math")
// ====================================================

#include "algorithm_ekf.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 常量定义
#define EKF_GRAVITY 9.80665f
#define EKF_DEG_TO_RAD (3.14159265f / 180.0f)
#define EKF_RAD_TO_DEG (180.0f / 3.14159265f)

// 统计检验参数
#define CHI_SQUARE_THRESHOLD_DEFAULT 1e-8f
#define ERROR_COUNT_MAX 50

// 信号统计特征阈值 (已优化)
#define EKF_STATIC_GYRO_NORM_THRESH      0.05f   // 【修改点】严格的静止判定阈值 (约1.1度/秒)
#define EKF_STATIC_ACC_LOW               9.0f
#define EKF_STATIC_ACC_HIGH              10.0f
#define EKF_Z_BIAS_TIME_CONSTANT_S       0.8f
#define EKF_Z_BIAS_ALPHA_MIN             0.001f
#define EKF_Z_BIAS_ALPHA_MAX             0.03f
#define EKF_Z_BIAS_DEADBAND_RAD_S        0.0005f
#define EKF_Z_BIAS_LIMIT_RAD_S           0.20f
#define EKF_Z_ERR_LP_TIME_CONSTANT_S     0.2f

// 私有函数声明
static float inv_sqrt(float x);
static void matrix_multiply(float* A, float* B, float* C, int m, int n, int p);
static void matrix_transpose(float* A, float* B, int m, int n);
static int matrix_inverse_3x3(float* src, float* dst);

// 状态转移矩阵雅可比 (Jacobian) 初始值
const float F_init[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

// 初始先验误差协方差矩阵 P
const float P_init[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100
};

Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config) {
    if (ekf_state == NULL || ekf_config == NULL) return EKF_INIT_ERROR;

    ekf_state->Q1 = ekf_config->process_noise_q;      
    ekf_state->Q2 = ekf_config->gyro_bias_noise;      
    ekf_state->R_val = ekf_config->measurement_noise_r; 
    ekf_state->lambda = ekf_config->fading_factor;    

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

    memcpy(ekf_state->P, P_init, sizeof(P_init));

    ekf_state->converge_flag = false;
    ekf_state->stable_flag = false;
    ekf_state->error_count = 0;
    ekf_state->update_count = 0;
    ekf_state->chi_square = 0.0f;
    ekf_state->z_bias_err_lp = 0.0f;
    ekf_state->yaw_round_count = 0;
    ekf_state->yaw_angle_last = 0.0f;
    ekf_state->yaw_total_angle = 0.0f;
    ekf_state->is_initialized = true;

    return EKF_NO_ERROR;
}

Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt) {
    if (!ekf_state->is_initialized) return EKF_INIT_ERROR;

    // =========================================================
    // 👇👇👇 核心优化：将大数组设为 static 防止爆栈 👇👇👇
    // =========================================================
    static float norm_acc[3];
    static float F[36];
    static float q_new[4];
    static float P[36], FP[36], FT[36], P_pred[36];
    static float H[18];
    static float g_pred[3], y[3];
    static float HP[18], HT[18], S[9], S_inv[9];
    static float S_inv_y[3];
    static float PHT[18], K[18];
    static float dx[6];
    static float KH[36], I_KH[36];
    // =========================================================

    float acc_norm_val, gyro_norm_val;
    float half_T = 0.5f * dt;

    // 1. 去除估计零偏后的角速度 (真实信号的无偏估计)
    float gx = gyro[0] - ekf_state->gyro_bias[0];
    float gy = gyro[1] - ekf_state->gyro_bias[1];
    float gz = gyro[2] - ekf_state->gyro_bias[2];

    gyro_norm_val = sqrtf(gx*gx + gy*gy + gz*gz);
    acc_norm_val = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

    // 信号稳态判定：用于确定量测模型的信噪比
    if (gyro_norm_val < EKF_STATIC_GYRO_NORM_THRESH &&
        acc_norm_val > EKF_STATIC_ACC_LOW &&
        acc_norm_val < EKF_STATIC_ACC_HIGH) {
        ekf_state->stable_flag = true;
    } else {
        ekf_state->stable_flag = false;
    }

    // Z轴零偏的低通估计：一种简化的一阶马尔可夫过程估计
    // if (ekf_state->stable_flag) {
    //     float alpha = dt / (dt + EKF_Z_BIAS_TIME_CONSTANT_S);
    //     if (alpha < EKF_Z_BIAS_ALPHA_MIN) alpha = EKF_Z_BIAS_ALPHA_MIN;
    //     if (alpha > EKF_Z_BIAS_ALPHA_MAX) alpha = EKF_Z_BIAS_ALPHA_MAX;

    //     float err_z = gyro[2] - ekf_state->gyro_bias[2];
    //     float err_alpha = dt / (dt + EKF_Z_ERR_LP_TIME_CONSTANT_S);
    //     if (err_alpha < 0.01f) err_alpha = 0.01f;
    //     if (err_alpha > 1.0f) err_alpha = 1.0f;
        
    //     ekf_state->z_bias_err_lp += err_alpha * (err_z - ekf_state->z_bias_err_lp);

    //     if (fabsf(ekf_state->z_bias_err_lp) > EKF_Z_BIAS_DEADBAND_RAD_S) {
    //         ekf_state->gyro_bias[2] += alpha * ekf_state->z_bias_err_lp;
    //         if (ekf_state->gyro_bias[2] > EKF_Z_BIAS_LIMIT_RAD_S) ekf_state->gyro_bias[2] = EKF_Z_BIAS_LIMIT_RAD_S;
    //         if (ekf_state->gyro_bias[2] < -EKF_Z_BIAS_LIMIT_RAD_S) ekf_state->gyro_bias[2] = -EKF_Z_BIAS_LIMIT_RAD_S;
    //     }
    // } 

    gz = gyro[2] - ekf_state->gyro_bias[2];
    
    // 减小死区，仅滤除极小的高斯白噪声，防止慢变信号丢失
    if(fabsf(gz) < 0.005f && ekf_state->stable_flag){
        gz = 0.0f;
    }

    // 归一化加速度 (量测向量 z_k)
    if (acc_norm_val > 1e-4f) {
        float inv_norm = 1.0f / acc_norm_val;
        norm_acc[0] = acc[0] * inv_norm;
        norm_acc[1] = acc[1] * inv_norm;
        norm_acc[2] = acc[2] * inv_norm;
    } else {
        return EKF_UPDATE_ERROR;
    }

    // ================== 先验估计 (Prediction) ==================

    // 雅可比矩阵 F (对非线性系统函数 f(x) 泰勒展开的一阶偏导)
    memcpy(F, F_init, sizeof(F_init)); 

    float hgx = -gx * half_T, hgy = -gy * half_T, hgz = -gz * half_T;
    float q0 = ekf_state->quaternion.q0, q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2, q3 = ekf_state->quaternion.q3;

    F[0*6+1] = hgx; F[0*6+2] = hgy; F[0*6+3] = hgz;
    F[1*6+0] = -hgx; F[1*6+2] = hgz; F[1*6+3] = -hgy;
    F[2*6+0] = -hgy; F[2*6+1] = -hgz; F[2*6+3] = hgx;
    F[3*6+0] = -hgz; F[3*6+1] = hgy; F[3*6+2] = -hgx;

    F[0*6+4] = q1 * half_T; F[0*6+5] = q2 * half_T;
    F[1*6+4] = -q0 * half_T; F[1*6+5] = q3 * half_T;
    F[2*6+4] = -q3 * half_T; F[2*6+5] = -q0 * half_T;
    F[3*6+4] = q2 * half_T; F[3*6+5] = -q1 * half_T;

    // 状态先验估计: \hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_k)
    q_new[0] = q0 + (-q1*gx - q2*gy - q3*gz) * half_T;
    q_new[1] = q1 + ( q0*gx - q3*gy + q2*gz) * half_T;
    q_new[2] = q2 + ( q3*gx + q0*gy - q1*gz) * half_T;
    q_new[3] = q3 + (-q2*gx + q1*gy + q0*gz) * half_T;

    float q_norm = inv_sqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    ekf_state->quaternion.q0 = q_new[0] * q_norm;
    ekf_state->quaternion.q1 = q_new[1] * q_norm;
    ekf_state->quaternion.q2 = q_new[2] * q_norm;
    ekf_state->quaternion.q3 = q_new[3] * q_norm;

    q0 = ekf_state->quaternion.q0; q1 = ekf_state->quaternion.q1;
    q2 = ekf_state->quaternion.q2; q3 = ekf_state->quaternion.q3;

    // 协方差先验估计: P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q_k
    memcpy(P, ekf_state->P, sizeof(P));
    matrix_multiply(F, P, FP, 6, 6, 6);
    matrix_transpose(F, FT, 6, 6);
    matrix_multiply(FP, FT, P_pred, 6, 6, 6);

    float dtQ1 = ekf_state->Q1 * dt;
    float dtQ2 = ekf_state->Q2 * dt;

    P_pred[0*6+0] += dtQ1; P_pred[1*6+1] += dtQ1; P_pred[2*6+2] += dtQ1; P_pred[3*6+3] += dtQ1;
    P_pred[4*6+4] += dtQ2; P_pred[5*6+5] += dtQ2;

    P_pred[4*6+4] /= ekf_state->lambda;
    P_pred[5*6+5] /= ekf_state->lambda;

    if(P_pred[4*6+4] > 10000.0f) P_pred[4*6+4] = 10000.0f;
    if(P_pred[5*6+5] > 10000.0f) P_pred[5*6+5] = 10000.0f;

    // ================== 后验估计 (Update) ==================

    // 观测矩阵雅可比 H (对量测函数 h(x) 泰勒展开)
    memset(H, 0, sizeof(H)); 
    float dx0 = 2*q0, dx1 = 2*q1, dx2 = 2*q2, dx3 = 2*q3;

    H[0*6+0] = -dx2; H[0*6+1] = dx3;  H[0*6+2] = -dx0; H[0*6+3] = dx1;
    H[1*6+0] = dx1;  H[1*6+1] = dx0;  H[1*6+2] = dx3;  H[1*6+3] = dx2;
    H[2*6+0] = dx0;  H[2*6+1] = -dx1; H[2*6+2] = -dx2; H[2*6+3] = dx3;

    // 创新 (新息 / 预测量测残差): y_k = z_k - h(\hat{x}_{k|k-1})
    g_pred[0] = 2*(q1*q3 - q0*q2);
    g_pred[1] = 2*(q0*q1 + q2*q3);
    g_pred[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    y[0] = norm_acc[0] - g_pred[0];
    y[1] = norm_acc[1] - g_pred[1];
    y[2] = norm_acc[2] - g_pred[2];

    // 创新协方差: S_k = H_k * P_{k|k-1} * H_k^T + R_k
    matrix_multiply(H, P_pred, HP, 3, 6, 6);
    matrix_transpose(H, HT, 3, 6);
    matrix_multiply(HP, HT, S, 3, 6, 3);
    
    S[0] += ekf_state->R_val;
    S[4] += ekf_state->R_val;
    S[8] += ekf_state->R_val;

    if (matrix_inverse_3x3(S, S_inv) != 0) return EKF_UPDATE_ERROR;

    // 卡方检验 (剔除置信区间外的异常突变信号)
    matrix_multiply(S_inv, y, S_inv_y, 3, 3, 1); 

    float chi_square = 0.0f;
    for(int i=0; i<3; i++) chi_square += y[i] * S_inv_y[i];
    ekf_state->chi_square = chi_square;

    float gain_scale = 1.0f;
    float threshold = CHI_SQUARE_THRESHOLD_DEFAULT;

    if (chi_square < 0.5f * threshold) ekf_state->converge_flag = true;

    if (chi_square > threshold && ekf_state->converge_flag) {
        if (ekf_state->stable_flag) ekf_state->error_count++;
        else ekf_state->error_count = 0;

        if (ekf_state->error_count > ERROR_COUNT_MAX) {
            ekf_state->converge_flag = false;
        } else {
            memcpy(ekf_state->P, P_pred, sizeof(P));
            return EKF_NO_ERROR;
        }
    } else {
        ekf_state->error_count = 0;
        if (chi_square > 0.1f * threshold && ekf_state->converge_flag) {
            gain_scale = (threshold - chi_square) / (0.9f * threshold);
            if (gain_scale < 0.0f) gain_scale = 0.0f; 
        }
    }

    // 卡尔曼增益: K_k = P_{k|k-1} * H_k^T * S_k^{-1}
    matrix_multiply(P_pred, HT, PHT, 6, 6, 3);
    matrix_multiply(PHT, S_inv, K, 6, 3, 3);

    for(int i=0; i<18; i++) K[i] *= gain_scale;
    for(int i=4; i<6; i++) { 
        for(int j=0; j<3; j++) {
             float cos_val = fabsf(g_pred[i-4]);
             float angle = acosf(cos_val);
             K[i*3+j] *= angle / 1.5707963f;
        }
    }

    // 状态后验更新: \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k * y_k
    matrix_multiply(K, y, dx, 6, 3, 1);

    if (ekf_state->converge_flag) {
        float limit = 1e-2f * dt;
        if (dx[4] > limit) dx[4] = limit;
        if (dx[4] < -limit) dx[4] = -limit;
        if (dx[5] > limit) dx[5] = limit;
        if (dx[5] < -limit) dx[5] = -limit;
    }

    dx[3] = 0.0f; // Z轴协方差更新屏蔽，依赖低通估计

    ekf_state->quaternion.q0 += dx[0];
    ekf_state->quaternion.q1 += dx[1];
    ekf_state->quaternion.q2 += dx[2];
    ekf_state->quaternion.q3 += dx[3];

    ekf_state->gyro_bias[0] += dx[4];
    ekf_state->gyro_bias[1] += dx[5];

    q_norm = inv_sqrt(ekf_state->quaternion.q0*ekf_state->quaternion.q0 +
                      ekf_state->quaternion.q1*ekf_state->quaternion.q1 +
                      ekf_state->quaternion.q2*ekf_state->quaternion.q2 +
                      ekf_state->quaternion.q3*ekf_state->quaternion.q3);
    ekf_state->quaternion.q0 *= q_norm;
    ekf_state->quaternion.q1 *= q_norm;
    ekf_state->quaternion.q2 *= q_norm;
    ekf_state->quaternion.q3 *= q_norm;

    // 协方差后验更新: P_{k|k} = (I - K_k * H_k) * P_{k|k-1}
    matrix_multiply(K, H, KH, 6, 3, 6);

    for(int i=0; i<36; i++) I_KH[i] = -KH[i];
    for(int i=0; i<6; i++) I_KH[i*6+i] += 1.0f;

    matrix_multiply(I_KH, P_pred, ekf_state->P, 6, 6, 6);

    Ekf_quaternion_to_euler(&ekf_state->quaternion, &ekf_state->euler);
    Ekf_update_yaw_continuity(ekf_state);

    ekf_state->update_count++;
    return EKF_NO_ERROR;
}

// ... 辅助矩阵运算和倒数平方根函数保持不变，无需修改 ...
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    (void)acc; (void)gyro;
    return ekf_state->stable_flag;
}

void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    
    // 【修改点】 修正欧拉角解算公式，防止奇点死锁
    euler->yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * EKF_RAD_TO_DEG;
    
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >= 1.0f) euler->pitch = 90.0f;
    else if (sinp <= -1.0f) euler->pitch = -90.0f;
    else euler->pitch = asinf(sinp) * EKF_RAD_TO_DEG;
    
    euler->roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * EKF_RAD_TO_DEG;
}

void Ekf_update_yaw_continuity(Ekf_state_t* ekf_state) {
    float yaw = ekf_state->euler.yaw;
    float last_yaw = ekf_state->yaw_angle_last;

    if (yaw - last_yaw > 180.0f) ekf_state->yaw_round_count--;
    else if (yaw - last_yaw < -180.0f) ekf_state->yaw_round_count++;

    ekf_state->yaw_total_angle = 360.0f * ekf_state->yaw_round_count + yaw;
    ekf_state->yaw_angle_last = yaw;
}

static void matrix_multiply(float* A, float* B, float* C, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) sum += A[i*n + k] * B[k*p + j];
            C[i*p + j] = sum;
        }
    }
}

static void matrix_transpose(float* A, float* B, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) B[j*m + i] = A[i*n + j];
    }
}

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

static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Ekf_set_temperature(Ekf_state_t* ekf_state, float current_temp) { (void)ekf_state; (void)current_temp; }
void Ekf_calculate_temp_compensation(const Ekf_state_t* ekf_state, float temp_compensation[3]) {
    (void)ekf_state;
    temp_compensation[0] = temp_compensation[1] = temp_compensation[2] = 0.0f;
}