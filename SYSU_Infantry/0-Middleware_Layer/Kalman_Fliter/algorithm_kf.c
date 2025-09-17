/**
 * @Author         : GitHub Copilot
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-17
 * @Note           : 标准卡尔曼滤波算法实现
 * @Copyright(c)   : SYSU Copyright
 */
#include "algorithm_kf.h"
#include <string.h>
#include <math.h>

// 标量卡尔曼滤波器实现

/**
 * @brief 初始化标量卡尔曼滤波器
 */
Kf_error_e Kf_scalar_init(Kf_scalar_t* kf, const Kf_config_t* config, float initial_value, float initial_variance) {
    if (kf == NULL || config == NULL) {
        return KF_INIT_ERROR;
    }
    
    // 初始化状态估计值和方差
    kf->x = initial_value;
    kf->P = initial_variance;
    
    // 设置系统模型
    kf->A = 1.0f;     // 默认为恒定状态模型
    kf->B = 0.0f;     // 默认无控制输入
    kf->H = 1.0f;     // 默认直接测量状态
    
    // 设置噪声参数
    kf->Q = config->Q;
    kf->R = config->R;
    
    // 标记为已初始化
    kf->is_initialized = true;
    
    return KF_NO_ERROR;
}

/**
 * @brief 标量卡尔曼滤波预测步骤
 */
void Kf_scalar_predict(Kf_scalar_t* kf, float control) {
    if (!kf->is_initialized) {
        return;
    }
    
    // 状态预测: x = A*x + B*u
    kf->x = kf->A * kf->x + kf->B * control;
    
    // 协方差预测: P = A*P*A' + Q
    kf->P = kf->A * kf->P * kf->A + kf->Q;
}

/**
 * @brief 标量卡尔曼滤波更新步骤
 */
float Kf_scalar_update(Kf_scalar_t* kf, float measurement) {
    if (!kf->is_initialized) {
        return measurement;
    }
    
    // 计算卡尔曼增益: K = P*H' / (H*P*H' + R)
    float K = kf->P * kf->H / (kf->H * kf->P * kf->H + kf->R);
    
    // 更新状态估计: x = x + K*(z - H*x)
    float innovation = measurement - kf->H * kf->x;
    kf->x = kf->x + K * innovation;
    
    // 更新协方差: P = (1 - K*H)*P
    kf->P = (1.0f - K * kf->H) * kf->P;
    
    return kf->x;
}

/**
 * @brief 标量卡尔曼滤波一步预测和更新
 */
float Kf_scalar_step(Kf_scalar_t* kf, float measurement, float control) {
    Kf_scalar_predict(kf, control);
    return Kf_scalar_update(kf, measurement);
}

// 二阶卡尔曼滤波器实现

/**
 * @brief 初始化二阶卡尔曼滤波器
 */
Kf_error_e Kf_2order_init(Kf_2order_t* kf, const Kf_config_t* config, 
                          float initial_pos, float initial_vel,
                          float initial_pos_var, float initial_vel_var) {
    if (kf == NULL || config == NULL) {
        return KF_INIT_ERROR;
    }
    
    // 清空结构体
    memset(kf, 0, sizeof(Kf_2order_t));
    
    // 初始化状态估计值
    kf->x[0] = initial_pos;  // 位置
    kf->x[1] = initial_vel;  // 速度
    
    // 初始化状态协方差矩阵
    kf->P[0][0] = initial_pos_var;
    kf->P[1][1] = initial_vel_var;
    
    // 设置系统模型（匀速运动模型）
    float dt = config->dt;
    kf->A[0][0] = 1.0f;
    kf->A[0][1] = dt;
    kf->A[1][0] = 0.0f;
    kf->A[1][1] = 1.0f;
    
    // 控制输入矩阵 (加速度影响)
    kf->B[0] = 0.5f * dt * dt;
    kf->B[1] = dt;
    
    // 观测矩阵 (只测量位置)
    kf->H[0] = 1.0f;
    kf->H[1] = 0.0f;
    
    // 设置噪声参数
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    // 过程噪声协方差矩阵（考虑位置和速度的相关性）
    kf->Q[0][0] = 0.25f * dt4 * config->Q;  // 位置噪声
    kf->Q[0][1] = 0.5f * dt3 * config->Q;   // 位置-速度相关
    kf->Q[1][0] = 0.5f * dt3 * config->Q;   // 速度-位置相关
    kf->Q[1][1] = dt2 * config->Q;          // 速度噪声
    
    // 测量噪声协方差
    kf->R = config->R;
    
    // 标记为已初始化
    kf->is_initialized = true;
    
    return KF_NO_ERROR;
}

/**
 * @brief 二阶卡尔曼滤波预测步骤
 */
void Kf_2order_predict(Kf_2order_t* kf, float control) {
    if (!kf->is_initialized) {
        return;
    }
    
    // 临时变量
    float x_pred[2];
    float P_pred[2][2];
    
    // 状态预测: x = A*x + B*u
    x_pred[0] = kf->A[0][0] * kf->x[0] + kf->A[0][1] * kf->x[1] + kf->B[0] * control;
    x_pred[1] = kf->A[1][0] * kf->x[0] + kf->A[1][1] * kf->x[1] + kf->B[1] * control;
    
    // 协方差预测: P = A*P*A' + Q
    // P_pred = A*P
    P_pred[0][0] = kf->A[0][0] * kf->P[0][0] + kf->A[0][1] * kf->P[1][0];
    P_pred[0][1] = kf->A[0][0] * kf->P[0][1] + kf->A[0][1] * kf->P[1][1];
    P_pred[1][0] = kf->A[1][0] * kf->P[0][0] + kf->A[1][1] * kf->P[1][0];
    P_pred[1][1] = kf->A[1][0] * kf->P[0][1] + kf->A[1][1] * kf->P[1][1];
    
    // P_pred = P_pred*A' + Q
    kf->P[0][0] = P_pred[0][0] * kf->A[0][0] + P_pred[0][1] * kf->A[0][1] + kf->Q[0][0];
    kf->P[0][1] = P_pred[0][0] * kf->A[1][0] + P_pred[0][1] * kf->A[1][1] + kf->Q[0][1];
    kf->P[1][0] = P_pred[1][0] * kf->A[0][0] + P_pred[1][1] * kf->A[0][1] + kf->Q[1][0];
    kf->P[1][1] = P_pred[1][0] * kf->A[1][0] + P_pred[1][1] * kf->A[1][1] + kf->Q[1][1];
    
    // 更新状态
    kf->x[0] = x_pred[0];
    kf->x[1] = x_pred[1];
}

/**
 * @brief 二阶卡尔曼滤波更新步骤
 */
float Kf_2order_update(Kf_2order_t* kf, float measurement) {
    if (!kf->is_initialized) {
        return measurement;
    }
    
    // 计算创新协方差: S = H*P*H' + R
    float S = kf->H[0] * kf->P[0][0] * kf->H[0] + 
              kf->H[0] * kf->P[0][1] * kf->H[1] + 
              kf->H[1] * kf->P[1][0] * kf->H[0] + 
              kf->H[1] * kf->P[1][1] * kf->H[1] + 
              kf->R;
    
    // 计算卡尔曼增益: K = P*H' / S
    float K[2];
    K[0] = (kf->P[0][0] * kf->H[0] + kf->P[0][1] * kf->H[1]) / S;
    K[1] = (kf->P[1][0] * kf->H[0] + kf->P[1][1] * kf->H[1]) / S;
    
    // 计算创新: innovation = z - H*x
    float innovation = measurement - (kf->H[0] * kf->x[0] + kf->H[1] * kf->x[1]);
    
    // 更新状态估计: x = x + K*innovation
    kf->x[0] = kf->x[0] + K[0] * innovation;
    kf->x[1] = kf->x[1] + K[1] * innovation;
    
    // 更新协方差: P = (I - K*H)*P
    float P_new[2][2];
    P_new[0][0] = kf->P[0][0] - K[0] * (kf->H[0] * kf->P[0][0] + kf->H[1] * kf->P[1][0]);
    P_new[0][1] = kf->P[0][1] - K[0] * (kf->H[0] * kf->P[0][1] + kf->H[1] * kf->P[1][1]);
    P_new[1][0] = kf->P[1][0] - K[1] * (kf->H[0] * kf->P[0][0] + kf->H[1] * kf->P[1][0]);
    P_new[1][1] = kf->P[1][1] - K[1] * (kf->H[0] * kf->P[0][1] + kf->H[1] * kf->P[1][1]);
    
    // 更新协方差矩阵
    kf->P[0][0] = P_new[0][0];
    kf->P[0][1] = P_new[0][1];
    kf->P[1][0] = P_new[1][0];
    kf->P[1][1] = P_new[1][1];
    
    return kf->x[0];
}

/**
 * @brief 二阶卡尔曼滤波一步预测和更新
 */
float Kf_2order_step(Kf_2order_t* kf, float measurement, float control) {
    Kf_2order_predict(kf, control);
    return Kf_2order_update(kf, measurement);
}

/**
 * @brief 获取二阶卡尔曼滤波器的速度估计
 */
float Kf_2order_get_velocity(const Kf_2order_t* kf) {
    if (!kf->is_initialized) {
        return 0.0f;
    }
    return kf->x[1];
}
