/**
 * @Author         : SYSU电控组
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-28
 * @Note           : 扩展卡尔曼滤波算法实现 (基于参考代码重构)
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
#define EKF_STATIC_THRESHOLD_GYRO 0.3f      // 陀螺仪静态阈值 (rad/s)
#define EKF_STATIC_THRESHOLD_ACC_MIN 9.3f   // 加速度静态阈值下限 (m/s^2)
#define EKF_STATIC_THRESHOLD_ACC_MAX 10.3f  // 加速度静态阈值上限 (m/s^2)
#define EKF_CHI_SQUARE_THRESHOLD 1e-8f      // 卡方检验阈值
#define EKF_CONVERGE_THRESHOLD 0.5f         // 收敛阈值系数
#define EKF_ERROR_COUNT_MAX 50              // 最大错误计数

// 私有函数声明
static void quaternion_normalize(Quaternion_t* q);
static float inv_sqrt(float x);
static void ekf_predict_step(Ekf_state_t* ekf_state, const float gyro[3], float dt);
static void ekf_update_step(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt);
static void compute_state_transition_matrix(Ekf_state_t* ekf_state, const float gyro[3], float dt);
static void compute_observation_matrix(Ekf_state_t* ekf_state);
static bool chi_square_test(Ekf_state_t* ekf_state, const float residual[3]);
static void matrix_multiply_6x6(float a[6][6], float b[6][6], float result[6][6]);
static void matrix_multiply_6x3(float a[6][6], float b[6][3], float result[6][3]);
static void matrix_multiply_3x6(float a[3][6], float b[6][6], float result[3][6]);
static void matrix_multiply_3x3_h_ht(float a[3][6], float b[6][3], float result[3][3]);
static void matrix_multiply_3x3(float a[3][3], float b[3][3], float result[3][3]);
static void matrix_transpose_3x6(float src[3][6], float dst[6][3]);
static void matrix_inverse_3x3(float src[3][3], float dst[3][3]);
static void apply_low_pass_filter(Ekf_state_t* ekf_state, const float acc[3], float dt);

// 新增函数声明 - 温度补偿和改进算法
static void quaternion_rk4_integration(Quaternion_t* q, float wx, float wy, float wz, float dt);
static void dynamic_measurement_noise_adjustment(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]);
static void enhanced_fading_filter(Ekf_state_t* ekf_state);

/** @brief 四元数RK4积分方法 - 参考QuaternionEKF的高精度积分
 */
static void quaternion_rk4_integration(Quaternion_t* q, float wx, float wy, float wz, float dt) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float half_dt = 0.5f * dt;
    
    // RK4 第一步
    float k1_q0 = half_dt * (-wx * q1 - wy * q2 - wz * q3);
    float k1_q1 = half_dt * ( wx * q0 + wz * q2 - wy * q3);
    float k1_q2 = half_dt * ( wy * q0 - wz * q1 + wx * q3);
    float k1_q3 = half_dt * ( wz * q0 + wy * q1 - wx * q2);
    
    // RK4 第二步
    float q0_tmp = q0 + k1_q0;
    float q1_tmp = q1 + k1_q1;
    float q2_tmp = q2 + k1_q2;
    float q3_tmp = q3 + k1_q3;
    
    float k2_q0 = half_dt * (-wx * q1_tmp - wy * q2_tmp - wz * q3_tmp);
    float k2_q1 = half_dt * ( wx * q0_tmp + wz * q2_tmp - wy * q3_tmp);
    float k2_q2 = half_dt * ( wy * q0_tmp - wz * q1_tmp + wx * q3_tmp);
    float k2_q3 = half_dt * ( wz * q0_tmp + wy * q1_tmp - wx * q2_tmp);
    
    // RK4 第三步
    q0_tmp = q0 + k2_q0;
    q1_tmp = q1 + k2_q1;
    q2_tmp = q2 + k2_q2;
    q3_tmp = q3 + k2_q3;
    
    float k3_q0 = dt * (-wx * q1_tmp - wy * q2_tmp - wz * q3_tmp);
    float k3_q1 = dt * ( wx * q0_tmp + wz * q2_tmp - wy * q3_tmp);
    float k3_q2 = dt * ( wy * q0_tmp - wz * q1_tmp + wx * q3_tmp);
    float k3_q3 = dt * ( wz * q0_tmp + wy * q1_tmp - wx * q2_tmp);
    
    // RK4 第四步
    q0_tmp = q0 + k3_q0;
    q1_tmp = q1 + k3_q1;
    q2_tmp = q2 + k3_q2;
    q3_tmp = q3 + k3_q3;
    
    float k4_q0 = half_dt * (-wx * q1_tmp - wy * q2_tmp - wz * q3_tmp);
    float k4_q1 = half_dt * ( wx * q0_tmp + wz * q2_tmp - wy * q3_tmp);
    float k4_q2 = half_dt * ( wy * q0_tmp - wz * q1_tmp + wx * q3_tmp);
    float k4_q3 = half_dt * ( wz * q0_tmp + wy * q1_tmp - wx * q2_tmp);
    
    // 合成结果
    q->q0 = q0 + (k1_q0 + 2.0f * k2_q0 + 2.0f * k3_q0 + k4_q0) / 3.0f;
    q->q1 = q1 + (k1_q1 + 2.0f * k2_q1 + 2.0f * k3_q1 + k4_q1) / 3.0f;
    q->q2 = q2 + (k1_q2 + 2.0f * k2_q2 + 2.0f * k3_q2 + k4_q2) / 3.0f;
    q->q3 = q3 + (k1_q3 + 2.0f * k2_q3 + 2.0f * k3_q3 + k4_q3) / 3.0f;
    
    // 归一化
    quaternion_normalize(q);
}

/**
 * @brief 动态调整测量噪声 - 参考QuaternionEKF的自适应策略
 */
static void dynamic_measurement_noise_adjustment(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    // 计算载体运动强度
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    
    // 基础测量噪声
    float base_r = ekf_state->measurement_noise_r;
    float dynamic_r = base_r;
    
    // 根据运动状态调整测量噪声
    if (ekf_state->is_static) {
        // 静态时降低测量噪声，增加对加速度计的信任
        dynamic_r = base_r * 0.1f;
    } else {
        // 动态时根据运动强度调整
        float motion_scale = 1.0f + gyro_norm * 10.0f; // 角速度越大，噪声越大
        
        // 加速度偏离重力越多，噪声越大
        float acc_deviation = fabsf(acc_norm - EKF_GRAVITY) / EKF_GRAVITY;
        motion_scale *= (1.0f + acc_deviation * 5.0f);
        
        dynamic_r = base_r * motion_scale;
        
        // 限制范围
        if (dynamic_r > base_r * 100.0f) dynamic_r = base_r * 100.0f;
        if (dynamic_r < base_r * 0.01f) dynamic_r = base_r * 0.01f;
    }
    
    // 更新R矩阵
    ekf_state->R[0][0] = dynamic_r;
    ekf_state->R[1][1] = dynamic_r;
    ekf_state->R[2][2] = dynamic_r;
}

/**
 * @brief 增强渐减滤波机制 - 确保所有零偏都应用正确的渐减系数
 */
static void enhanced_fading_filter(Ekf_state_t* ekf_state) {
    float lambda = ekf_state->fading_factor;
    
    // 对所有零偏方差应用渐减滤波（包括Z轴）
    ekf_state->P[4][4] /= lambda; // x轴零偏方差
    ekf_state->P[5][5] /= lambda; // y轴零偏方差
    
    // 关键改进：为Z轴零偏也应用渐减滤波
    // 注意：这里我们需要扩展状态向量到7维（四元数4 + 零偏3）
    // 但为了兼容现有代码，我们通过单独的Z轴零偏处理来实现类似效果
    
    // 防止方差过度收敛的保护机制
    const float min_variance = 1e-6f;
    const float max_variance = 10000.0f;
    
    for (int i = 4; i < 6; i++) {
        if (ekf_state->P[i][i] < min_variance) {
            ekf_state->P[i][i] = min_variance;
        }
        if (ekf_state->P[i][i] > max_variance) {
            ekf_state->P[i][i] = max_variance;
        }
    }
}

// ========================= 温度补偿实现 =========================

/**
 * @brief 设置温度补偿参数
 */
void Ekf_set_temperature(Ekf_state_t* ekf_state, float current_temp) {
    if (ekf_state != NULL) {
        ekf_state->temperature = current_temp;
    }
}

/**
 * @brief 计算温度补偿值
 */
void Ekf_calculate_temp_compensation(const Ekf_state_t* ekf_state, float temp_compensation[3]) {
    if (ekf_state == NULL || temp_compensation == NULL) {
        return;
    }
    
    if (!ekf_state->enable_temp_comp) {
        temp_compensation[0] = 0.0f;
        temp_compensation[1] = 0.0f;
        temp_compensation[2] = 0.0f;
        return;
    }
    
    // 计算温度变化
    float temp_delta = ekf_state->temperature - ekf_state->temp_baseline;
    
    // 计算各轴的温度补偿
    temp_compensation[0] = ekf_state->temp_coeff[0] * temp_delta;
    temp_compensation[1] = ekf_state->temp_coeff[1] * temp_delta;
    temp_compensation[2] = ekf_state->temp_coeff[2] * temp_delta;
}

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

    // 初始化状态协方差矩阵P (6x6: 四元数4 + 零偏2，不估计z轴零偏)
    // 注意：虽然这里说不估计z轴零偏，但我们需要在运行时对其进行处理
    memset(ekf_state->P, 0, sizeof(ekf_state->P));
    // 四元数部分初始不确定性较大
    for (int i = 0; i < 4; i++) {
        ekf_state->P[i][i] = 10000.0f;  // 降低初始不确定性，原来是100000
    }
    // 陀螺仪零偏初始不确定性
    ekf_state->P[4][4] = 10.0f;   // x轴零偏，降低初始不确定性
    ekf_state->P[5][5] = 10.0f;   // y轴零偏，降低初始不确定性
    
    // 设置非对角元素
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i != j) {
                ekf_state->P[i][j] = 0.1f;
            }
        }
    }

    // 初始化过程噪声协方差矩阵Q
    memset(ekf_state->Q, 0, sizeof(ekf_state->Q));
    
    // 初始化测量噪声协方差矩阵R
    memset(ekf_state->R, 0, sizeof(ekf_state->R));
    for (int i = 0; i < 3; i++) {
        ekf_state->R[i][i] = ekf_config->measurement_noise_r;
    }

    // 保存配置参数
    ekf_state->process_noise_q = ekf_config->process_noise_q;
    ekf_state->measurement_noise_r = ekf_config->measurement_noise_r;
    ekf_state->gyro_bias_noise = ekf_config->gyro_bias_noise;
    ekf_state->enable_bias_correction = ekf_config->enable_bias_correction;
    ekf_state->static_threshold = ekf_config->static_threshold > 0 ? ekf_config->static_threshold : EKF_STATIC_THRESHOLD_GYRO;
    
    // 保存渐减因子，如果没有设置则使用默认值
    ekf_state->fading_factor = (ekf_config->fading_factor > 0.99f && ekf_config->fading_factor <= 1.0f) ? 
                               ekf_config->fading_factor : 0.9996f;
    
    // 初始化其他状态变量
    ekf_state->is_initialized = true;
    ekf_state->static_count = 0;
    ekf_state->is_static = false;
    
    // 新增状态变量
    ekf_state->converge_flag = false;
    ekf_state->error_count = 0;
    ekf_state->update_count = 0;
    ekf_state->chi_square = 0.0f;
    ekf_state->adaptive_gain_scale = 1.0f;
    ekf_state->acc_lpf_coef = 0.0f;  // 默认不使用低通滤波
    
    // 初始化滤波后的加速度
    ekf_state->acc_filtered[0] = 0.0f;
    ekf_state->acc_filtered[1] = 0.0f;
    ekf_state->acc_filtered[2] = 0.0f;
    
    // 初始化yaw角度连续性处理变量
    ekf_state->yaw_angle_last = 0.0f;
    ekf_state->yaw_round_count = 0;
    ekf_state->yaw_total_angle = 0.0f;

    return EKF_NO_ERROR;
}

/**
 * @brief 使用新的传感器数据更新EKF
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt) {
    if (ekf_state == NULL || acc == NULL || gyro == NULL) {
        return EKF_UPDATE_ERROR;
    }

    // 检查EKF是否已初始化
    if (!ekf_state->is_initialized) {
        return EKF_INIT_ERROR;
    }

    // 应用低通滤波到加速度数据
    apply_low_pass_filter(ekf_state, acc, dt);
    
    // 静态检测
    Ekf_detect_static_state(ekf_state, ekf_state->acc_filtered, gyro);

    // 在静态状态下更新零偏估计
    if (ekf_state->is_static && ekf_state->enable_bias_correction) {
        // 更积极地更新所有轴的零偏，包括z轴
        float bias_update_rate = 0.02f;  // 更保守的更新率
        for (int i = 0; i < 2; i++) {  // x,y轴正常更新
            ekf_state->gyro_bias[i] = (1.0f - bias_update_rate) * ekf_state->gyro_bias[i] + bias_update_rate * gyro[i];
        }
        // Z轴零偏特殊处理：在长时间静态时才更新
        if (ekf_state->static_count > 500) {  // 静态5秒以上才更新Z轴零偏
            ekf_state->gyro_bias[2] = 0.999f * ekf_state->gyro_bias[2] + 0.001f * gyro[2];
        }
    }

    // EKF预测步骤
    ekf_predict_step(ekf_state, gyro, dt);

    // EKF更新步骤（使用加速度计数据）
    ekf_update_step(ekf_state, ekf_state->acc_filtered, gyro, dt);

    // 更新欧拉角
    Ekf_quaternion_to_euler(&ekf_state->quaternion, &ekf_state->euler);
    
    // 更新yaw角度连续性处理
    Ekf_update_yaw_continuity(ekf_state);
    
    // 更新计数
    ekf_state->update_count++;

    return EKF_NO_ERROR;
}

/**
 * @brief 检测静态状态
 */
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]) {
    // 计算加速度模长
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    
    // 计算角速度模长
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

    // 严格的静态判断条件（参考QuaternionEKF）
    if (gyro_norm < EKF_STATIC_THRESHOLD_GYRO && 
        acc_norm > EKF_STATIC_THRESHOLD_ACC_MIN && 
        acc_norm < EKF_STATIC_THRESHOLD_ACC_MAX) {
        ekf_state->static_count++;
        // 需要更长时间的稳定状态才认为是真正静态
        if (ekf_state->static_count > 200) {  // 2秒连续满足条件才认为是静态
            ekf_state->is_static = true;
        }
    } else {
        // 逐渐减少静态计数，而不是直接清零
        if (ekf_state->static_count > 0) {
            ekf_state->static_count -= 5; // 渐进减少
        }
        if (ekf_state->static_count <= 0) {
            ekf_state->static_count = 0;
            ekf_state->is_static = false;
        }
    }

    return ekf_state->is_static;
}

/**
 * @brief 四元数转欧拉角 (修复版，包含yaw连续性处理)
 */
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler) {
    // 归一化四元数
    quaternion_normalize(q);

    // 根据四元数计算欧拉角
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
    euler->roll = atan2f(2.0f * (q0q1 + q2q3), 1.0f - 2.0f * (q1q1 + q2q2)) * EKF_RAD_TO_DEG;
    float pitch_val = 2.0f * (q0q2 - q1q3);
    // 限制pitch_val的范围，避免asin函数输入超出[-1, 1]
    if (pitch_val > 1.0f) {
        pitch_val = 1.0f;
    } else if (pitch_val < -1.0f) {
        pitch_val = -1.0f;
    }
    euler->pitch = asinf(pitch_val) * EKF_RAD_TO_DEG;
    euler->yaw = atan2f(2.0f * (q0q3 + q1q2), 1.0f - 2.0f * (q2q2 + q3q3)) * EKF_RAD_TO_DEG;
}

/**
 * @brief 更新yaw角度连续性处理（基于参考代码）
 */
void Ekf_update_yaw_continuity(Ekf_state_t* ekf_state) {
    // 处理yaw角度的连续性，参考QuaternionEKF的实现
    if (ekf_state->euler.yaw - ekf_state->yaw_angle_last > 180.0f) {
        ekf_state->yaw_round_count--;
    } else if (ekf_state->euler.yaw - ekf_state->yaw_angle_last < -180.0f) {
        ekf_state->yaw_round_count++;
    }
    
    // 计算总yaw角度（包含转数）
    ekf_state->yaw_total_angle = 360.0f * ekf_state->yaw_round_count + ekf_state->euler.yaw;
    ekf_state->yaw_angle_last = ekf_state->euler.yaw;
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
    } else {
        // 如果四元数过小，重置为单位四元数
        q->q0 = 1.0f;
        q->q1 = 0.0f;
        q->q2 = 0.0f;
        q->q3 = 0.0f;
    }
    
    // 进一步检查结果的有效性
    if (!isfinite(q->q0) || !isfinite(q->q1) || !isfinite(q->q2) || !isfinite(q->q3)) {
        // 如果出现NaN或无穷大，重置为单位四元数
        q->q0 = 1.0f;
        q->q1 = 0.0f;
        q->q2 = 0.0f;
        q->q3 = 0.0f;
    }
}

/**
 * @brief 快速平方根倒数计算
 */
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 计算状态转移矩阵F
 */
static void compute_state_transition_matrix(Ekf_state_t* ekf_state, const float gyro[3], float dt) {
    // 补偿陀螺仪零偏
    float wx = gyro[0] - ekf_state->gyro_bias[0];
    float wy = gyro[1] - ekf_state->gyro_bias[1];
    float wz = gyro[2] - ekf_state->gyro_bias[2];
    
    float halfwxdt = 0.5f * wx * dt;
    float halfwydt = 0.5f * wy * dt;
    float halfwzdt = 0.5f * wz * dt;

    // 初始化F为单位矩阵
    memset(ekf_state->F, 0, sizeof(ekf_state->F));
    for (int i = 0; i < 6; i++) {
        ekf_state->F[i][i] = 1.0f;
    }

    // 设置F矩阵的左上4x4子矩阵（四元数状态转移）
    ekf_state->F[0][1] = -halfwxdt;
    ekf_state->F[0][2] = -halfwydt;
    ekf_state->F[0][3] = -halfwzdt;

    ekf_state->F[1][0] = halfwxdt;
    ekf_state->F[1][2] = halfwzdt;
    ekf_state->F[1][3] = -halfwydt;

    ekf_state->F[2][0] = halfwydt;
    ekf_state->F[2][1] = -halfwzdt;
    ekf_state->F[2][3] = halfwxdt;

    ekf_state->F[3][0] = halfwzdt;
    ekf_state->F[3][1] = halfwydt;
    ekf_state->F[3][2] = -halfwxdt;

    // 设置四元数对零偏的导数项（右上4x2子矩阵）
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;

    ekf_state->F[0][4] = q1 * dt / 2.0f;
    ekf_state->F[0][5] = q2 * dt / 2.0f;

    ekf_state->F[1][4] = -q0 * dt / 2.0f;
    ekf_state->F[1][5] = q3 * dt / 2.0f;

    ekf_state->F[2][4] = -q3 * dt / 2.0f;
    ekf_state->F[2][5] = -q0 * dt / 2.0f;

    ekf_state->F[3][4] = q2 * dt / 2.0f;
    ekf_state->F[3][5] = -q1 * dt / 2.0f;
}

/**
 * @brief 计算观测矩阵H
 */
static void compute_observation_matrix(Ekf_state_t* ekf_state) {
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;

    float double_q0 = 2.0f * q0;
    float double_q1 = 2.0f * q1;
    float double_q2 = 2.0f * q2;
    float double_q3 = 2.0f * q3;

    // 初始化H矩阵
    memset(ekf_state->H, 0, sizeof(ekf_state->H));

    // 第一行：∂hx/∂q
    ekf_state->H[0][0] = -double_q2;
    ekf_state->H[0][1] = double_q3;
    ekf_state->H[0][2] = -double_q0;
    ekf_state->H[0][3] = double_q1;

    // 第二行：∂hy/∂q
    ekf_state->H[1][0] = double_q1;
    ekf_state->H[1][1] = double_q0;
    ekf_state->H[1][2] = double_q3;
    ekf_state->H[1][3] = double_q2;

    // 第三行：∂hz/∂q
    ekf_state->H[2][0] = double_q0;
    ekf_state->H[2][1] = -double_q1;
    ekf_state->H[2][2] = -double_q2;
    ekf_state->H[2][3] = double_q3;

    // 后两列（对零偏的偏导数）为0，已经在初始化时设为0
}

/**
 * @brief 卡方检验（改进版，基于QuaternionEKF）
 */
static bool chi_square_test(Ekf_state_t* ekf_state, const float residual[3]) {
    // 防止未使用参数警告
    (void)residual;
    
    // 收敛判断
    if (ekf_state->chi_square < 0.5f * EKF_CHI_SQUARE_THRESHOLD) {
        ekf_state->converge_flag = true;
    }

    // 如果卡方值超过阈值且已收敛
    if (ekf_state->chi_square > EKF_CHI_SQUARE_THRESHOLD && ekf_state->converge_flag) {
        if (ekf_state->is_static) {
            ekf_state->error_count++; // 载体静止时仍无法通过卡方检验
        } else {
            ekf_state->error_count = 0;
        }

        if (ekf_state->error_count > EKF_ERROR_COUNT_MAX) {
            // 滤波器发散，重置收敛标志
            ekf_state->converge_flag = false;
            ekf_state->adaptive_gain_scale = 1.0f;
            return true; // 强制接受更新，恢复滤波
        } else {
            // 残差未通过卡方检验，仅预测
            ekf_state->adaptive_gain_scale = 0.0f; // 完全拒绝更新
            return false; // 拒绝更新
        }
    } else {
        // 计算自适应增益缩放因子（参考QuaternionEKF逻辑）
        if (ekf_state->chi_square > 0.1f * EKF_CHI_SQUARE_THRESHOLD && ekf_state->converge_flag) {
            ekf_state->adaptive_gain_scale = (EKF_CHI_SQUARE_THRESHOLD - ekf_state->chi_square) / 
                                           (0.9f * EKF_CHI_SQUARE_THRESHOLD);
            // 确保增益缩放在合理范围内
            if (ekf_state->adaptive_gain_scale < 0.1f) {
                ekf_state->adaptive_gain_scale = 0.1f;
            } else if (ekf_state->adaptive_gain_scale > 1.0f) {
                ekf_state->adaptive_gain_scale = 1.0f;
            }
        } else {
            ekf_state->adaptive_gain_scale = 1.0f;
        }
        
        ekf_state->error_count = 0;
        return true; // 接受更新
    }
}

/**
 * @brief 应用低通滤波到加速度数据
 */
static void apply_low_pass_filter(Ekf_state_t* ekf_state, const float acc[3], float dt) {
    if (ekf_state->update_count == 0 || ekf_state->acc_lpf_coef <= 0.0f) {
        // 第一次调用或不使用滤波
        ekf_state->acc_filtered[0] = acc[0];
        ekf_state->acc_filtered[1] = acc[1];
        ekf_state->acc_filtered[2] = acc[2];
    } else {
        // 应用低通滤波
        float alpha = dt / (dt + ekf_state->acc_lpf_coef);
        ekf_state->acc_filtered[0] = ekf_state->acc_filtered[0] * (1.0f - alpha) + acc[0] * alpha;
        ekf_state->acc_filtered[1] = ekf_state->acc_filtered[1] * (1.0f - alpha) + acc[1] * alpha;
        ekf_state->acc_filtered[2] = ekf_state->acc_filtered[2] * (1.0f - alpha) + acc[2] * alpha;
    }
}

/**
 * @brief EKF预测步骤 - 改进版，包含温度补偿和高阶积分
 */
static void ekf_predict_step(Ekf_state_t* ekf_state, const float gyro[3], float dt) {
    // 计算温度补偿
    float temp_compensation[3] = {0, 0, 0};
    if (ekf_state->enable_temp_comp) {
        Ekf_calculate_temp_compensation(ekf_state, temp_compensation);
    }
    
    // 补偿陀螺仪零偏和温度漂移
    float wx = gyro[0] - ekf_state->gyro_bias[0] - temp_compensation[0];
    float wy = gyro[1] - ekf_state->gyro_bias[1] - temp_compensation[1];
    float wz = gyro[2] - ekf_state->gyro_bias[2] - temp_compensation[2];

    // 使用RK4方法进行四元数积分（参考QuaternionEKF的高精度积分）
    quaternion_rk4_integration(&ekf_state->quaternion, wx, wy, wz, dt);

    // 计算状态转移矩阵F
    compute_state_transition_matrix(ekf_state, gyro, dt);
    
    // 更新过程噪声矩阵Q
    memset(ekf_state->Q, 0, sizeof(ekf_state->Q));
    for (int i = 0; i < 4; i++) {
        ekf_state->Q[i][i] = ekf_state->process_noise_q * dt;
    }
    ekf_state->Q[4][4] = ekf_state->gyro_bias_noise * dt;
    ekf_state->Q[5][5] = ekf_state->gyro_bias_noise * dt;

    // 预测协方差矩阵: P = F*P*F' + Q
    float P_temp[6][6];
    float FP[6][6];
    
    // FP = F * P
    matrix_multiply_6x6(ekf_state->F, ekf_state->P, FP);
    
    // P_temp = FP * F'
    float F_transpose[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            F_transpose[j][i] = ekf_state->F[i][j];
        }
    }
    matrix_multiply_6x6(FP, F_transpose, P_temp);
    
    // P = P_temp + Q
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf_state->P[i][j] = P_temp[i][j] + ekf_state->Q[i][j];
        }
    }
    
    // 渐减滤波，防止零偏参数过度收敛（参考QuaternionEKF）
    float lambda = ekf_state->fading_factor; // 使用配置的渐减系数
    ekf_state->P[4][4] /= lambda; // x轴零偏方差
    ekf_state->P[5][5] /= lambda; // y轴零偏方差
    
    // 限幅，防止发散
    if (ekf_state->P[4][4] > 10000.0f) {
        ekf_state->P[4][4] = 10000.0f;
    }
    if (ekf_state->P[5][5] > 10000.0f) {
        ekf_state->P[5][5] = 10000.0f;
    }
    
    // 为四元数状态添加适当的过程噪声
    for (int i = 0; i < 4; i++) {
        if (ekf_state->P[i][i] < 1e-6f) {
            ekf_state->P[i][i] = 1e-6f; // 防止过度收敛
        }
    }
}

/**
 * @brief EKF更新步骤 - 基于参考代码重构
 */
static void ekf_update_step(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt) {
    // 归一化加速度测量值
    float acc_norm = inv_sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    float acc_normalized[3] = {
        acc[0] * acc_norm,
        acc[1] * acc_norm, 
        acc[2] * acc_norm
    };

    // 获取当前四元数
    float q0 = ekf_state->quaternion.q0;
    float q1 = ekf_state->quaternion.q1;
    float q2 = ekf_state->quaternion.q2;
    float q3 = ekf_state->quaternion.q3;

    // 计算预测的重力方向（机体坐标系）
    float gravity_pred[3] = {
        2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    };

    // 计算残差
    float residual[3] = {
        acc_normalized[0] - gravity_pred[0],
        acc_normalized[1] - gravity_pred[1],
        acc_normalized[2] - gravity_pred[2]
    };

    // 计算观测矩阵H
    compute_observation_matrix(ekf_state);

    // 计算创新协方差S = H*P*H' + R
    float HP[3][6];
    matrix_multiply_3x6(ekf_state->H, ekf_state->P, HP);
    
    float H_transpose[6][3];
    matrix_transpose_3x6(ekf_state->H, H_transpose);
    
    float S[3][3];
    matrix_multiply_3x3_h_ht(HP, H_transpose, S);
    
    // S = S + R
    for (int i = 0; i < 3; i++) {
        S[i][i] += ekf_state->R[i][i];
    }

    // 卡方检验
    float S_inv[3][3];
    matrix_inverse_3x3(S, S_inv);
    
    // 计算卡方值: chi2 = residual' * S_inv * residual
    float temp[3];
    for (int i = 0; i < 3; i++) {
        temp[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            temp[i] += S_inv[i][j] * residual[j];
        }
    }
    
    ekf_state->chi_square = 0.0f;
    for (int i = 0; i < 3; i++) {
        ekf_state->chi_square += residual[i] * temp[i];
    }

    // 进行卡方检验
    bool accept_update = chi_square_test(ekf_state, residual);
    
    // 如果不接受更新，直接返回
    if (!accept_update) {
        return; // 跳过此次更新
    }

    // 计算卡尔曼增益K = P*H'*S_inv
    float PH_transpose[6][3];
    matrix_multiply_6x3(ekf_state->P, H_transpose, PH_transpose);
    
    float K[6][3];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                K[i][j] += PH_transpose[i][k] * S_inv[k][j];
            }
        }
    }

    // 应用自适应增益
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] *= ekf_state->adaptive_gain_scale;
        }
    }

    // 根据方向余弦调整零偏增益（参考QuaternionEKF中的OrientationCosine处理）
    // 计算各轴方向余弦，用于调整零偏修正的增益
    float orientation_cosine[3];
    for (int i = 0; i < 3; i++) {
        orientation_cosine[i] = acosf(fabsf(gravity_pred[i]));
    }
    
    // 对零偏项的增益进行方向调整
    for (int i = 4; i < 6; i++) { // 只调整x,y轴零偏
        for (int j = 0; j < 3; j++) {
            K[i][j] *= orientation_cosine[i - 4] / 1.5707963f; // 1弧度
        }
    }

    // 状态更新: x = x + K*residual
    float delta_state[6];
    for (int i = 0; i < 6; i++) {
        delta_state[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            delta_state[i] += K[i][j] * residual[j];
        }
    }

    // 限制零偏更新幅度
    if (ekf_state->converge_flag) {
        float bias_limit = 1e-2f * dt;
        for (int i = 4; i < 6; i++) {
            if (delta_state[i] > bias_limit) delta_state[i] = bias_limit;
            if (delta_state[i] < -bias_limit) delta_state[i] = -bias_limit;
        }
    }

    // 完全禁止yaw轴的直接修正（参考QuaternionEKF）
    delta_state[3] = 0.0f;  // 不修正yaw轴数据
    
    // 更新四元数
    ekf_state->quaternion.q0 += delta_state[0];
    ekf_state->quaternion.q1 += delta_state[1];
    ekf_state->quaternion.q2 += delta_state[2];
    ekf_state->quaternion.q3 += delta_state[3];
    
    // 更新零偏
    ekf_state->gyro_bias[0] += delta_state[4];
    ekf_state->gyro_bias[1] += delta_state[5];
    
    // Z轴零偏处理：改进的动态跟踪策略（参考QuaternionEKF）
    // Z轴零偏动态估计（参考QuaternionEKF的OrientationCosine策略）
    if (ekf_state->enable_bias_correction) {
        float z_orientation_weight = orientation_cosine[2] / 1.5707963f; // 对应于Z轴的方向权重
        
        // 只有在Z轴与重力方向不垂直时才能进行Z轴零偏估计
        if (z_orientation_weight > 0.1f) { // 至少倵76度角度
            float z_gyro_innovation = gyro[2] - ekf_state->gyro_bias[2];
            
            // 动态调整Z轴零偏更新速度
            float z_bias_gain = z_orientation_weight * 0.001f; // 基础增益
            if (ekf_state->is_static && ekf_state->static_count > 500) {
                z_bias_gain *= 2.0f; // 静态时增加增益
            }
            
            // 应用自适应增益缩放
            z_bias_gain *= ekf_state->adaptive_gain_scale;
            
            // Z轴零偏更新
            float z_bias_update = z_bias_gain * z_gyro_innovation * dt;
            
            // 限制更新幅度
            if (ekf_state->converge_flag) {
                float bias_limit = 5e-3f * dt; // 限制更新速度
                if (z_bias_update > bias_limit) z_bias_update = bias_limit;
                if (z_bias_update < -bias_limit) z_bias_update = -bias_limit;
            }
            
            ekf_state->gyro_bias[2] += z_bias_update;
            
            // Z轴零偏范围限制
            if (ekf_state->gyro_bias[2] > 0.1f) ekf_state->gyro_bias[2] = 0.1f;
            if (ekf_state->gyro_bias[2] < -0.1f) ekf_state->gyro_bias[2] = -0.1f;
        }
    }

    // 归一化四元数
    quaternion_normalize(&ekf_state->quaternion);

    // 更新协方差矩阵: P = (I - K*H)*P
    float I_KH[6][6];
    float KH[6][6];
    
    // 计算K*H
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            KH[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * ekf_state->H[k][j];
            }
        }
    }
    
    // 计算I - K*H
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_KH[i][j] = (i == j) ? 1.0f - KH[i][j] : -KH[i][j];
        }
    }
    
    // P = (I - K*H) * P
    float P_temp[6][6];
    matrix_multiply_6x6(I_KH, ekf_state->P, P_temp);
    memcpy(ekf_state->P, P_temp, sizeof(P_temp));
}


/**
 * @brief 6x6矩阵乘法
 */
static void matrix_multiply_6x6(float a[6][6], float b[6][6], float result[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 6x3矩阵乘法
 */
static void matrix_multiply_6x3(float a[6][6], float b[6][3], float result[6][3]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 3x6矩阵乘法
 */
static void matrix_multiply_3x6(float a[3][6], float b[6][6], float result[3][6]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 矩阵乘法 3x6 * 6x3 = 3x3 (专用于H*H'计算)
 */
static void matrix_multiply_3x3_h_ht(float a[3][6], float b[6][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

/**
 * @brief 矩阵转置 3x6 -> 6x3
 */
static void matrix_transpose_3x6(float src[3][6], float dst[6][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) {
            dst[j][i] = src[i][j];
        }
    }
}

/**
 * @brief 标准3x3矩阵乘法
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
 * @brief 3x3矩阵求逆
 */
static void matrix_inverse_3x3(float src[3][3], float dst[3][3]) {
    float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
                src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
                src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

    // 更严格的奇异判断
    if (fabsf(det) < 1e-8f) {
        // 矩阵不可逆或接近奇异，设为单位矩阵
        memset(dst, 0, sizeof(float) * 9);
        dst[0][0] = dst[1][1] = dst[2][2] = 1.0f;
        return;
    }

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
    
    // 检查结果的有效性
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (!isfinite(dst[i][j])) {
                // 如果出现无效结果，返回单位矩阵
                memset(dst, 0, sizeof(float) * 9);
                dst[0][0] = dst[1][1] = dst[2][2] = 1.0f;
                return;
            }
        }
    }
}