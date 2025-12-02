/**
 * @Author         : SYSU电控组
 * @Date           : 2025-09-29
 * @Note           : 适配QuaternionEKF移植版
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 四元数结构体
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

/**
 * @brief 欧拉角结构体（单位：度）
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_angles_t;

/**
 * @brief EKF配置参数结构体
 */
typedef struct {
    float process_noise_q;      /*!< 四元数过程噪声 (Q1) */
    float measurement_noise_r;  /*!< 加速度计测量噪声 (R) */
    float gyro_bias_noise;      /*!< 陀螺仪零偏过程噪声 (Q2) */
    float dt;
    bool enable_bias_correction;
    float static_threshold;
    float fading_factor;        /*!< 渐减因子 (lambda) */

    // 兼容字段（移植版未使用）
    bool enable_temp_compensation;
    float temp_coeff_x;
    float temp_coeff_y;
    float temp_coeff_z;
    float baseline_temp;
} Ekf_config_t;

/**
 * @brief EKF状态结构体
 */
typedef struct {
    Quaternion_t quaternion;
    Euler_angles_t euler;
    float gyro_bias[3];          /*!< 陀螺仪零偏 [x,y,z]，z轴恒为0 */

    float P[36];                 /*!< 协方差矩阵 6x6 */

    // 参数存储
    float Q1;                    /*!< 四元数过程噪声 */
    float Q2;                    /*!< 零偏过程噪声 */
    float R_val;                 /*!< 测量噪声 */
    float lambda;                /*!< 渐减因子 */

    bool is_initialized;

    // 状态标志
    bool converge_flag;          /*!< 收敛标志 */
    bool stable_flag;            /*!< 稳定标志 (acc/gyro range check) */
    uint32_t error_count;        /*!< 错误计数 */
    uint32_t update_count;
    float chi_square;            /*!< 卡方值 */

    // Yaw连续性
    float yaw_angle_last;
    int32_t yaw_round_count;
    float yaw_total_angle;

} Ekf_state_t;

typedef enum {
    EKF_NO_ERROR = 0,
    EKF_INIT_ERROR = 0x01,
    EKF_UPDATE_ERROR = 0x02,
} Ekf_error_e;

// 接口函数声明
Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config);
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt);
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]);
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler);
void Ekf_update_yaw_continuity(Ekf_state_t* ekf_state);

// 兼容性接口（新算法不需要外部温度补偿）
void Ekf_set_temperature(Ekf_state_t* ekf_state, float current_temp);
void Ekf_calculate_temp_compensation(const Ekf_state_t* ekf_state, float temp_compensation[3]);
