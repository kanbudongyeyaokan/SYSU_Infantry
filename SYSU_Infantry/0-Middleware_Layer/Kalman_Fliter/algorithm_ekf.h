/**
 * @Author         : SYSU电控组
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-28
 * @Note           : 扩展卡尔曼滤波算法头文件 (支持动态dt)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// EKF相关常数
#define EKF_DEG_TO_RAD (3.14159265f / 180.0f)
#define EKF_RAD_TO_DEG (180.0f / 3.14159265f)
#define EKF_GRAVITY 9.80665f

// 姿态结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_angles_t;

// 配置参数
typedef struct {
    float process_noise_q;      // 过程噪声
    float measurement_noise_r;  // 测量噪声
    float gyro_bias_noise;      // 零偏噪声
    float dt;                   // 默认采样周期(s)
    bool enable_bias_correction;// 是否启用零偏校正
    float static_threshold;     // 静态阈值
} Ekf_config_t;

// 状态结构体
typedef struct {
    Quaternion_t quaternion;
    Euler_angles_t euler;
    float gyro_bias[3];         // [bx, by, bz]

    // 协方差矩阵 (P: 6x6, Q: 6x6, R: 3x3)
    // 为了节省栈空间，这里使用一维数组模拟或直接定义二维
    float P[6][6];
    float Q[6][6];
    float R[3][3];

    // 雅可比矩阵
    float F[6][6];
    float H[3][6];

    // 状态标志
    bool is_initialized;
    bool is_static;
    uint32_t static_count;

    // 配置备份
    float process_noise_q;
    float measurement_noise_r;
    float gyro_bias_noise;
    bool enable_bias_correction;
    float static_threshold;

    // 辅助变量
    float yaw_total_angle;      // 累计Yaw角
    float yaw_angle_last;
    int32_t yaw_round_count;
} Ekf_state_t;

typedef enum {
    EKF_NO_ERROR = 0,
    EKF_INIT_ERROR,
    EKF_UPDATE_ERROR,
} Ekf_error_e;

// --- 核心接口 ---

/**
 * @brief 初始化EKF
 */
Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config);

/**
 * @brief EKF 更新步骤 (核心)
 * @param acc  加速度 [ax, ay, az] (m/s^2)
 * @param gyro 角速度 [gx, gy, gz] (rad/s)
 * @param dt   距离上次更新的时间间隔 (s)
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt);

// 辅助功能
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]);
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler);