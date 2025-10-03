/**
 * @Author         : GitHub Copilot
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-17
 * @Note           : 扩展卡尔曼滤波算法实现
 * @Copyright(c)   : SYSU Copyright
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// EKF相关常数定义
#define EKF_DEG_TO_RAD (3.14159265f / 180.0f)
#define EKF_RAD_TO_DEG (180.0f / 3.14159265f)
#define EKF_GRAVITY 9.80665f
#define EKF_STATIC_THRESHOLD_DEFAULT 0.2f
#define EKF_STATIC_COUNT_THRESHOLD 100

/**
 * @brief 四元数结构体
 */
typedef struct {
    float q0; /*!< 四元数实部 */
    float q1; /*!< 四元数虚部i */
    float q2; /*!< 四元数虚部j */
    float q3; /*!< 四元数虚部k */
} Quaternion_t;

/**
 * @brief 欧拉角结构体（单位：度）
 */
typedef struct {
    float roll;  /*!< 横滚角 */
    float pitch; /*!< 俯仰角 */
    float yaw;   /*!< 偏航角 */
} Euler_angles_t;

/**
 * @brief EKF配置参数结构体
 */
typedef struct {
    float process_noise_q;      /*!< 过程噪声协方差 */
    float measurement_noise_r;  /*!< 测量噪声协方差 */
    float gyro_bias_noise;      /*!< 陀螺仪零偏噪声 */
    float dt;                   /*!< 采样周期(s) */
    bool enable_bias_correction;/*!< 启用零偏校正 */
    float static_threshold;     /*!< 静态检测阈值 */
    float fading_factor;        /*!< 渐减系数，防止过度收敛，建议0.9996 */
    
    // 温度补偿配置
    bool enable_temp_compensation; /*!< 启用温度补偿 */
    float temp_coeff_x;          /*!< X轴温度系数(rad/s/°C) */
    float temp_coeff_y;          /*!< Y轴温度系数(rad/s/°C) */
    float temp_coeff_z;          /*!< Z轴温度系数(rad/s/°C) */
    float baseline_temp;         /*!< 基准温度(°C) */
} Ekf_config_t;

/**
 * @brief EKF状态结构体
 */
typedef struct {
    Quaternion_t quaternion;     /*!< 姿态四元数 */
    Euler_angles_t euler;        /*!< 欧拉角 */
    float gyro_bias[3];          /*!< 陀螺仪零偏 [x,y,z] */
    float P[6][6];               /*!< 协方差矩阵 6x6 (四元数4 + 零偏2) */
    float Q[6][6];               /*!< 过程噪声协方差矩阵 */
    float R[3][3];               /*!< 测量噪声协方差矩阵 */
    float F[6][6];               /*!< 状态转移矩阵 */
    float H[3][6];               /*!< 观测矩阵 */
    bool is_initialized;         /*!< 初始化标志 */
    uint16_t static_count;       /*!< 静态计数器 */
    bool is_static;              /*!< 静态状态标志 */
    
    // 新增状态变量，基于参考代码
    bool converge_flag;          /*!< 收敛标志 */
    uint32_t error_count;        /*!< 错误计数 */
    uint32_t update_count;       /*!< 更新计数 */
    float chi_square;            /*!< 卡方检验值 */
    float adaptive_gain_scale;   /*!< 自适应增益缩放因子 */
    float acc_lpf_coef;          /*!< 加速度低通滤波系数 */
    float acc_filtered[3];       /*!< 滤波后的加速度 */
    
    // Yaw角度连续性处理
    float yaw_angle_last;        /*!< 上次yaw角度 */
    int32_t yaw_round_count;     /*!< yaw转数计数 */
    float yaw_total_angle;       /*!< yaw总角度（包含转数） */
    
    // 配置参数
    float process_noise_q;       /*!< 过程噪声协方差 */
    float measurement_noise_r;   /*!< 测量噪声协方差 */
    float gyro_bias_noise;       /*!< 陀螺仪零偏噪声 */
    bool enable_bias_correction; /*!< 启用零偏校正 */
    float static_threshold;      /*!< 静态检测阈值 */
    float fading_factor;         /*!< 渐减系数，防止过度收敛 */
    
    // 温度补偿相关
    float temperature;           /*!< 当前温度(°C) */
    float temp_baseline;         /*!< 基准温度(°C) */
    float temp_coeff[3];         /*!< 温度补偿系数(rad/s/°C) [x,y,z] */
    bool enable_temp_comp;       /*!< 启用温度补偿 */
} Ekf_state_t;

/**
 * @brief EKF错误枚举
 */
typedef enum {
    EKF_NO_ERROR = 0,
    EKF_INIT_ERROR = 0x01,
    EKF_UPDATE_ERROR = 0x02,
} Ekf_error_e;

// EKF公共接口函数

/**
 * @brief 初始化EKF参数和状态
 * @param ekf_state EKF状态结构体指针
 * @param ekf_config EKF配置参数指针
 * @return 错误代码
 */
Ekf_error_e Ekf_init(Ekf_state_t* ekf_state, Ekf_config_t* ekf_config);

/**
 * @brief 使用新的传感器数据更新EKF
 * @param ekf_state EKF状态结构体指针
 * @param acc 加速度数据数组[x,y,z]，单位m/s^2
 * @param gyro 角速度数据数组[x,y,z]，单位rad/s
 * @return 错误代码
 */
Ekf_error_e Ekf_update(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3], float dt);

/**
 * @brief 检测静态状态
 * @param ekf_state EKF状态结构体指针
 * @param acc 加速度数据数组[x,y,z]
 * @param gyro 角速度数据数组[x,y,z]
 * @return 是否处于静态状态
 */
bool Ekf_detect_static_state(Ekf_state_t* ekf_state, const float acc[3], const float gyro[3]);

/**
 * @brief 设置温度补偿参数
 * @param ekf_state EKF状态结构体指针
 * @param current_temp 当前温度(°C)
 */
void Ekf_set_temperature(Ekf_state_t* ekf_state, float current_temp);

/**
 * @brief 计算温度补偿值
 * @param ekf_state EKF状态结构体指针
 * @param temp_compensation 输出的温度补偿值[x,y,z]
 */
void Ekf_calculate_temp_compensation(const Ekf_state_t* ekf_state, float temp_compensation[3]);

/**
 * @brief 四元数转欧拉角
 * @param q 四元数指针
 * @param euler 欧拉角指针
 */
void Ekf_quaternion_to_euler(Quaternion_t* q, Euler_angles_t* euler);

/**
 * @brief 更新yaw角度连续性处理
 * @param ekf_state EKF状态结构体指针
 */
void Ekf_update_yaw_continuity(Ekf_state_t* ekf_state);