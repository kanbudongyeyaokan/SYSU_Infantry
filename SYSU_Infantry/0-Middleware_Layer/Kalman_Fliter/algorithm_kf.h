/**
 * @Author         : GitHub Copilot
 * @Date           : 2025-09-17
 * @LastEditTime   : 2025-09-17
 * @Note           : 标准卡尔曼滤波算法实现
 * @Copyright(c)   : SYSU Copyright
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 卡尔曼滤波器参数结构体
 */
typedef struct {
    float Q;       /*!< 过程噪声协方差 */
    float R;       /*!< 测量噪声协方差 */
    float dt;      /*!< 采样周期(s)，用于状态预测 */
} Kf_config_t;

/**
 * @brief 标量卡尔曼滤波器状态结构体
 * 适用于单变量情况，如位置、速度、温度等
 */
typedef struct {
    float x;        /*!< 状态估计值 */
    float P;        /*!< 状态估计协方差 */
    float A;        /*!< 状态转移矩阵（标量情况下为系数）*/
    float B;        /*!< 控制输入矩阵（标量情况下为系数）*/
    float H;        /*!< 观测矩阵（标量情况下为系数）*/
    float Q;        /*!< 过程噪声协方差 */
    float R;        /*!< 测量噪声协方差 */
    bool is_initialized; /*!< 初始化标志 */
} Kf_scalar_t;

/**
 * @brief 二阶卡尔曼滤波器状态结构体
 * 适用于位置-速度、角度-角速度等二阶系统
 */
typedef struct {
    float x[2];     /*!< 状态估计值 [位置, 速度] */
    float P[2][2];  /*!< 状态估计协方差矩阵 */
    float A[2][2];  /*!< 状态转移矩阵 */
    float B[2];     /*!< 控制输入矩阵 */
    float H[2];     /*!< 观测矩阵 */
    float Q[2][2];  /*!< 过程噪声协方差矩阵 */
    float R;        /*!< 测量噪声协方差（假设单一测量） */
    bool is_initialized; /*!< 初始化标志 */
} Kf_2order_t;

/**
 * @brief 卡尔曼滤波错误枚举
 */
typedef enum {
    KF_NO_ERROR = 0,
    KF_INIT_ERROR = 0x01,
    KF_UPDATE_ERROR = 0x02,
} Kf_error_e;

// 标量卡尔曼滤波器函数

/**
 * @brief 初始化标量卡尔曼滤波器
 * @param kf 卡尔曼滤波器状态结构体指针
 * @param config 卡尔曼滤波器配置参数
 * @param initial_value 初始状态估计值
 * @param initial_variance 初始状态估计方差
 * @return 错误代码
 */
Kf_error_e Kf_scalar_init(Kf_scalar_t* kf, const Kf_config_t* config, float initial_value, float initial_variance);

/**
 * @brief 标量卡尔曼滤波预测步骤
 * @param kf 卡尔曼滤波器状态结构体指针
 * @param control 控制输入（可选，如果不使用则传入0）
 */
void Kf_scalar_predict(Kf_scalar_t* kf, float control);

/**
 * @brief 标量卡尔曼滤波更新步骤
 * @param kf 卡尔曼滤波器状态结构体指针
 * @param measurement 测量值
 * @return 更新后的状态估计值
 */
float Kf_scalar_update(Kf_scalar_t* kf, float measurement);

/**
 * @brief 标量卡尔曼滤波一步预测和更新
 * @param kf 卡尔曼滤波器状态结构体指针
 * @param measurement 测量值
 * @param control 控制输入（可选，如果不使用则传入0）
 * @return 更新后的状态估计值
 */
float Kf_scalar_step(Kf_scalar_t* kf, float measurement, float control);

// 二阶卡尔曼滤波器函数

/**
 * @brief 初始化二阶卡尔曼滤波器
 * @param kf 二阶卡尔曼滤波器状态结构体指针
 * @param config 卡尔曼滤波器配置参数
 * @param initial_pos 初始位置估计值
 * @param initial_vel 初始速度估计值
 * @param initial_pos_var 初始位置估计方差
 * @param initial_vel_var 初始速度估计方差
 * @return 错误代码
 */
Kf_error_e Kf_2order_init(Kf_2order_t* kf, const Kf_config_t* config, 
                          float initial_pos, float initial_vel,
                          float initial_pos_var, float initial_vel_var);

/**
 * @brief 二阶卡尔曼滤波预测步骤
 * @param kf 二阶卡尔曼滤波器状态结构体指针
 * @param control 控制输入（可选，如果不使用则传入0）
 */
void Kf_2order_predict(Kf_2order_t* kf, float control);

/**
 * @brief 二阶卡尔曼滤波更新步骤
 * @param kf 二阶卡尔曼滤波器状态结构体指针
 * @param measurement 位置测量值
 * @return 更新后的位置估计值
 */
float Kf_2order_update(Kf_2order_t* kf, float measurement);

/**
 * @brief 二阶卡尔曼滤波一步预测和更新
 * @param kf 二阶卡尔曼滤波器状态结构体指针
 * @param measurement 位置测量值
 * @param control 控制输入（可选，如果不使用则传入0）
 * @return 更新后的位置估计值
 */
float Kf_2order_step(Kf_2order_t* kf, float measurement, float control);

/**
 * @brief 获取二阶卡尔曼滤波器的速度估计
 * @param kf 二阶卡尔曼滤波器状态结构体指针
 * @return 当前速度估计值
 */
float Kf_2order_get_velocity(const Kf_2order_t* kf);