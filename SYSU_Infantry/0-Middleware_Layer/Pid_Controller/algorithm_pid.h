#ifndef _ALGORITHM_PID_H
#define _ALGORITHM_PID_H

#include "main.h"
#include <stdint.h>

/* PID优化环节标志 (使用Hex消除编译器警告) */
typedef enum
{
    PID_OPTIMIZE_NONE           = 0x00, // 无优化
    PID_OUTPUT_LIMIT            = 0x01, // 输出限幅 & 积分限幅
    PID_DIFFERENTIAL_GO_FIRST   = 0x02, // 微分先行 (对测量值微分而不是误差)
    PID_TRAPEZOID_INTERGRAL     = 0x04, // 梯形积分 (提高积分精度)
    PID_OUTPUT_FILTER           = 0x08, // 输出滤波 (低通)
    PID_FEEDFOWARD              = 0x10, // 前馈控制
    PID_INTEGRAL_ANTI_WINDUP = 0x20, // 积分抗饱和 (变速积分等策略)
    PID_INTEGRAL_SEPARATION = 0x40, // 积分分离 (误差大时关闭积分，防止超调)
} Pid_optimization_e;

/* PID结构体 */
typedef struct
{
    // --- PID基础参数 ---
    float kp;
    float ki;
    float kd;

    float max_iout;     // 最大积分输出 (积分限幅)
    float max_out;      // 最大总输出 (输出限幅)
    float deadband;     // 死区 (误差绝对值小于此值时不调节)

    // --- 运行时数据 ---
    float measure;      // 当前测量值
    float last_measure; // 上一次测量值
    float target;       // 当前目标值
    float error;        // 当前误差
    float last_error;   // 上一次误差

    float ITerm;        // 当前周期的积分增量 (Ki * err * dt)
    float Pout;         // P项输出
    float Iout;         // I项累计输出
    float Dout;         // D项输出
    float Output;       // 最终总输出

    float Last_Output;  // 上一次的总输出 (用于滤波)

    // --- 优化选项 ---
    uint32_t optimization;          // 优化选项位掩码
    float feedfoward_coefficient;   // 前馈系数
    float LPF_coefficient;          // 低通滤波器系数 (0~1, 越小滤波越强)
    float integral_separation_threshold;    // 积分分离阈值 (|error| > 此值时关闭积分)

    // --- 计时相关 ---
    uint32_t dwt_counter;   // DWT 计数器快照
    float    dt;            // 当前控制周期 (单位: 秒) [重要: 必须是float]

} Pid_instance_t;

/* PID初始化配置结构体 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_iout;
    float max_out;
    float deadband;

    uint32_t optimization;          // 优化选项
    float feedfoward_coefficient;
    float LPF_coefficient;
    float integral_separation_threshold; // 积分分离阈值
} Pid_init_t;

/**
 * @brief 初始化PID实例
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void Pid_init(Pid_instance_t *pid, Pid_init_t *config);

/**
 * @brief 计算PID输出 (包含自动dt计算)
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param target  设定值
 * @return float  PID计算输出
 */
float Pid_calculate(Pid_instance_t *pid, float measure, float target);

/**
 * @brief 清除PID状态 (用于电机停止时重置积分)
 * @param pid PID实例指针
 */
void Pid_reset(Pid_instance_t *pid);

#endif //_ALGORITHM_PID_H