/**
 * @file    chassis_scurve.h
 * @brief   底盘二阶 S 曲线动态跟踪器（专为高频遥控输入设计）
 */
#ifndef __CHASSIS_SCURVE_H
#define __CHASSIS_SCURVE_H

#include <stdint.h>

// 单轴 S 曲线控制器结构体
typedef struct {
    float current_v;    // 当前速度
    float current_a;    // 当前加速度
    
    // 调参参数
    float kp_acc;       // 加速响应系数（越大加速越快）
    float kp_brk;       // 刹车响应系数（建议比 kp_acc 大，实现急刹）
    float kd;           // 阻尼系数（防止超调，一般设为 2*sqrt(kp) 附近）
    float max_a;        // 绝对最大加速度限制（防超功率、防翻车）
    
    float dt;           // 控制周期 (例如 1kHz 循环就是 0.001f)
} SCurve_Axis_t;

// 底盘双轴 S 曲线实例
typedef struct {
    SCurve_Axis_t vx_axis;
    SCurve_Axis_t vy_axis;
} Chassis_SCurve_t;

/**
 * @brief  初始化底盘 S 曲线控制器
 * @param  scurve_inst 控制器实例指针
 */
void Chassis_SCurve_Init(Chassis_SCurve_t *scurve_inst);

/**
 * @brief  复位控制器（失能时调用，清空速度和加速度累积）
 * @param  scurve_inst 控制器实例指针
 */
void Chassis_SCurve_Reset(Chassis_SCurve_t *scurve_inst);

/**
 * @brief  更新 S 曲线状态，输出平滑后的速度
 * @param  scurve_inst 控制器实例指针
 * @param  target_vx   目标 VX
 * @param  target_vy   目标 VY
 * @param  out_vx      平滑后的 VX 输出
 * @param  out_vy      平滑后的 VY 输出
 */
void Chassis_SCurve_Update(Chassis_SCurve_t *scurve_inst, float target_vx, float target_vy, float *out_vx, float *out_vy);

#endif // __CHASSIS_SCURVE_H