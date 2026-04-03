/**
 * @file    chassis_scurve.c
 * @brief   底盘二阶 S 曲线动态跟踪器实现
 */
#include "chassis_scurve.h"
#include <stddef.h>
#include <math.h>

// 辅助函数：符号判断
static float Sign(float val) {
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

// 辅助函数：限幅
static float Clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 单轴核心更新算法
static void SCurve_Axis_Update(SCurve_Axis_t *axis, float target, float *out) {
    float err = target - axis->current_v;
    float current_kp = axis->kp_acc;

    // 判断是加速阶段还是减速(刹车/反向)阶段，实现非对称手感
    // 如果目标为0，或者目标方向与当前速度方向相反，说明在刹车
    if (fabsf(target) < 0.01f || (target * axis->current_v < 0.0f)) {
        current_kp = axis->kp_brk;
    }

    // 核心：基于二阶 PD 阻尼系统的加速度计算 (产生 S 曲线)
    // a_next = a_current + (Kp * error - Kd * a_current) * dt
    float desired_jerk = current_kp * err - axis->kd * axis->current_a;
    axis->current_a += desired_jerk * axis->dt;

    // 加速度限幅 (保护云台，防止电机瞬间索要过大电流)
    axis->current_a = Clamp(axis->current_a, -axis->max_a, axis->max_a);

    // 更新当前速度
    axis->current_v += axis->current_a * axis->dt;

    // 微小误差消除，防止稳态抖动
    if (fabsf(err) < 1.0f && fabsf(axis->current_a) < 1.0f) {
        axis->current_v = target;
        axis->current_a = 0.0f;
    }

    *out = axis->current_v;
}

void Chassis_SCurve_Init(Chassis_SCurve_t *scurve_inst) {
    if (scurve_inst == NULL) return;

    // 初始化 X 轴参数
    scurve_inst->vx_axis.current_v = 0.0f;
    scurve_inst->vx_axis.current_a = 0.0f;
    scurve_inst->vx_axis.kp_acc = 800.0f;  // 加速系数：决定起步冲力
    scurve_inst->vx_axis.kp_brk = 900.0f;  // 刹车系数：比加速大，确保松摇杆即停
    scurve_inst->vx_axis.kd     = 80.0f;   // 阻尼系数：防止冲过头，通常调在 2*sqrt(kp_acc) 到 3*sqrt(kp_acc) 之间
    scurve_inst->vx_axis.max_a  = 20000.0f; // 最大加速度限制：防止超功率侧翻
    scurve_inst->vx_axis.dt     = 0.001f;  // 1ms 控制周期

    // 初始化 Y 轴参数 (可以与 X 轴相同)
    scurve_inst->vy_axis = scurve_inst->vx_axis; 
}

void Chassis_SCurve_Reset(Chassis_SCurve_t *scurve_inst) {
    if (scurve_inst == NULL) return;
    scurve_inst->vx_axis.current_v = 0.0f;
    scurve_inst->vx_axis.current_a = 0.0f;
    scurve_inst->vy_axis.current_v = 0.0f;
    scurve_inst->vy_axis.current_a = 0.0f;
}

void Chassis_SCurve_Update(Chassis_SCurve_t *scurve_inst, float target_vx, float target_vy, float *out_vx, float *out_vy) {
    if (scurve_inst == NULL) return;
    
    SCurve_Axis_Update(&scurve_inst->vx_axis, target_vx, out_vx);
    SCurve_Axis_Update(&scurve_inst->vy_axis, target_vy, out_vy);
}