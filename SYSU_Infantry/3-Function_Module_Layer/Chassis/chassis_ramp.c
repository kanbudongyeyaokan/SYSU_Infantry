/**
 * @file    chassis_ramp.c
 * @brief   底盘斜坡缓冲控制器实现
 */
#include "chassis_ramp.h"
#include <stddef.h>

// 绝对值斜坡函数
static float Apply_Ramp(float target, float current, float step) {
    if (target > current) {
        return (current + step > target) ? target : (current + step);
    } else if (target < current) {
        return (current - step < target) ? target : (current - step);
    }
    return target;
}

void Chassis_Ramp_Init(Chassis_Ramp_t *ramp_inst, float step) {
    if (ramp_inst == NULL) return;
    ramp_inst->ramp_vx = 0.0f;
    ramp_inst->ramp_vy = 0.0f;
    ramp_inst->step = step;
}

void Chassis_Ramp_Reset(Chassis_Ramp_t *ramp_inst) {
    if (ramp_inst == NULL) return;
    ramp_inst->ramp_vx = 0.0f;
    ramp_inst->ramp_vy = 0.0f;
}

void Chassis_Ramp_Update(Chassis_Ramp_t *ramp_inst, float target_vx, float target_vy, float *out_vx, float *out_vy) {
    if (ramp_inst == NULL) return;
    
    // 分别对 VX 和 VY 进行斜坡滤波
    ramp_inst->ramp_vx = Apply_Ramp(target_vx, ramp_inst->ramp_vx, ramp_inst->step);
    ramp_inst->ramp_vy = Apply_Ramp(target_vy, ramp_inst->ramp_vy, ramp_inst->step);
    
    // 输出平滑后的值
    if (out_vx != NULL) *out_vx = ramp_inst->ramp_vx;
    if (out_vy != NULL) *out_vy = ramp_inst->ramp_vy;
}