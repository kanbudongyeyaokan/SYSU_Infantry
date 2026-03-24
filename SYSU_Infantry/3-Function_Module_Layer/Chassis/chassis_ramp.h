/**
 * @file    chassis_ramp.h
 * @brief   底盘斜坡缓冲控制器接口
 */
#ifndef __CHASSIS_RAMP_H
#define __CHASSIS_RAMP_H

#include <stdint.h>

// 斜坡控制器结构体
typedef struct {
    float ramp_vx;  // 当前 X 轴平滑速度
    float ramp_vy;  // 当前 Y 轴平滑速度
    float step;     // 每次调用的最大步长（加速度限制）
} Chassis_Ramp_t;

/**
 * @brief  初始化斜坡控制器
 * @param  ramp_inst 控制器实例指针
 * @param  step      步长限制 (决定加减速的猛烈程度)
 */
void Chassis_Ramp_Init(Chassis_Ramp_t *ramp_inst, float step);

/**
 * @brief  复位斜坡控制器（通常在电机失能时调用，防止重新使能时暴走）
 * @param  ramp_inst 控制器实例指针
 */
void Chassis_Ramp_Reset(Chassis_Ramp_t *ramp_inst);

/**
 * @brief  更新斜坡状态并输出平滑后的速度
 * @param  ramp_inst 控制器实例指针
 * @param  target_vx 目标 VX
 * @param  target_vy 目标 VY
 * @param  out_vx    平滑后的 VX 输出地址
 * @param  out_vy    平滑后的 VY 输出地址
 */
void Chassis_Ramp_Update(Chassis_Ramp_t *ramp_inst, float target_vx, float target_vy, float *out_vx, float *out_vy);

#endif // __CHASSIS_RAMP_H