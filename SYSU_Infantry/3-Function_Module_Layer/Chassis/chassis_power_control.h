#ifndef SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H
#define SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "chassis.h"
#include "dji_motor.h"

/* 功率闭环反馈混合系数 (0~1)，越大越信任实测功率 */
#define CHASSIS_POWER_FB_RATIO_DEFAULT   0.7f

/* 4 轮麦轮底盘功率控制结构体定义 */
typedef struct
{
	float i_cmd[4];      /* PID 输出原始目标电流 */
	float w_fdb[4];      /* 电机反馈转速 */
	float p_limit;       /* 裁判系统动态功率上限(W) */
	float e_buffer;      /* 当前缓冲能量(J) */
} chassis_power_ctrl_input_t;

typedef struct
{
	float k_t;                /* 功率估算系数 K_t */
	float p_static;           /* 单轮静态功耗补偿 */
	float danger_energy_line; /* 缓冲能量防线，默认 30J */
	float k_p_buffer;         /* 危险区功率衰减系数 */
	float p_min_allow;        /* 允许最小功率，防止 <=0 */
	float fb_ratio;           /* 闭环反馈混合系数 (0~1)，实测功率权重 */
} chassis_power_ctrl_param_t;

typedef struct
{
	float p_wheel_esti[4]; /* 四轮估算功率 */
	float p_estimated;     /* 估算总功率 (前馈) */
	float p_measured;      /* 实测功率 (反馈) */
	float p_total;         /* 混合后用于决策的功率 */
	float p_max_allow;     /* 当前允许最大功率 */
	float alpha;           /* 等比例缩放系数 */
	float i_out[4];        /* 缩放后的目标电流 */
} chassis_power_ctrl_output_t;

/*
 * 核心函数：
 * 1) 单轮功率估算
 * 2) 总功率计算
 * 3) 缓冲能量防线动态限功
 * 4) 四轮等比例电流缩放
 */
void Chassis_Power_CalcAndScale(const chassis_power_ctrl_input_t *input,
                                const chassis_power_ctrl_param_t *param,
                                chassis_power_ctrl_output_t *output,
                                float p_measured);

/* 与现有电机驱动对接的包装函数，直接修改 motors[i]->out_current */
void Chassis_Power_Control_Init(void);
void Chassis_Power_Control(Djimotor_device_t *motors[4], float p_measured);

#endif // SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H
