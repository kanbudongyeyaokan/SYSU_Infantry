#ifndef SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H
#define SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H

#include "chassis.h"
#include "dji_motor.h"

// 功率控制模块初始化 (包含能量环PID初始化)
void Chassis_Power_Control_Init(void);

// RLS 动态参数更新 (仅在你拥有硬件级底盘真实功率采样时调用，否则无需调用)
void Chassis_Power_RLS_Update(Djimotor_device_t *motors[4]);

// 核心功率约束执行函数 (大P误差分配 + 负功回收 + 二次方程逆解)
void Chassis_Power_Control(Djimotor_device_t *motors[4]);

#endif // SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H