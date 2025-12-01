#ifndef __INS_H
#define __INS_H

#include "bmi088.h" // 包含 Euler_angles_t 等定义

// 姿态数据导出结构体
typedef struct {
    Euler_angles_t euler_angles; // 欧拉角 (度)
    Acc_raw_data_t accel_raw;    // 加速度 (m/s^2)
    Gyro_raw_data_t gyro_raw;    // 角速度 (rad/s)
    float temperature;           // 温度
    float dt;                    // 运行周期
} attitude_t;

void INS_Init(void);
void INS_Task(void);
const attitude_t* INS_Get_Attitude(void);

#endif