#ifndef SYSU_INFANTRY_INS_H
#define SYSU_INFANTRY_INS_H

#include "bmi088.h"
#include "algorithm_ekf.h"

/**
 * @brief 姿态数据结构体
 */
typedef struct {
    Acc_raw_data_t accel_raw;       // 加速度计原始数据
    Gyro_raw_data_t gyro_raw;       // 陀螺仪原始数据
    Euler_angles_t euler_angles;    // 解算后的欧拉角 (Roll, Pitch, Yaw)
    float temperature;              // IMU温度
    float dt;                       // 实际采样周期
} attitude_t;

/**
 * @brief INS 模块初始化
 * @note 包括 BMI088 初始化、EKF 初始化、温控初始化
 */
void INS_Init(void);

/**
 * @brief INS 任务主循环函数
 * @note 包含数据读取(DMA)、温控、EKF解算。建议 1kHz 调用。
 */
void INS_Task(void);

/**
 * @brief 获取全局姿态数据指针
 */
const attitude_t* INS_Get_Attitude(void);

#endif // SYSU_INFANTRY_INS_H