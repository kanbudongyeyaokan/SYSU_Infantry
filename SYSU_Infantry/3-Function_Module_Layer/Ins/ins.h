#ifndef SYSU_INFANTRY_INS_H
#define SYSU_INFANTRY_INS_H

#include "bmi088.h" // 包含bmi088.h以获取数据结构定义

/**
 * @brief 姿态数据结构体，包含原始数据和解算后的欧拉角
 */
typedef struct {
    Acc_raw_data_t accel_raw;       // 加速度计原始数据
    Gyro_raw_data_t gyro_raw;       // 陀螺仪原始数据
    Euler_angles_t euler_angles;    // 解算后的欧拉角 (Roll, Pitch, Yaw)
    float yaw_total_angle;          // 累积多圈的Yaw角（单位：度）
    float yaw_rate_dps;             // Yaw角速度（单位：度/秒）
    int32_t yaw_round_count;        // 累积圈数
    Imu_state_e state;              // IMU整体状态
} attitude_t;

/**
 * @brief 获取最新的姿态数据
 * @return 返回一个指向全局姿态数据结构体的常量指针
 * @note 返回的是一个指针，指向的数据会由ins_task实时更新
 */
attitude_t* get_attitude_data(void);

/**
 * @brief 更新姿态数据（由Ins_task调用）
 * @param acc 最新的加速度数据
 * @param gyro 最新的陀螺仪数据
 * @param euler 最新的欧拉角数据
 */
void update_attitude_data(const Acc_raw_data_t* acc,
                          const Gyro_raw_data_t* gyro,
                          const Euler_angles_t* euler,
                          const float* yaw_total_angle,
                          Imu_state_e state);

/**
 * @brief 查询INS当前状态
 */
Imu_state_e ins_get_state(void);

#endif //SYSU_INFANTRY_INS_H
