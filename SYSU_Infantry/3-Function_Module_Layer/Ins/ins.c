#include "ins.h"
#include <string.h>

// 姿态数据实例
static attitude_t g_attitude;

/**
 * @brief 获取最新的姿态数据
 * @return 返回一个指向全局姿态数据结构体的常量指针
 */
attitude_t* get_attitude_data(void)
{
    return &g_attitude;
}

/**
 * @brief 更新姿态数据（由Ins_task调用）
 * @param acc 最新的加速度数据
 * @param gyro 最新的陀螺仪数据
 * @param euler 最新的欧拉角数据
 */
void update_attitude_data(const Acc_raw_data_t* acc, const Gyro_raw_data_t* gyro, const Euler_angles_t* euler)
{
    if (acc) {
        memcpy(&g_attitude.accel_raw, acc, sizeof(Acc_raw_data_t));
    }
    if (gyro) {
        memcpy(&g_attitude.gyro_raw, gyro, sizeof(Gyro_raw_data_t));
    }
    if (euler) {
        memcpy(&g_attitude.euler_angles, euler, sizeof(Euler_angles_t));
    }
}