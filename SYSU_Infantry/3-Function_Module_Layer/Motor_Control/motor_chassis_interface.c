/**
 * @file    motor_chassis_interface.c
 * @brief   Motor Task 和 Chassis Task 之间的数据传递接口实现
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    实现两个模块之间的静态全局变量接口
 */

#include "motor_chassis_interface.h"
#include <string.h>

// 全局变量定义
Chassis_motor_speed_t g_chassis_motor_speed = {0};
Chassis_motor_config_t g_chassis_motor_config = {0};

/**
 * @brief 初始化Motor和Chassis接口
 */
void Motor_chassis_interface_init(void)
{
    memset(&g_chassis_motor_speed, 0, sizeof(Chassis_motor_speed_t));
    memset(&g_chassis_motor_config, 0, sizeof(Chassis_motor_config_t));
    
    // 设置默认配置
    g_chassis_motor_config.chassis_mode = CHASSIS_ZERO_FORCE;
    g_chassis_motor_config.config_updated = 1;  // 初始化时标记需要更新
}

/**
 * @brief Chassis Task设置电机速度目标值
 * @param motor_speeds 四个电机的速度数组
 */
void Chassis_set_motor_speeds(float motor_speeds[4])
{
    for (int i = 0; i < 4; i++) {
        g_chassis_motor_speed.chassis_motor_speed[i] = motor_speeds[i];
    }
    g_chassis_motor_speed.speed_updated = 1;
}

/**
 * @brief Chassis Task设置电机控制配置
 * @param chassis_mode 底盘控制模式
 */
void Chassis_set_motor_config(chassis_mode_e chassis_mode)
{
    g_chassis_motor_config.chassis_mode = chassis_mode;
    g_chassis_motor_config.config_updated = 1;
}

/**
 * @brief Motor Task检查并获取电机速度目标值
 * @param motor_speeds 输出四个电机的速度数组
 * @return uint8_t 1表示有新的速度值，0表示无更新
 */
uint8_t Motor_get_chassis_speeds(float motor_speeds[4])
{
    if (g_chassis_motor_speed.speed_updated) {
        for (int i = 0; i < 4; i++) {
            motor_speeds[i] = g_chassis_motor_speed.chassis_motor_speed[i];
        }
        g_chassis_motor_speed.speed_updated = 0;  // 清除更新标志
        return 1;
    }
    return 0;
}

/**
 * @brief Motor Task检查并获取电机控制配置
 * @param chassis_mode 输出底盘控制模式
 * @return uint8_t 1表示有新的配置，0表示无更新
 */
uint8_t Motor_get_chassis_config(chassis_mode_e *chassis_mode)
{
    if (g_chassis_motor_config.config_updated) {
        *chassis_mode = g_chassis_motor_config.chassis_mode;
        g_chassis_motor_config.config_updated = 0;  // 清除更新标志
        return 1;
    }
    return 0;
}