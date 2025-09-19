/**
 * @file    chassis.h
 * @brief   底盘功能模块头文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    提供底盘运动学解算功能，支持多种底盘类型
 */

#ifndef SYSU_INFANTRY_CHASSIS_FUNCTION_H
#define SYSU_INFANTRY_CHASSIS_FUNCTION_H

#include "main.h"
#include "robot_definitions.h"
#include "decision_making.h"

/**
 * @brief 底盘物理参数结构体
 */
typedef struct {
    float wheel_radius;        // 轮子半径 (m)
    float chassis_radius;      // 底盘半径，中心到轮子距离 (m)
    float wheel_base;          // 轮距 (m) - 用于差速底盘
    float track_width;         // 轮宽 (m) - 用于麦克纳姆轮底盘
    chassis_type_e chassis_type; // 底盘类型
} Chassis_params_t;

/**
 * @brief 底盘解算结果结构体
 */
typedef struct {
    float motor_speed[4];      // 四个电机的转速 (rpm)
    uint8_t motor_count;       // 使用的电机数量
} Chassis_output_t;

/**
 * @brief 底盘功能模块初始化
 * @param params 底盘物理参数
 */
void Chassis_init(const Chassis_params_t *params);

/**
 * @brief 底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
void Chassis_kinematics_solve(const Chassis_cmd_send_t *cmd, Chassis_output_t *output);

/**
 * @brief 设置底盘类型
 * @param chassis_type 底盘类型
 */
void Chassis_set_type(chassis_type_e chassis_type);

/**
 * @brief 获取当前底盘类型
 * @return chassis_type_e 当前底盘类型
 */
chassis_type_e Chassis_get_type(void);

#endif //SYSU_INFANTRY_CHASSIS_FUNCTION_H