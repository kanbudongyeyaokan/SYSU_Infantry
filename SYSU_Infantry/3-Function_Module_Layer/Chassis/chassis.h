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
#include "dji_motor.h"

/**********************************底盘数据结构体************************************ */
/**
 * @brief 底盘物理参数结构体
 */
typedef struct {
    float wheel_radius;        // 轮子半径 (mm)
    float wheel_perimeter;     // 轮子周长 (mm)
    float chassis_radius;      // 底盘半径，中心到轮子距离 (mm)
    float wheel_base;          // 轴距 (mm)
    float half_wheel_base;      //半轴距（mm）
    float track_width;         //轮距 (mm)
    float half_track_width;    // 半轮距 (mm) - 用于麦克纳姆轮底盘
    chassis_type_e chassis_type; // 底盘类型
} Chassis_params_t;

/**
 * @brief 底盘解算结果结构体
 */
typedef struct {
    float motor_speed[4];      // 四个电机的转速 (rpm)
} Chassis_output_t;


/*********************************底盘方法接口************************************* */
/**
 * @brief 底盘功能模块初始化
 * @param params 底盘物理参数
 */
void Chassis_init();

/**
 * @brief 底盘任务初始化
 */
void Chassis_task_init(void);

/**
 * @brief 处理底盘控制指令
 */
// void Chassis_handle_command(void);
void Chassis_Update_Control(const Chassis_cmd_send_t *cmd);

/**
 * @brief 底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 * @note  将接收到的底盘控制量转换为4个电机的转速
 */
void Chassis_kinematics_solve(const Chassis_cmd_send_t *cmd, Chassis_output_t *output);




#endif //SYSU_INFANTRY_CHASSIS_FUNCTION_H