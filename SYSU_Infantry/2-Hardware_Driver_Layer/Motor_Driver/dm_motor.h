#ifndef _DM_MOTOR_H
#define _DM_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "string.h"
#include <stdbool.h>
#include "bsp_wdg.h"

/* ======================== 电机最大数量配置 ======================== */
#define DM_MOTOR_MAX_COUNT 8

/* ======================== MIT模式参数范围定义 ======================== */
/* 
 * 注意：实际控制时应使用电机实例中的 params 参数。
 * 不同的电机型号(如J4310, J8009等)参数范围不同。
 */

/* ======================== 枚举类型定义 ======================== */

/** 达妙电机型号 */
typedef enum
{
    DM_J4310_2EC = 0,   // J4310-2EC V1.1 减速电机
} DMmotor_type_e;

/** 电机状态 */
typedef enum
{
    DM_MOTOR_ENABLED = 0,
    DM_MOTOR_STOP
} DMmotor_status_e;

/** 电机控制模式命令 (MIT模式) */
typedef enum
{
    DM_CMD_ENABLE      = 0xFC,  // 使能电机
    DM_CMD_DISABLE     = 0xFD,  // 失能电机
    DM_CMD_SET_ZERO    = 0xFE,  // 设置零点
    DM_CMD_CLEAR_ERROR = 0xFB   // 清除错误
} DMmotor_cmd_e;

/** 电机控制模式 */
typedef enum
{
    DM_CTRL_MIT = 0,        // MIT模式
    DM_CTRL_POS_VEL,        // 位置速度模式
    DM_CTRL_VEL             // 速度模式 (预留)
} DMmotor_control_mode_e;

/* ======================== 结构体定义 ======================== */

/** 电机参数配置 */
typedef struct
{
    float p_max;    // 位置最大值 (rad)
    float v_max;    // 速度最大值 (rad/s)
    float t_max;    // 力矩最大值 (Nm)
    float gravity_mgl; // 静态最大重力矩 (m*g*l) (Nm)
} DMmotor_params_t;

/** 电机测量数据 */
typedef struct
{
    uint8_t id;             // 电机ID
    uint8_t state;          // 电机状态
    float position;         // 当前位置 (rad)
    float last_position;    // 上次位置 (用于多圈计算)
    float velocity;         // 当前速度 (rad/s)
    float torque;           // 当前力矩 (Nm)
    float T_Mos;            // MOS管温度
    float T_Rotor;          // 转子温度
    int32_t total_round;    // 总圈数
    float total_angle;      // 总角度 (累积值)
} DMmotor_measure_t;

/** 电机控制命令缓存 */
typedef struct
{
    // MIT模式参数
    float p_des;            // 目标位置 (rad)
    float v_des;            // 目标速度 (rad/s)
    float kp;               // 位置刚度 (N*m/rad)
    float kd;               // 速度阻尼 (N*m*s/rad)
    float t_ff;             // 前馈力矩 (Nm)
    
    // 位置速度模式参数
    float pos_vel_p_des;    // 目标位置 (rad)
    float pos_vel_v_des;    // 目标速度 (rad/s) (作为最大速度限制)
} DMmotor_cmd_check_t;

/** 电机设备实例 */
typedef struct
{
    char motor_name[16];                    // 电机名称
    DMmotor_type_e motor_type;              // 电机型号
    DMmotor_status_e motor_status;          // 电机状态
    DMmotor_control_mode_e mode;            // 当前控制模式
    uint8_t master_id;                      // 主机ID (用于MIT模式发送)
    
    DMmotor_params_t params;                // 电机参数配置
    
    DMmotor_measure_t motor_measure;        // 电机反馈数据
    DMmotor_cmd_check_t cmd_data;           // 待发送的控制数据
    
    Can_controller_t *can_controller;       // CAN控制器
    Watchdog_device_t *wdg;                 // 看门狗
} DMmotor_device_t;

/** 电机初始化配置 */
typedef struct
{
    char motor_name[16];                // 电机名称
    DMmotor_type_e motor_type;          // 电机型号
    DMmotor_status_e motor_status;      // 初始状态
    uint8_t master_id;                  // 主机ID (1-32)
    DMmotor_params_t params;            // 电机参数配置
    Can_init_t can_init;                // CAN配置
} DMmotor_init_config_t;

/* ======================== 函数接口 ======================== */

/**
 * @brief 初始化达妙电机
 * @param config 电机初始化配置
 * @return 电机设备指针，失败返回NULL
 */
DMmotor_device_t *DM_Motor_Init(DMmotor_init_config_t *config);

/**
 * @brief 设置MIT模式控制参数
 * @param motor 电机实例
 * @param p_des 目标位置 (rad)
 * @param v_des 目标速度 (rad/s)
 * @param kp    位置刚度 (0-500)
 * @param kd    速度阻尼 (0-5)
 * @param t_ff  前馈力矩 (Nm)
 */
void DMmotor_control_mit(DMmotor_device_t *motor, float p_des, float v_des, float kp, float kd, float t_ff);

/**
 * @brief 带重力前馈的位置控制 (MIT模式)
 * @param motor 电机实例
 * @param target_p 目标角度 (rad)
 * @param kp 位置刚度
 * @param kd 速度阻尼
 * @param horizontal_offset 臂部处于水平面时的编码器读数 (用于角度校准)
 * @note 自动计算重力前馈 t_ff = mgl * cos(current_pos - offset)
 */
void DMmotor_control_mit_with_gravity(DMmotor_device_t *motor, float target_p, float kp, float kd, float horizontal_offset);

/**
 * @brief 设置位置速度模式控制参数
 * @param motor 电机实例
 * @param p_des 目标位置 (rad)
 * @param v_des 目标速度 (rad/s) (即最大速度限制)
 * @note 此模式下电机运行特定的位置速度闭环
 */
void DMmotor_control_pos_vel(DMmotor_device_t *motor, float p_des, float v_des);

/**
 * @brief 使能电机
 */
void DMmotor_enable(DMmotor_device_t *motor);

/**
 * @brief 停止电机 (输出零力矩或失能)
 */
void DMmotor_stop(DMmotor_device_t *motor);

/**
 * @brief 设置电机零点 (当前位置设为0)
 */
void DMmotor_set_zero(DMmotor_device_t *motor);

/**
 * @brief 清除电机错误
 */
void DMmotor_clear_error(DMmotor_device_t *motor);

/**
 * @brief 获取电机状态
 */
DMmotor_status_e DMmotor_get_status(DMmotor_device_t *motor);

/**
 * @brief 获取电机测量数据
 */
DMmotor_measure_t DMmotor_get_measure(DMmotor_device_t *motor);

/**
 * @brief 发送所有电机的控制命令
 * @note 需要在任务中周期调用
 */
void DMmotor_control_all(void);

#endif // _DM_MOTOR_H
