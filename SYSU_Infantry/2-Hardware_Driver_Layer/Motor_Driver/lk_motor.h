#ifndef _LK_MOTOR_H
#define _LK_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "algorithm_pid.h"
#include "string.h"
#include <stdbool.h>

#define MAX_LK_MOTOR_COUNT 16

// 领控协议单位转换系数
#define LK_ANGLE_TO_DEG 0.01f       // 协议单位 0.01度
#define LK_SPEED_TO_RPM 0.166667f   // 协议单位 1dps -> RPM (1 dps = 60/360 rpm = 1/6 rpm)
#define LK_RPM_TO_DPS   6.0f        // RPM -> dps
#define LK_RPM_TO_SPEED_CTRL 600.0f // RPM -> 0.01dps (用于速度闭环控制命令)

/**领控电机类型**/
typedef enum
{
    LK_MG4010 = 0,
    LK_MG5010,
    LK_MS6015,
    LK_MG6012,
    LK_MG7010
} Lkmotor_type_e;

/**领控电机控制状态**/
typedef enum
{
    LK_MOTOR_ENABLED = 0,
    LK_MOTOR_STOP
} Lkmotor_status_e;

/**领控电机控制模式 (对应内部指令)**/
typedef enum
{
    LK_OPEN_LOOP = 0,           // 开环 (0xA0)
    LK_CURRENT_LOOP,            // 转矩闭环 (0xA1)
    LK_SPEED_LOOP,              // 速度闭环 (0xA2)
    LK_ANGLE_LOOP,              // 多圈位置闭环 (0xA3/0xA4)
    LK_SPEED_AND_CURRENT_LOOP,  // 兼容定义
    LK_ANGLE_AND_SPEED_LOOP     // 兼容定义
} Lkmotor_closeloop_e;

/**领控电机反馈信息**/
typedef struct
{
    int8_t temperature;         // 电机温度
    int16_t current_torque;     // 当前转矩电流/功率
    int16_t speed_dps;          // 速度 (dps)
    float angular_velocity;     // 转速 (RPM)
    uint16_t encoder_raw;       // 编码器原始值 (0-65535)

    // 多圈角度处理
    uint16_t last_encoder;
    int32_t total_round;
    float total_angle;          // 累积角度 (度)
    bool total_angle_initialized;
} Lkmotor_measure_t;

/**领控电机控制器结构体**/
typedef struct
{
    Lkmotor_closeloop_e close_loop;
    float pid_target;           // 目标值
    float max_speed;            // 最大速度限制
    Pid_instance_t current_pid;
    Pid_instance_t angle_pid;
    Pid_instance_t speed_pid;
} Lkmotor_controller_t;

/**领控电机初始化参数结构体**/
typedef struct
{
    Lkmotor_closeloop_e close_loop;
    Pid_init_t current_pid;
    Pid_init_t angle_pid;
    Pid_init_t speed_pid;
} Lkmotor_controller_init_t;

/**领控电机实例**/
typedef struct
{
    char motor_name[16];
    Lkmotor_type_e motor_type;
    Lkmotor_status_e motor_status;
    Lkmotor_measure_t motor_measure;
    Lkmotor_controller_t motor_controller;
    Can_controller_t *can_controller;
    bool hardware_run_cmd_sent; // 握手标志位
} Lkmotor_device_t;

/**电机初始化配置结构体**/
typedef struct
{
    char motor_name[16];
    Lkmotor_type_e motor_type;
    Lkmotor_status_e motor_status;
    Lkmotor_controller_init_t motor_controller_init;
    Can_init_t can_init;
} Lkmotor_init_config_t;

/* ---------------- 接口函数 ---------------- */

Lkmotor_device_t *Lk_Motor_Init(Lkmotor_init_config_t *config);
void Lkmotor_set_target(Lkmotor_device_t *motor, float target);
void Lkmotor_control_all(void);
Lkmotor_measure_t Lkmotor_get_measure(Lkmotor_device_t *motor);
void Lkmotor_set_status(Lkmotor_device_t *motor, Lkmotor_status_e status);

#endif // _LK_MOTOR_H