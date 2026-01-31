#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "algorithm_pid.h"
#include "string.h"
#include "robot_definitions.h"
#include <stdbool.h>
#include "bsp_wdg.h" // [新增] 引入看门狗

#define MAX_MOTOR_COUNT 16

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9      // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/**DJI电机类型**/
typedef enum
{
    M3508 = 0,
    M2006,
    GM6020
} Djimotor_type_e;

/**电机控制场景枚举**/
typedef chassis_mode_e Djimotor_scene_e;

/**DJI电机控制状态**/
typedef enum
{
    MOTOR_ENABLED = 0,
    MOTOR_STOP
} Djimotor_status_e;

/**DJI电机控制类型**/
typedef enum
{
    OPEN_LOOP = 0b0000,             //开环
    CURRENT_LOOP = 0b0001,          //电流环
    SPEED_LOOP = 0b0010,            //速度环
    ANGLE_LOOP = 0b0100,            //角度环
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
} Djimotor_closeloop_e;

typedef enum
{
    MOTOR_FEEDBACK = 0,
    OTHER_FEEDBACK
} Djimotor_feedback_source_e;

/**DJI电机反馈信息**/
typedef struct
{
    uint16_t last_ecd;          //上一次记录的编码器值
    uint16_t current_ecd;       //当前编码器值
    float current_angle;        //当前电机单圈角度 (0-360)
    float angular_velocity;     //电机转速，单位：【rpm】
    float linear_velocity;      //电机线速度
    int16_t real_current;       //电机实际电流
    uint8_t  motor_temperature; //电机实际温度

    // [新增] 多圈角度相关
    int32_t total_round;        // 总圈数
    float   total_angle;        // 总角度 (累积值，无范围限制，如 3600.5 度)
} Djimotor_measure_t;
#pragma pack()

/**单个场景的PID配置**/
typedef struct
{
    Djimotor_closeloop_e close_loop; //电机控制模式
    Pid_init_t current_pid;     //电流PID参数
    Pid_init_t angle_pid;       //角度PID参数
    Pid_init_t speed_pid;       //速度PID参数
} Djimotor_scene_config_t;
#pragma pack()

/**DJI电机控制器结构体**/
typedef struct
{
    Djimotor_closeloop_e close_loop;       //当前电机模式
    Djimotor_feedback_source_e angle_source;//电机角度反馈值来源
    Djimotor_feedback_source_e speed_source;
    float *other_angle_feedback_ptr; // 其他角度反馈数据指针
    float *other_speed_feedback_ptr; // 其他速度反馈数据指针
    Pid_instance_t current_pid; //电流环pid实例
    Pid_instance_t angle_pid;   //角度环
    Pid_instance_t speed_pid;   //速度环
    float pid_target;           //PID目标量

    // 速度前馈量，用于存放底盘反向速度等外部干扰补偿
    float speed_feedforward;
} Djimotor_controller_t;
#pragma pack()

typedef struct
{
    Djimotor_closeloop_e close_loop;       //电机模式
    Djimotor_feedback_source_e angle_source;//电机角度反馈值来源
    Djimotor_feedback_source_e speed_source;
    float *other_angle_feedback_ptr; // 其他角度反馈数据指针
    float *other_speed_feedback_ptr; // 其他速度反馈数据指针
    Pid_init_t current_pid;     //电流PID初始化
    Pid_init_t angle_pid;       //角度PID初始化
    Pid_init_t speed_pid;       //速度PID初始化
} Djimotor_controller_init_t;
#pragma pack()

/**DJI电机实例**/
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    Djimotor_measure_t motor_measure;   //电机自身运动信息
    Djimotor_controller_t   motor_pid;  //电机自身的PID控制器
    Can_controller_t *can_controller;   //电机自身的CAN管理者
    int16_t deadzone_compensation;      // 电机死区补偿值

    // [新增] 看门狗句柄
    Watchdog_device_t *wdg;

    // 缓存计算出的输出电流值 (等待发送)
    int16_t out_current;
} Djimotor_device_t;
#pragma pack()

/**电机初始化结构体**/
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    int16_t deadzone_compensation;      // 电机死区补偿值
    Djimotor_controller_init_t motor_controller_init;
    Can_init_t can_init;
} Djimotor_init_config_t;
#pragma pack()


/****************************电机方法接口******************************/

// 电机初始化函数
Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config);

//电机控制函数
void Djimotor_set_target(Djimotor_device_t *motor, float target);

// [新增] 单个电机 PID 计算 (应用层调用)
void Djimotor_Calc_Output(Djimotor_device_t *motor);

// [新增] 发送所有电机 CAN 总线数据 (MotorTask 调用)
void Djimotor_Send_All_Bus(void);

//切换电机控制模式
void Djimotor_change_controller(Djimotor_device_t *motor, Djimotor_controller_init_t ctrl_params);

// 获取电机状态
Djimotor_status_e Djimotor_get_status(Djimotor_device_t *motor);

// 获取电机测量数据
Djimotor_measure_t Djimotor_get_measure(Djimotor_device_t *motor);

//设置电机状态
void Djimotor_set_status(Djimotor_device_t *motor, Djimotor_status_e status);

// 设置电机死区补偿值
void Djimotor_set_deadzone(Djimotor_device_t *motor, int16_t deadzone);

// 获取电机死区补偿值
int16_t Djimotor_get_deadzone(Djimotor_device_t *motor);

#endif //_DJI_MOTOR_H