#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "algorithm_pid.h"
#include "string.h"
#include "robot_definitions.h"

#define MAX_MOTOR_COUNT 16
#define MAX_MOTOR_SCENES 5  // 最大支持的场景数量

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制


/**DJI电机类型**/
typedef enum
{
    M3508 = 0,
    M2006,
    GM6020
}Djimotor_type_e;

/**电机控制场景枚举**/
// 使用robot_definitions.h中的chassis_mode_e替代原有的场景枚举
// 为保持兼容性，保留场景枚举的别名定义
typedef chassis_mode_e Djimotor_scene_e;

/**DJI电机控制状态**/
typedef enum
{
    MOTOR_ENABLED = 0,
    MOTOR_STOP
}Djimotor_status_e;

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
    MOTOR_FEEDBACK =0,
    OTHER_FEEDBACK
}Djimotor_feedback_source_e;

/**DJI电机反馈信息**/
#pragma pack(1)
typedef struct
{
    uint16_t last_ecd;          //上一次记录的编码器值，编码器值（0-8191）
    uint16_t current_ecd;       //当前编码器值
    float current_angle;        //当前电机角度
    float angular_velocity;     //电机转速，单位：【rpm】
    float linear_velocity;      //电机线速度
    int16_t real_current;       //电机实际电流
    uint8_t  motor_temperature;//电机实际温度
}Djimotor_measure_t;
#pragma pack()

/**单个场景的PID配置**/
#pragma pack(1)
typedef struct
{
    Djimotor_closeloop_e close_loop;       //电机控制模式
    Pid_init_t current_pid;     //电流PID参数
    Pid_init_t angle_pid;       //角度PID参数
    Pid_init_t speed_pid;       //速度PID参数
}Djimotor_scene_config_t;
#pragma pack()

/**DJI电机控制器结构体**/
#pragma pack(1)
typedef struct
{
    Djimotor_closeloop_e close_loop;       //当前电机模式
    Djimotor_feedback_source_e angle_source;//电机角度反馈值来源
    Djimotor_feedback_source_e speed_source;
    float *other_angle_feedback_ptr; // 其他角度反馈数据指针
    float *other_speed_feedback_ptr; // 其他速度反馈数据指针,单位为度/秒

    Pid_instance_t current_pid; //电流环pid实例
    Pid_instance_t angle_pid;   //角度环
    Pid_instance_t speed_pid;   //速度环

    float pid_target;           //PID目标量
    
    // 场景管理
    chassis_mode_e current_scene;                        // 当前场景
    Djimotor_scene_config_t scene_configs[MAX_MOTOR_SCENES]; // 所有场景的配置
}Djimotor_controller_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    Djimotor_closeloop_e close_loop;       //电机模式
    Djimotor_feedback_source_e angle_source;//电机角度反馈值来源
    Djimotor_feedback_source_e speed_source;
    float *other_angle_feedback_ptr; // 其他角度反馈数据指针
    float *other_speed_feedback_ptr; // 其他速度反馈数据指针,单位为度/秒
    Pid_init_t current_pid;     //电流PID初始化
    Pid_init_t angle_pid;       //角度PID初始化
    Pid_init_t speed_pid;       //速度PID初始化
}Djimotor_controller_init_t;
#pragma pack()

/**DJI电机实例**/
#pragma pack(1)
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    Djimotor_measure_t motor_measure;   //电机自身运动信息
    Djimotor_controller_t   motor_pid;  //电机自身的PID控制器
    Can_controller_t *can_controller;   //电机自身的CAN管理者
    int16_t deadzone_compensation;      // 电机死区补偿值
}Djimotor_device_t;
#pragma pack()

/**电机初始化结构体**/
#pragma pack(1)
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    int16_t deadzone_compensation;      // 电机死区补偿值
    Djimotor_controller_init_t motor_controller_init;
    Can_init_t can_init;
}Djimotor_init_config_t;
#pragma pack()

// 电机初始化函数
Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config);

//电机控制函数，设定PID目标值，单位可以是速度，也可以是角度，电流
void Djimotor_set_target(Djimotor_device_t *motor,float target);

//管理所有电机的控制命令发送，发送PID目标值到缓冲区，然后通过CAN发送出去
void Djimotor_control_all(void);

// 获取电机状态
Djimotor_status_e Djimotor_get_status(Djimotor_device_t *motor) ;

// 获取电机测量数据
Djimotor_measure_t Djimotor_get_measure(Djimotor_device_t *motor);

//设置电机状态
void Djimotor_set_status(Djimotor_device_t *motor,Djimotor_status_e status);

// 场景管理函数
// 切换电机控制场景
void Djimotor_switch_scene(Djimotor_device_t *motor, chassis_mode_e scene);

// 更新场景PID配置
void Djimotor_update_scene_config(Djimotor_device_t *motor, chassis_mode_e scene, Djimotor_scene_config_t *config);

// 获取当前场景
chassis_mode_e Djimotor_get_current_scene(Djimotor_device_t *motor);

// 设置电机死区补偿值
void Djimotor_set_deadzone(Djimotor_device_t *motor, int16_t deadzone);

// 获取电机死区补偿值
int16_t Djimotor_get_deadzone(Djimotor_device_t *motor);

#endif //_DJI_MOTOR_H