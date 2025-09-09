#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "bsp_can.h"
#include "algorithm_pid.h"
#include "string.h"

#define MAX_MOTOR_COUNT 16

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制


/**DJI电机类型**/
typedef enum
{
    M3508 = 0,
    M2006,
    G3508
}Djimotor_type_e;

/**DJI电机控制状态**/
typedef enum
{
    MOTOR_ENBALED = 0,
    MOTOR_STOP
}Djimotor_status_e;

/**DJI电机控制类型**/
typedef enum {
    POS_MODE=0,     //位置模式
    SPEED_MODE,     //速度模式
    CURRENT_MODE    //电流模式
}Djimotor_mode_e;


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

/**DJI电机控制器结构体**/
#pragma pack(1)
typedef struct
{
    Djimotor_mode_e mode;       //电机模式
    Pid_instance_t current_pid; //电流环
    Pid_instance_t angle_pid;   //角度环
    Pid_instance_t speed_pid;   //速度环

    float pid_target;           //PID目标量

}Djimotor_controller_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    Djimotor_mode_e mode;       //电机模式
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
    Can_controller_t *can_controller;    //电机自身的CAN管理者
}Djimotor_device_t;
#pragma pack()

/**电机初始化结构体**/
#pragma pack(1)
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    Djimotor_controller_init_t motor_controller_init;
    Can_init_t can_init;
}Djimotor_init_config_t;
#pragma pack()

// 电机初始化函数
Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config);



#endif //_DJI_MOTOR_H