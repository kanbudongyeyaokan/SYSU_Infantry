#ifndef _ROBOT_DEFINITIONS_H
#define _ROBOT_DEFINITIONS_H

//该层只放机器人的物理参数定义，以及各个模块状态的描述

/**************************Robot_physical parameters*******************************/



/******************************Robot_state_machine*********************************/
/********机器人ROBOT************/
typedef enum
{
    ROBOT_ON = 0,//机器人开启
    ROBOT_OFF    //机器人离线，一般用于急停模式的处理
}Robot_status_e;

/********云台GIMBAL************/
// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_VISION_MODE,    //云台视觉模式，根据视觉给的控制量移动
} gimbal_mode_e;

/********底盘CHASSIS************/
//底盘类型设置
typedef enum
{
    CHASSIS_TYPE_OMNI = 0,     // 全向轮底盘
    CHASSIS_TYPE_MECANUM,      // 麦克纳姆轮底盘
    CHASSIS_TYPE_STEERING      // 舵轮底盘
} chassis_type_e;

//底盘运动模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL, // 跟随模式，底盘叠加角度环控制
    CHASSIS_ROTATE,            // 匀速小陀螺模式
} chassis_mode_e;

/********发射机构SHOOT**********/
// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,      //停止射击，此时摩擦轮不会转动
    SHOOT_ON,           //开始设计，此时摩擦轮会转动
} shoot_mode_e;
//拨弹盘模式
typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;



#endif /* _ROBOT_DEFINITIONS_H */