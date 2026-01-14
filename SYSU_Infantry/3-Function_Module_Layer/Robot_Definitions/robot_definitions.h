#ifndef _ROBOT_DEFINITIONS_H
#define _ROBOT_DEFINITIONS_H

//该层只放机器人的物理参数定义，以及各个模块状态的描述

/**************************Robot_physical parameters*******************************/

#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
          
// 云台参数
// #define YAW_CHASSIS_ALIGN_ECD 900  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_CHASSIS_ALIGN_ECD 2094  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 760      // 云台处于水平位置时编码器值（760）,若对云台有机械改动需要修改

//云台编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360


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