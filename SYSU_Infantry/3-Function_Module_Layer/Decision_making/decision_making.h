#ifndef SYSU_INFANTRY_DECISION_MAKING_H
#define SYSU_INFANTRY_DECISION_MAKING_H

#include "main.h"
#include "robot_definitions.h"
#include "ins.h"

/**************决策*****************/

/*机器人控制来源------键鼠/遥控器*/
#define RC_CTRL 0
#define KEYBOARD_CTRL 1

/******************决策任务发送给各个模块的信息**********************/
//底盘
typedef struct
{
     // 控制量
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    float gimbal_yaw_total_angle; // 云台多圈角度
    float gimbal_yaw_rate;        // 云台角速度
    chassis_mode_e chassis_mode;        //底盘控制模式
}Chassis_cmd_send_t;

//云台
typedef struct 
{
    float yaw;          // yaw控制量
    float pitch;        // pitch控制量
    float chassis_wz;   // 底盘角速度，用于云台陀螺仪模式下的补偿
    
    gimbal_mode_e gimbal_mode;          //云台控制模式
}Gimbal_cmd_send_t;


//发射机构
typedef struct 
{
    shoot_mode_e shoot_mode;    //发射模式-若打开则摩擦轮也跟着打开
    loader_mode_e loader_mode;  //子弹发射模式
    uint8_t shoot_rate;         //发射弹频 (发/秒)
}Shoot_cmd_send_t;

/******************决策任务接收各个模块的反馈信息**********************/
//底盘
typedef struct 
{
    //接收底盘陀螺仪反馈数据
    float chassis_wz;   //底盘角速度，可以由底盘陀螺仪反馈回来
}Chassis_feedback_info_t;
//云台
typedef struct 
{
    //接收云台陀螺仪反馈数据

    //YAW轴电机的单圈角度---由编码器得来
    float yaw_motor_single_round_angle;
    //YAW轴电机的多圈角度---由多圈解算得来
    float yaw_motor_total_angle;
    //IMU输出的Yaw多圈角度（度）
    float imu_yaw_total_angle;
    //IMU输出的Yaw角速度（度/秒）
    float imu_yaw_rate;
    //IMU状态
    // Imu_state_e imu_state;
}Gimbal_feedback_info_t;
//发射机构
typedef struct 
{
    uint8_t gun_rest_heat;      // 剩余枪管热量,在裁判系统中读取？

}Shoot_feedback_info_t;



/**
 * @brief 任务初始化函数
 *        1. 初始化决策层的发布者和订阅者 (uORB 或其他消息机制)
 *        2. 获取遥控器数据指针，准备读取遥控器输入
 */
void Decision_making_task_init();

/**
 * @brief 接收各个应用层反馈回来的数据
 *        包括底盘电机状态、云台角度、发射机构热量等，用于决策逻辑判断
 */
void Receive_feedback_infomation();

/**
 * @brief 发送决策层的控制信息给各个应用层
 *        将计算好的 Chassis_cmd, Gimbal_cmd, Shoot_cmd 发送出去
 */
void Send_command_to_all_task();

/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command();

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置,不做发送
 *
*/
void RC_command_set();

void RC_ctrl_set();

void Keyboard_ctrl_set();


void Emergency_stop();
/**
 * @brief 控制输入为键鼠的模式和控制量设置，不做发送
 *
*/
static void Keyboard_command_set();

/**
 * @brief 根据gimbal传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360
 *
 */
void Calc_offset_angle();
#endif //SYSU_INFANTRY_DECISION_MAKING_H