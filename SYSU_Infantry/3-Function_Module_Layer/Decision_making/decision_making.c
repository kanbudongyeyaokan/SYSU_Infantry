/**
 * @file    decision_making.c
 * @brief   决策任务
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    决策控制模式，控制量，并传送到对应的任务
 */
#include "decision_making.h"
//通信
#include "message_center.h"
//状态机获取
#include "robot_definitions.h"
//遥控器控制、
#include <stdio.h>

#include "remote_control.h"

#include "main.h"
/**********************发出决策信息***************************/
//存储遥控器数据，CURRENT-当前数据,LAST-上一次数据
static RC_ctrl_t *rc_data;

//底盘控制模式/控制量发布
static Publisher_t *chassis_cmd_pub;        //底盘控制信息发布者
static Chassis_cmd_send_t chassis_cmd_send;//存储决策层给底盘应用层的控制信息

//云台控制模式/控制量发布
static Publisher_t *gimbal_cmd_pub;          //云台控制信息发布者
static Gimbal_cmd_send_t  gimbal_cmd_send;  //存储决策层给云台应用层的控制信息

//发射机构控制模式/控制量发布
static Publisher_t *shoot_cmd_pub;           //发射机构控制信息发布者
static Shoot_cmd_send_t   shoot_cmd_send;   //存储决策层给发射机构应用层的控制信息

//机器人整体工作状态（二元）：----ON：在线 OFF：离线
static Robot_status_e robot_state = ROBOT_OFF;

/************************************************************/

/**********************接收反馈信息***************************/
//底盘反馈数据读取
static Subscriber_t *chassis_feedback_sub;               //底盘反馈信息订阅者
static Chassis_feedback_info_t chassis_feedback_recv;   //存储底盘应用层发给决策层的信息

//云台反馈数据读取
static Subscriber_t *gimbal_feedback_sub;                //云台反馈信息订阅者
static Gimbal_feedback_info_t  gimbal_feedback_recv;    //存储云台应用层发给决策层的信息

//发射机构反馈数据读取
static Subscriber_t *shoot_feedback_sub;                 //发射反馈信息订阅者
static Shoot_feedback_info_t   shoot_feedback_recv;     //存储发射应用层发给决策层的信息
/************************************************************/

/**
 * @brief 任务初始化函数，初始化决策层的发布者和订阅者,获取遥控器数据
 *
*/
void Decision_making_task_init()
{
    //接收遥控器数据
    rc_data = RC_Data_Get(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    /***********************************初始化决策层的发布者和订阅者***************************************/
    //底盘
    chassis_cmd_pub = Pub_register("chassis_cmd",sizeof(Chassis_cmd_send_t));
    chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));//底盘反馈数据订阅者
    //云台
    gimbal_cmd_pub = Pub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));//云台注册的话题是gimbal_cmd
    gimbal_feedback_sub = Sub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));
    //发射机构
    shoot_cmd_pub = Pub_register("shoot_cmd", sizeof(Shoot_cmd_send_t));
    shoot_feedback_sub = Sub_register("shoot_feedback", sizeof(Shoot_feedback_info_t));

    //机器人开始工作
    robot_state = ROBOT_ON;

}

void Receive_feedback_infomation()
{

    //获取底盘反馈信息
    Sub_get_message(chassis_feedback_sub,(void *)(&chassis_feedback_recv));
    //获取云台反馈信息
    Sub_get_message(gimbal_feedback_sub,(void *)(&gimbal_feedback_recv));
    //获取发射机构反馈信息
    Sub_get_message(shoot_feedback_sub,(void *)(&shoot_feedback_recv));
}

void Send_command_to_all_task()
{
    //发送底盘控制信息
    Pub_push_message(chassis_cmd_pub,(void *)(&chassis_cmd_send));
    //发送云台控制信息
    Pub_push_message(gimbal_cmd_pub,(void *)(&gimbal_cmd_send));
    //发送发射机构控制信息
    Pub_push_message(shoot_cmd_pub,(void *)(&shoot_cmd_send));
}

/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command()
{
   // printf("rc_data[CURRENT].rc.Rrocker_x:%d\r\n",rc_data[CURRENT].rc.Rrocker_x);
   // printf("rc_data[CURRENT].rc.Rrocker_y:%d\r\n",rc_data[CURRENT].rc.Rrocker_y);
   // printf("rc_data[CURRENT].rc.Lrocker_x:%d\r\n",rc_data[CURRENT].rc.Lrocker_x);
   // printf("rc_data[CURRENT].rc.Lrocker_y:%d\r\n",rc_data[CURRENT].rc.Lrocker_y);
   //  printf("rc_data[CURRENT].rc.dial:%d\r\n",rc_data[CURRENT].rc.dial);
   // printf("rc_data[CURRENT].rc.Lswitch:%d\r\n",rc_data[CURRENT].rc.Lswitch);
   // printf("rc_data[CURRENT].rc.Rswitch:%d\r\n",rc_data[CURRENT].rc.Rswitch);
    //左边开关打下，进入遥控器控制模式
    if (rc_data[CURRENT].rc.Lswitch == SWITCH_IS_DOWN)
    {
        RC_ctrl_set();
    }
    //左边开关打上，进入键盘控制模式
    else if (rc_data[CURRENT].rc.Lswitch == SWITCH_IS_UP)
    {
        Keyboard_ctrl_set();
    }
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
*/
void RC_ctrl_set()
{
    /**根据遥控器开关状态设定模式**/
   // printf("RC_ctrl_set \n");
    /**底盘/云台模式设定**/
    //如果右边开关打下，则进入底盘跟随云台模式,云台进入陀螺仪反馈模式
    if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_DOWN)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打中间，则底盘进入小陀螺模式，云台进入自由模式
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_UP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮,进入准备射击模式
    if (rc_data[CURRENT].rc.dial < -100)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    //正常情况下不打开摩擦轮
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    //左边拨轮往上打到底，开始发射子弹
    if (rc_data[CURRENT].rc.dial < -400)
    {
        shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;//连发
    }
    //正常情况下不发射子弹
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;     //
    }
    //急停模式
    Emergency_stop();
    /****************控制量设定*****************/
    //底盘控制量
     /*后续可增加死区限制，解决遥控器通道值因老化而造成的零漂问题*/
    if (rc_data[CURRENT].rc.Lrocker_y>=-32&&rc_data[CURRENT].rc.Lrocker_y<=0)
        chassis_cmd_send.vy=0;
    else {
        chassis_cmd_send.vy = 2.0f * (float)rc_data[CURRENT].rc.Lrocker_y; //竖直方向
    }
    chassis_cmd_send.vx = 2.0f * (float)rc_data[CURRENT].rc.Lrocker_x; //水平方向

    //云台控制量
    gimbal_cmd_send.yaw += 0.008f * (float)rc_data[CURRENT].rc.Rrocker_x;
    gimbal_cmd_send.pitch += 0.01f * (float)(rc_data[CURRENT].rc.Rrocker_y);
    
    //发射机构控制量
    // 射频控制,固定每秒1发
    shoot_cmd_send.shoot_rate = 10; 

}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
void Keyboard_ctrl_set()
{
    // 键盘控制设置的临时实现
    // 默认模式设置
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    // 这里添加键盘鼠标的具体控制逻辑

}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线等
 *
 */
void Emergency_stop()
{
    // 拨轮的向下打到底则进入急停模式
    if (rc_data[CURRENT].rc.dial < -300 || robot_state == ROBOT_OFF)
    {
        robot_state = ROBOT_OFF;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_UP)
    {
        robot_state = ROBOT_ON;
    }
}

/**
 * @brief 根据gimbal传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360
 *
 */
void Calc_offset_angle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_feedback_recv.yaw_motor_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}
