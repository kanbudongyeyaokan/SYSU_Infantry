#include "decision_making.h"
//通信
#include "message_center.h"
//状态机获取
#include "robot_definitions.h"
//遥控器控制、
#include "remote_control.h"

/**********************发出决策信息***************************/
//存储遥控器数据，CURRENT-当前数据,LAST-上一次数据
static RC_ctrl_t *rc_data;

//底盘控制模式/控制量发布
static Publisher_t chassis_cmd_pub;        //底盘控制信息发布者
static Chassis_cmd_send_t chassis_cmd_send;//存储决策层给底盘应用层的控制信息

//云台控制模式/控制量发布
static Publisher_t gimbal_cmd_pub;          //云台控制信息发布者
static Gimbal_cmd_send_t  gimbal_cmd_send;  //存储决策层给云台应用层的控制信息

//发射机构控制模式/控制量发布
static Publisher_t shoot_cmd_pub;           //发射机构控制信息发布者
static Shoot_cmd_send_t   shoot_cmd_send;   //存储决策层给发射机构应用层的控制信息
/************************************************************/

/**********************接收反馈信息***************************/
//底盘反馈数据读取
static Subscriber_t chassis_feedback_sub;               //底盘反馈信息订阅者
static Chassis_feedback_info_t chassis_feedback_recv;   //存储底盘应用层发给决策层的信息

//云台反馈数据读取
static Subscriber_t gimbal_feedback_sub;                //云台反馈信息订阅者
static Gimbal_feedback_info_t  gimbal_feedback_recv;    //存储云台应用层发给决策层的信息

//发射机构反馈数据读取
static Subscriber_t shoot_feedback_sub;                 //发射反馈信息订阅者
static Shoot_feedback_info_t   shoot_feedback_recv;     //存储发射应用层发给决策层的信息
/************************************************************/

/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command()
{
    //左边开关打下，进入遥控器控制模式
    if (rc_data[CURRNET].rc.Lswitch == SWITCH_IS_DOWN)
    {
        RC_ctrl_set();
    }
    //左边开关打上，进入键盘控制模式
    else if (rc_data[CURRNET].rc.Lswitch == SWITCH_IS_UP)
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

    /**底盘/云台模式设定**/
    //如果右边开关打下，则进入底盘跟随云台模式,云台进入陀螺仪反馈模式
    if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_DOWN)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打中间，则底盘进入小陀螺模式，云台进入自由模式
    else if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_MID)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_UP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮,进入准备射击模式
    if (rc_data[CURRNET].rc.dial < -100)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    //正常情况下不打开摩擦轮
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    //左边拨轮往上打到底，开始发射子弹
    if (rc_data[CURRNET].rc.dial < -500)
    {
        shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;//连发
    }
    //正常情况下不发射子弹
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;     //
    }

    /****************控制量设定*****************/
    //底盘控制量




    //云台控制量

}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
static void Keyboard_ctrl_set()
{
    // 键盘控制设置的临时实现
    // 默认模式设置
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    
    // 这里添加键盘鼠标的具体控制逻辑
    // ...
}

