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
//遥控器控制
#include <stdio.h>
#include "remote_control.h"
#include "main.h"
#include <stdbool.h>
#include <math.h>



/**********************发出决策信息***************************/
//存储遥控器数据，CURRENT-当前数据,LAST-上一次数据
static RC_ctrl_t *rc_data;

//底盘控制模式/控制量发布
static Publisher_t *chassis_cmd_pub;        //底盘控制信息发布者
static Chassis_cmd_send_t chassis_cmd_send;//存储决策层给底盘应用层的控制信息

//云台控制模式/控制量发布
static Publisher_t *gimbal_cmd_pub;          //云台控制信息发布者
static Gimbal_cmd_send_t  gimbal_cmd_send;  //存储决策层给云台应用层的控制信息
#define MAX_GIMBAL_YAW_LEAD 10.0f
#define MAX_GIMBAL_PITCH_LEAD 20.0f // Pitch轴通常有机械限位，也可以限制一下

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
static bool gimbal_yaw_initialized = false;


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

    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;
    gimbal_yaw_initialized = false;
    chassis_cmd_send.gimbal_yaw_total_angle = 0.0f;
    chassis_cmd_send.gimbal_yaw_rate = 0.0f;

}

void Receive_feedback_infomation()
{

    //获取底盘反馈信息
    Sub_get_message(chassis_feedback_sub,(void *)(&chassis_feedback_recv));
    //获取云台反馈信息
    Sub_get_message(gimbal_feedback_sub,(void *)(&gimbal_feedback_recv));
    // 修复：使用IMU的多圈角度初始化云台目标值（与云台电机反馈源一致）
    // 同时确保IMU已就绪，避免使用无效数据
    if (!gimbal_yaw_initialized && gimbal_feedback_recv.imu_state == IMU_STATE_READY)
    {
        gimbal_cmd_send.yaw = gimbal_feedback_recv.imu_yaw_total_angle;
        gimbal_yaw_initialized = true;
    }
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
    //printf("rc_data[CURRENT].rc.Rrocker_x:%d\r\n",rc_data[CURRENT].rc.Rrocker_x);
    //printf("rc_data[CURRENT].rc.Rrocker_y:%d\r\n",rc_data[CURRENT].rc.Rrocker_y);
    //printf("rc_data[CURRENT].rc.Lrocker_x:%d\r\n",rc_data[CURRENT].rc.Lrocker_x);
    //printf("rc_data[CURRENT].rc.Lrocker_y:%d\r\n",rc_data[CURRENT].rc.Lrocker_y);
    // printf("rc_data[CURRENT].rc.dial:%d\r\n",rc_data[CURRENT].rc.dial);
    //printf("rc_data[CURRENT].rc.Lswitch:%d\r\n",rc_data[CURRENT].rc.Lswitch);
    //printf("rc_data[CURRENT].rc.Rswitch:%d\r\n",rc_data[CURRENT].rc.Rswitch);
     //左边开关打下，进入遥控器控制模式
   if (rc_data[CURRENT].rc.Lswitch == SWITCH_IS_DOWN)
   {
        RC_ctrl_set();
    }
    //左边开关打上，进入单发模式
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
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  
    }
    //如果右边开关打中间，则底盘进入小陀螺模式，云台进入自由模式
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_UP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮,进入准备射击模式
    if (rc_data[CURRENT].rc.dial > 150)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    //正常情况下不打开摩擦轮
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
     //左边拨轮往上打到底，开始发射子弹
 
     if (rc_data[CURRENT].rc.dial > 450)
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;//连发
            shoot_cmd_send.shoot_rate = 8;//每分钟80发
        }
        //正常情况下不发射子弹
        else
        {
            shoot_cmd_send.loader_mode = LOAD_STOP;     //
        }
    
    


    //正常情况下不发射子弹

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
    /**************** yaw 轴积分防止饱和 ****************/
    /*
    // 1. 获取输入增量
        // 注意：0.008f 是灵敏度系数，如果觉得太快或太慢可以调整这里
        float yaw_delta = 0.008f * (float)rc_data[CURRENT].rc.Rrocker_x;
    
        // 2. 更新虚拟目标（用户想去哪）
        gimbal_virtual_target_yaw += yaw_delta;
        
        // 3. 获取当前实际值
        float current_actual_yaw = gimbal_feedback_recv.imu_yaw_total_angle;
        
        // 4. 计算领跑误差
        float virtual_lead_error = gimbal_virtual_target_yaw - current_actual_yaw;
        
        // 5. 领跑限制逻辑
        // 定义“刹车缓冲距离”，建议设置得和 MAX_GIMBAL_YAW_LEAD 一样
        // 这样松手时，云台最多只需要追赶 60度
        float windup_limit = MAX_GIMBAL_YAW_LEAD; 
        
        if (virtual_lead_error > MAX_GIMBAL_YAW_LEAD) 
        {
            // 限制发给电机的指令：永远保持在前方 60 度，引导电机全速旋转
            gimbal_cmd_send.yaw = current_actual_yaw + MAX_GIMBAL_YAW_LEAD;
            
            // 关键修改：限制虚拟目标不要跑太远
            // 如果超出了 windup_limit (60度)，强行把它拉回来，防止积攒过多行程
            if (gimbal_virtual_target_yaw > current_actual_yaw + windup_limit) {
                gimbal_virtual_target_yaw = current_actual_yaw + windup_limit;
            }
        } 
        else if (virtual_lead_error < -MAX_GIMBAL_YAW_LEAD) 
        {
            // 反向同理
            gimbal_cmd_send.yaw = current_actual_yaw - MAX_GIMBAL_YAW_LEAD;
            
            if (gimbal_virtual_target_yaw < current_actual_yaw - windup_limit) {
                gimbal_virtual_target_yaw = current_actual_yaw - windup_limit;
            }
        } 
        else 
        {
            // 正常范围内，指哪打哪
            gimbal_cmd_send.yaw = gimbal_virtual_target_yaw;
        }
    */  
    // Pitch轴逻辑保持不变...
    gimbal_cmd_send.pitch += 0.01f * (float)(rc_data[CURRENT].rc.Rrocker_y);
    gimbal_cmd_send.yaw += 0.008f * (float)rc_data[CURRENT].rc.Rrocker_x;   

}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
void Keyboard_ctrl_set()
{
     if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_DOWN)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  
    }
    //如果右边开关打中间，则底盘进入小陀螺模式，云台进入自由模式
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_UP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
     /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮,进入准备射击模式
    if (rc_data[CURRENT].rc.dial > 150)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    //正常情况下不打开摩擦轮
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
//正常情况下不发射子弹
  
    if (rc_data[CURRENT].rc.dial > 300 ){
        shoot_cmd_send.loader_mode =  LOAD_1_BULLET;//单发
    }
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
    gimbal_cmd_send.yaw += 0.005f * (float)rc_data[CURRENT].rc.Rrocker_x;
    gimbal_cmd_send.pitch += 0.01f * (float)(rc_data[CURRENT].rc.Rrocker_y);

    // 键盘控制设置的临时实现
    // 默认模式设置
   // chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    //gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
   // shoot_cmd_send.shoot_mode = SHOOT_OFF;
   // shoot_cmd_send.loader_mode = LOAD_STOP;
    // 这里添加键盘鼠标的具体控制逻辑

    // float yaw_delta = (float)rc_data[TEMP].mouse.x / 660.0f * 10.0f;
    // gimbal_cmd_send.yaw += yaw_delta;
    // gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660.0f * 10.0f;

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
    // 遥控器右侧开关为[中],恢复正常运行
    if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
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
    // 如果机器人处于急停状态，不计算偏差角，直接清零
    if (robot_state == ROBOT_OFF) {
        chassis_cmd_send.offset_angle = 0.0f;
        chassis_cmd_send.gimbal_yaw_total_angle = 0.0f;
        chassis_cmd_send.gimbal_yaw_rate = 0.0f;
        return;
    }
    
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float last_offset_angle = 0.0f; // 记录上一次的偏差角
    angle = gimbal_feedback_recv.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    
    float temp_offset_angle;
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        temp_offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif

    // 检测角度跳变，如果跳变超过180度，说明发生了0/360度边界跨越
    float angle_diff = temp_offset_angle - last_offset_angle;
    if (angle_diff > 180.0f) {
        temp_offset_angle -= 360.0f;
    } else if (angle_diff < -180.0f) {
        temp_offset_angle += 360.0f;
    }
    
    // 限制偏差角在-180到180度之间
    if (temp_offset_angle > 180.0f) {
        temp_offset_angle -= 360.0f;
    } else if (temp_offset_angle < -180.0f) {
        temp_offset_angle += 360.0f;
    }
    
    chassis_cmd_send.offset_angle = temp_offset_angle;
    last_offset_angle = temp_offset_angle;

    chassis_cmd_send.gimbal_yaw_total_angle = gimbal_feedback_recv.imu_yaw_total_angle;
    chassis_cmd_send.gimbal_yaw_rate = gimbal_feedback_recv.imu_yaw_rate;
}
