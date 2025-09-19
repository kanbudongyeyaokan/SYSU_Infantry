/**
 * @file    Chassis_task.c
 * @brief   底盘控制任务源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    负责底盘运动学解算和控制量传递
 */

#include "Chassis_task.h"
#include "chassis.h"
#include "motor_chassis_interface.h"
#include "message_center.h"
#include "decision_making.h"
#include "dji_motor.h"
#include <stdio.h>

// 订阅决策层发来的底盘控制指令
static Subscriber_t *chassis_cmd_sub;

// 发布给决策层的底盘反馈信息
static Publisher_t *chassis_feedback_pub;

// 存储决策层发来的控制命令
static Chassis_cmd_send_t chassis_cmd;

// 存储发送给决策层的反馈信息
static Chassis_feedback_info_t chassis_feedback;

/**
 * @brief 底盘任务初始化
 */
static void Chassis_task_init(void)
{
    // 初始化底盘功能模块
    Chassis_params_t chassis_params = {
        .wheel_radius = 0.076f,         // 轮子半径76mm
        .chassis_radius = 0.2f,         // 底盘半径200mm  
        .wheel_base = 0.4f,             // 轮距400mm
        .track_width = 0.3f,            // 轮宽300mm
        .chassis_type = CHASSIS_TYPE_OMNI // 全向轮底盘
    };
    Chassis_init(&chassis_params);
    
    // 初始化Motor-Chassis接口
    Motor_chassis_interface_init();
    
    // 订阅决策层发来的底盘控制指令
    chassis_cmd_sub = Sub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
    
    // 注册底盘反馈信息发布者
    chassis_feedback_pub = Pub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));
}


/**
 * @brief 处理底盘控制指令
 */
static void Chassis_handle_command(void)
{
    // 从消息中心获取最新的底盘控制指令
    if (Sub_get_message(chassis_cmd_sub, &chassis_cmd)) {
        
        // 设置电机配置（只传递底盘模式）
        Chassis_set_motor_config(chassis_cmd.chassis_mode);
        
        // 进行底盘运动学解算
        Chassis_output_t chassis_output;
        Chassis_kinematics_solve(&chassis_cmd, &chassis_output);
        
        // 设置电机速度目标值
        Chassis_set_motor_speeds(chassis_output.motor_speed);
        
        // 更新底盘反馈信息（这里可以添加底盘角速度的反馈）
        // 简化处理，假设底盘角速度直接来自控制指令
        chassis_feedback.chassis_wz = chassis_cmd.wz;
        Pub_push_message(chassis_feedback_pub, &chassis_feedback);
    }
}

/**
 * @brief 底盘控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Chassis_control_task(void const *argument)
{
    // 初始化底盘任务
    Chassis_task_init();
    
    // 任务主循环
    for (;;)
    {
        // 处理底盘控制指令
        Chassis_handle_command();
        
        // 任务延时2ms，保持500Hz的运行频率
        osDelay(2);
    }
}