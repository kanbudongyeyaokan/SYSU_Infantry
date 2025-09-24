/**
 * @file    chassis.c
 * @brief   底盘功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    实现底盘运动学解算功能，支持多种底盘类型
 */

#include "chassis.h"
#include "bsp_can.h"
#include <math.h>
#include "message_center.h"
#include "math_lib.h"

/****************接收决策层的底盘控制信息********************/
// 订阅决策层发来的底盘控制指令
static Subscriber_t *chassis_cmd_sub;
// 存储决策层发来的控制命令
static Chassis_cmd_send_t chassis_cmd_recv;

/****************发送给决策层的底盘反馈信息******************/
// 发布给决策层的底盘反馈信息
static Publisher_t *chassis_feedback_pub;
// 存储发送给决策层的反馈信息
static Chassis_feedback_info_t chassis_feedback;

/****************底盘参数存储***************************** */ 
static Chassis_params_t chassis_params = {0};

/****************底盘电机实例*******************************/
static Djimotor_device_t *chassis_motors[4] = {0};


/*********************************底盘方法接口**************************************/
/**
 * @brief 底盘任务初始化
 */
void Chassis_task_init(void)
{
   //底盘模块初始化
    Chassis_init();
    // 先完成消息中心注册，避免后续大量内存分配导致订阅失败
    chassis_cmd_sub = Sub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
    chassis_feedback_pub = Pub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));
   
}
/**
 * @brief 底盘功能模块初始化
 */
void Chassis_init()
{
    // 设置底盘物理参数
    chassis_params.wheel_radius = 0.076f;   // 默认轮子半径76mm
    chassis_params.chassis_radius = 0.2f;   // 默认底盘半径200mm
    chassis_params.wheel_base = 0.4f;       // 默认轮距400mm
    chassis_params.track_width = 0.3f;      // 默认轮宽300mm
    chassis_params.chassis_type=CHASSIS_TYPE_OMNI;// 全向轮底盘

    //设置底盘电机参数
    Djimotor_init_config_t cfg[4] = {
        {
            .motor_name = "CHASSIS_FR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
        },{
            .motor_name = "CHASSIS_FL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
        },{
            .motor_name = "CHASSIS_BL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
        },{
            .motor_name = "CHASSIS_BR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 4, .rx_id = 0x204}
        }
    };

    for (int i = 0; i < 4; i++) {
        chassis_motors[i] = DJI_Motor_Init(&cfg[i]);
    }
}

/**
 * @brief 处理底盘控制指令
 */
void Chassis_handle_command(void)
{
    // 从消息中心获取最新的底盘控制指令
    if (Sub_get_message(chassis_cmd_sub, &chassis_cmd_recv)) {
        //底盘四个电机的输出
        static Chassis_output_t chassis_output;
        switch (chassis_cmd_recv.chassis_mode)
        {
        /* 底盘无力 */
        case CHASSIS_ZERO_FORCE:
            for (uint8_t i = 0; i < 4; i++) {
               Djimotor_set_target(chassis_motors[i],0);
            }    
            break;
        /* 底盘不跟随云台 */
        case CHASSIS_NO_FOLLOW:
            //底盘解算
            Chassis_kinematics_solve(&chassis_cmd_recv, &chassis_output);
            // 直接将目标写入各底盘电机实例（这些实例应已在底盘/电机相关模块初始化）
            for (uint8_t i = 0; i < 4; i++) {
                // 速度模式：单位rpm
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
            }
            break;
        /* 底盘跟随云台 */
        case CHASSIS_FOLLOW_GIMBAL:

            break;
        /* 底盘小陀螺 */
        case CHASSIS_ROTATE:

            break;
        default:
            break;
        }

        // 更新底盘反馈信息（这里可以添加底盘角速度的反馈）
        // 简化处理，假设底盘角速度直接来自控制指令
        chassis_feedback.chassis_wz = chassis_cmd_recv.wz;
        Pub_push_message(chassis_feedback_pub, &chassis_feedback);
    }
}

/**
 * @brief 全向轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_omni_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 全向轮布局（从俯视图看）：
    //    1(前)
    //  2(左) 0(右)
    //    3(后)
    // 
    // 全向轮运动学模型（轮子方向为90度间隔）
    // 右轮(0) = +vy - wz*R  （贡献侧向移动和旋转）
    // 前轮(1) = +vx - wz*R  （贡献前向移动和旋转）
    // 左轮(2) = -vy - wz*R  （贡献侧向移动和旋转）
    // 后轮(3) = -vx - wz*R  （贡献前向移动和旋转）
    
    float rotate_compensation = cmd->wz * chassis_params.chassis_radius / chassis_params.wheel_radius;
    
    // 计算各轮子线速度 (m/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] = cmd->vy - rotate_compensation;   // 右轮
    wheel_linear_speed[1] = cmd->vx - rotate_compensation;   // 前轮
    wheel_linear_speed[2] = -cmd->vy - rotate_compensation;  // 左轮
    wheel_linear_speed[3] = -cmd->vx - rotate_compensation;  // 后轮
    
    // 转换为角速度 (rad/s) 再转换为 rpm
    for (int i = 0; i < 4; i++) {
        float angular_velocity = wheel_linear_speed[i] / chassis_params.wheel_radius; // rad/s
        output->motor_speed[i] = angular_velocity * 60.0f / (2.0f * M_PI); // rpm
    }
    
}

/**
 * @brief 麦克纳姆轮底盘运动学解算
 * @param cmd 底盘控制指令  
 * @param output 解算输出结果
 */
static void Chassis_mecanum_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 麦克纳姆轮布局（从俯视图看）：
    // 左前(0)  右前(1)
    // 左后(2)  右后(3)
    //
    // 麦克纳姆轮运动学模型
    // 左前轮 = vx - vy - wz*(wheelbase + track_width)/2
    // 右前轮 = vx + vy + wz*(wheelbase + track_width)/2  
    // 左后轮 = vx + vy - wz*(wheelbase + track_width)/2
    // 右后轮 = vx - vy + wz*(wheelbase + track_width)/2
    
    float L = chassis_params.wheel_base;      // 轮距
    float W = chassis_params.track_width;     // 轮宽
    float rotate_compensation = cmd->wz * (L + W) / (2.0f * chassis_params.wheel_radius);
    
    // 计算各轮子线速度 (m/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] = cmd->vx - cmd->vy - rotate_compensation; // 左前
    wheel_linear_speed[1] = cmd->vx + cmd->vy + rotate_compensation; // 右前
    wheel_linear_speed[2] = cmd->vx + cmd->vy - rotate_compensation; // 左后
    wheel_linear_speed[3] = cmd->vx - cmd->vy + rotate_compensation; // 右后
    
    // 转换为角速度 (rad/s) 再转换为 rpm
    for (int i = 0; i < 4; i++) {
        float angular_velocity = wheel_linear_speed[i] / chassis_params.wheel_radius; // rad/s
        output->motor_speed[i] = angular_velocity * 60.0f / (2.0f * M_PI); // rpm
    }
    
}



/**
 * @brief 底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
void Chassis_kinematics_solve(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 根据底盘类型调用对应的解算函数
    switch (chassis_params.chassis_type) {
        case CHASSIS_TYPE_OMNI:
            Chassis_omni_kinematics(cmd, output);
            break;
            
        case CHASSIS_TYPE_MECANUM:
            Chassis_mecanum_kinematics(cmd, output);
            break;
            
        default:
            break;
    }
}



