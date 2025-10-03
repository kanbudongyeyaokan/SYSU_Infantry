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
#include <stdio.h>

#include "message_center.h"
#include "math_lib.h"
#include "main.h"
#include "arm_math.h"

#define CHASSIS_FOLLOW_YAW_GAIN 0.5f
#define CHASSIS_FOLLOW_WZ_LIMIT 200.0f
#define CHASSIS_ROTATE_WZ 400.0f
#define CHASSIS_MOTOR_PID_MAX_OUT 15000.0f

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
    chassis_params.wheel_radius = 60.0f;   // 轮子半径60mm
    chassis_params.wheel_perimeter = chassis_params.wheel_radius*2*M_PI;//轮子周长
    chassis_params.chassis_radius = 0.2f;   // 默认底盘半径200mm
    chassis_params.wheel_base = 295.0f;       // 默认轮距295mm
    chassis_params.half_wheel_base = chassis_params.wheel_base/2.0;
    chassis_params.track_width = 295.0f;      // 默认轮宽295mm
    chassis_params.half_track_width = chassis_params.track_width/2.0;
    chassis_params.chassis_type=CHASSIS_TYPE_OMNI;// 全向轮底盘
    //设置底盘电机参数
    Djimotor_init_config_t cfg[4] = {
        {
            .motor_name = "CHASSIS_FR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 10,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = CHASSIS_MOTOR_PID_MAX_OUT,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
        },{
            .motor_name = "CHASSIS_FL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 10,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = CHASSIS_MOTOR_PID_MAX_OUT,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
        },{
            .motor_name = "CHASSIS_BL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 10,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = CHASSIS_MOTOR_PID_MAX_OUT,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
        },{
            .motor_name = "CHASSIS_BR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 10,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = CHASSIS_MOTOR_PID_MAX_OUT,
                }
            },
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
   // printf("speed is:%f",chassis_motors[0]->motor_measure.angular_velocity);
    // 从消息中心获取最新的底盘控制指令
    if (Sub_get_message(chassis_cmd_sub, &chassis_cmd_recv)) {
        //底盘四个电机的输出
        static Chassis_output_t chassis_output;
        switch (chassis_cmd_recv.chassis_mode)
        {
            /* 底盘无力 */
            case CHASSIS_ZERO_FORCE:
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_status(chassis_motors[i], MOTOR_STOP);
                    Djimotor_set_target(chassis_motors[i], 0);
                }
                break;
            /* 底盘不跟随云台 */
            case CHASSIS_NO_FOLLOW:
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
                }
                chassis_cmd_recv.wz = 0; //不旋转
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
            {
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
                }
                 // @TODO，不知道为什么云台相对底盘朝向差
                float angle_error = chassis_cmd_recv.offset_angle + 90.0f; // 目标与当前夹角误差，+90是因为底盘前方为云台右侧
                
                // 这里的符号是 +，否则会进入正反馈
                float wz_cmd = CHASSIS_FOLLOW_YAW_GAIN * angle_error * fabsf(angle_error);
                chassis_cmd_recv.wz = clamp_float(wz_cmd, -CHASSIS_FOLLOW_WZ_LIMIT, CHASSIS_FOLLOW_WZ_LIMIT);
                
                // 添加死区处理，避免小角度时的震荡
                if (fabsf(angle_error) < 2.0f) {
                    chassis_cmd_recv.wz = 0;
                }

                float cos_theta = arm_cos_f32(angle_error * MATH_DEG2RAD);
                float sin_theta = arm_sin_f32(angle_error * MATH_DEG2RAD);

                Chassis_cmd_send_t follow_cmd = chassis_cmd_recv;
                follow_cmd.vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
                follow_cmd.vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

                Chassis_kinematics_solve(&follow_cmd, &chassis_output);
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                }
                break;
            }
            /* 底盘小陀螺 */
            case CHASSIS_ROTATE:
            {
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
                }
                chassis_cmd_recv.wz = CHASSIS_ROTATE_WZ;   //设置小陀螺转速
                // @TODO，不知道为什么云台相对底盘朝向差和这里的指令杆量
                float angle_error = chassis_cmd_recv.offset_angle + 90.0f; // 目标与当前夹角误差，+90是因为底盘前方为云台右侧

                // 直接将云台坐标系下的杆量转换到底盘坐标系
                float cos_theta = arm_cos_f32(angle_error * MATH_DEG2RAD);
                float sin_theta = arm_sin_f32(angle_error * MATH_DEG2RAD);

                Chassis_cmd_send_t rotate_cmd = chassis_cmd_recv; // 复制一份指令
                rotate_cmd.vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
                rotate_cmd.vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

                Chassis_kinematics_solve(&rotate_cmd, &chassis_output);
                // 直接将目标写入各底盘电机实例
                for (uint8_t i = 0; i < 4; i++) {
                    Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                }

                break;
            }
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
    //目前以电池所在位置为后方，其对面为正前方
    // 计算各轮子线速度 (rad/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] =  -cmd->vx - cmd->vy - cmd->wz*(chassis_params.half_wheel_base+chassis_params.half_track_width)*MATH_DEG2RAD;   // 左前轮
    wheel_linear_speed[1] =  cmd->vx + cmd->vy - cmd->wz*(chassis_params.half_wheel_base+chassis_params.half_track_width)*MATH_DEG2RAD;   // 右前轮
    wheel_linear_speed[2] = -cmd->vx + cmd->vy - cmd->wz*(chassis_params.half_wheel_base+chassis_params.half_track_width)*MATH_DEG2RAD;  // 左后轮
    wheel_linear_speed[3] =  cmd->vx - cmd->vy - cmd->wz*(chassis_params.half_wheel_base+chassis_params.half_track_width)*MATH_DEG2RAD;  // 右后轮
    for (int i = 0; i < 4; i++) {
        output->motor_speed[i] = wheel_linear_speed[i];
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



