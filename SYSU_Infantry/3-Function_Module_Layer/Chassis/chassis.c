/**
 * @file    chassis.c
 * @brief   底盘功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
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
#include "bsp_usart.h"
#include "robot_task.h"
#include "chassis_power_control.h"
#include "supercap_comm.h"


#define CHASSIS_FOLLOW_YAW_GAIN 0.5f
#define CHASSIS_FOLLOW_WZ_LIMIT 200.0f
#define CHASSIS_ROTATE_WZ 500.0f
#define CHASSIS_MOTOR_PID_MAX_OUT 15000.0f

/****************发送给决策层的底盘反馈信息******************/
// 发布给决策层的底盘反馈信息
// static Publisher_t *chassis_feedback_pub;
// 存储发送给决策层的反馈信息
static Chassis_feedback_info_t chassis_feedback;

/****************底盘参数存储***************************** */
static Chassis_params_t chassis_params = {0};

/****************底盘电机实例及控制参数*******************************/
static Djimotor_device_t *chassis_motors[4] = {0};

// 底盘四个电机的输出
static Chassis_output_t chassis_output;

extern QueueHandle_t Chassis_feedback_queue_handle; // 声明外部底盘反馈队列句柄

Chassis_cmd_send_t test_cmd;

#define abs(x) ((x > 0) ? x : -x)

/*********************************底盘方法接口**************************************/
/**
 * @brief 底盘任务初始化
 */
void Chassis_task_init(void) {
    //底盘模块初始化
    Chassis_init();
    //超电初始化
    // SuperCap_Comm_Init(&hcan2);

    //底盘功率控制初始化
    Chassis_Power_Control_Init();
}

/**
 * @brief 底盘功能模块初始化
 */
void Chassis_init() {
    // 设置底盘物理参数
    chassis_params.wheel_radius = 60.0f; // 轮子半径60mm
    chassis_params.wheel_perimeter = chassis_params.wheel_radius * 2 * M_PI; //轮子周长
    chassis_params.chassis_radius = 0.2f; // 默认底盘半径200mm
    chassis_params.wheel_base = 295.0f; // 默认轮距295mm
    chassis_params.half_wheel_base = chassis_params.wheel_base / 2.0;
    chassis_params.track_width = 295.0f; // 默认轮宽295mm
    chassis_params.half_track_width = chassis_params.track_width / 2.0;
    chassis_params.chassis_type = CHASSIS_TYPE_OMNI; // 全向轮底盘

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
                    .kp = 15,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .LPF_coefficient = 0.8,
                    .feedfoward_coefficient = 0.2,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST |
                                    PID_FEEDFOWARD,
                    .max_out = 15000,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
        },
        {
            .motor_name = "CHASSIS_FL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 15,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .LPF_coefficient = 0.8,
                    .feedfoward_coefficient = 0.2,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST |
                                    PID_FEEDFOWARD,
                    .max_out = 15000,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
        },
        {
            .motor_name = "CHASSIS_BL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 15,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .LPF_coefficient = 0.8,
                    .feedfoward_coefficient = 0.2,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST |
                                    PID_FEEDFOWARD,
                    .max_out = 15000,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
        },
        {
            .motor_name = "CHASSIS_BR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 15,
                    .ki = 0,
                    .kd = 0,
                    .max_iout = 3000,
                    .LPF_coefficient = 0.8,
                    .feedfoward_coefficient = 0.2,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST |
                                    PID_FEEDFOWARD,
                    .max_out = 15000,
                }
            },
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 4, .rx_id = 0x204}
        }
    };

    for (int i = 0; i < 4; i++) {
        chassis_motors[i] = DJI_Motor_Init(&cfg[i]);
    }
    // chassis_motors[2] = DJI_Motor_Init(&cfg[2]);
    // chassis_motors[3] = DJI_Motor_Init(&cfg[3]);
}

/**
 * @brief 底盘逻辑更新函数
 * @param cmd 指向接收到的指令结构体
 */
void Chassis_Update_Control(const Chassis_cmd_send_t *cmd)
{
    //testing
    //printf("vx: %f,vy:%f\r\n", cmd->vx,cmd->vy);
    //  Uart_printf(test_uart,"vx: %f,vy:%f,mode %d\r\n", cmd->vx,cmd->vy,cmd->chassis_mode);
    // Uart_printf(test_uart,"offset_angle: %.2f\r\n", cmd->offset_angle);
    // printf("chassis_mode: %d\r\n", cmd->chassis_mode);
   test_cmd = *cmd;

    // 1. 使用传入的 'cmd' 指针代替原来的全局变量
    switch (cmd->chassis_mode) // [注意] 这里把 . 改成了 ->
    {
        case CHASSIS_ZERO_FORCE:
            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_status(chassis_motors[i], MOTOR_STOP);
                Djimotor_set_target(chassis_motors[i], 0);
                //计算PID
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;

        case CHASSIS_NO_FOLLOW:
            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
            }

            Chassis_kinematics_solve(cmd, &chassis_output);
            // Chassis_Power_Control(&chassis_output, chassis_motors);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                //计算PID
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;

        case CHASSIS_FOLLOW_GIMBAL:
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
            }

            Chassis_cmd_send_t cmd_solved = *cmd;

            //限制平方项输出
            // float omega_z = 0.1f * cmd->offset_angle * abs(cmd->offset_angle);
            // if (omega_z < -CHASSIS_FOLLOW_WZ_LIMIT) {
            //     omega_z = -CHASSIS_FOLLOW_WZ_LIMIT;
            // } else if (omega_z > CHASSIS_FOLLOW_WZ_LIMIT) {
            //     omega_z = CHASSIS_FOLLOW_WZ_LIMIT;
            // }
            // cmd_solved.wz = omega_z;

            cmd_solved.wz = 0.5 * cmd->offset_angle * abs(cmd->offset_angle); 

            // 矢量变换逻辑
            float theta = -cmd->offset_angle * (M_PI / 180.0f);
            float cos_theta = arm_cos_f32(theta);
            float sin_theta = arm_sin_f32(theta);

            cmd_solved.vx = cmd->vx * cos_theta - cmd->vy * sin_theta;
            cmd_solved.vy = cmd->vx * sin_theta + cmd->vy * cos_theta;

            Chassis_kinematics_solve(&cmd_solved, &chassis_output);
            //Chassis_Power_Control(&chassis_output, chassis_motors);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                //计算PID
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;
        }

        case CHASSIS_ROTATE:
            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
            }

            Chassis_cmd_send_t rotate_cmd = *cmd;

            // 修改副本的 Wz
            rotate_cmd.wz = CHASSIS_ROTATE_WZ;

            float angle_error = cmd->offset_angle;
            float cos_theta = arm_cos_f32(angle_error * MATH_DEG2RAD);
            float sin_theta = arm_sin_f32(angle_error * MATH_DEG2RAD);

            // [修正] 使用原始数据 cmd 计算，赋值给副本 rotate_cmd
            // 这样保证计算 vy 时，vx 还是原始值
            rotate_cmd.vx = cmd->vx * cos_theta - cmd->vy * sin_theta;
            rotate_cmd.vy = cmd->vx * sin_theta + cmd->vy * cos_theta;

            // 传入副本的地址
            Chassis_kinematics_solve(&rotate_cmd, &chassis_output);
            //Chassis_Power_Control(&chassis_output, chassis_motors);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                //计算PID
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;
        default:
            break;
    }

    // 反馈底盘数据回决策层
    chassis_feedback.chassis_wz = cmd->wz;
    xQueueOverwrite(Chassis_feedback_queue_handle, &chassis_feedback); // 使用队列发送反馈信息
    
}

/**
 * @brief 全向轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_omni_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output) {
    //目前以电池所在位置为后方，其对面为正前方
    // 计算各轮子线速度 (rad/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] = -cmd->vx - cmd->vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 右前轮
    wheel_linear_speed[1] = -cmd->vx + cmd->vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 左后轮
    wheel_linear_speed[2] = cmd->vx + cmd->vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 左前轮
    wheel_linear_speed[3] = cmd->vx - cmd->vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 右后轮
    for (int i = 0; i < 4; i++) {
        output->motor_speed[i] = wheel_linear_speed[i];
    }
}

/**
 * @brief 麦克纳姆轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_mecanum_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output) {
    // 麦克纳姆轮布局（从俯视图看）：
    // 左前(0)  右前(1)
    // 左后(2)  右后(3)
    //
    // 麦克纳姆轮运动学模型
    // 左前轮 = vx - vy - wz*(wheelbase + track_width)/2
    // 右前轮 = vx + vy + wz*(wheelbase + track_width)/2
    // 左后轮 = vx + vy - wz*(wheelbase + track_width)/2
    // 右后轮 = vx - vy + wz*(wheelbase + track_width)/2

    float L = chassis_params.wheel_base; // 轮距
    float W = chassis_params.track_width; // 轮宽
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
void Chassis_kinematics_solve(const Chassis_cmd_send_t *cmd, Chassis_output_t *output) {
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
