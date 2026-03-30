/**
 * @file    gimbal.c
 * @brief   云台功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    云台电机初始化
 */

//
#include "gimbal.h"

#include <stdio.h>

#include "bsp_usart.h"
#include "dji_motor.h"
#include "decision_making.h"
#include "message_center.h"
#include "ins.h"
#include "robot_definitions.h"
#include "robot_task.h"
#include "vofa.h"
#include "math.h"
#include "shell.h"
#include "shell_port.h"
#include <stdlib.h> 
#include <string.h> 
#include "lowpass_filter.h"
#include "bmi088.h"
#include "error_handler.h"
#include "vision_comm.h"
#include "bsp_dwt.h"   // 用于获取微秒时间戳 DWT_GetTimeline_s()

#include "SEGGER_RTT.h"
#include "bsp_rtt.h"

//用于角度和弧度的转换
#define ANGLE_TO_RAD 0.01745329252f
#define RAD_TO_ANGLE 57.295779513f


//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//云台模块的姿态数据指针，指向ins模块的全局变量
static Ins_data_t *gimbal_imu_data;

// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

extern QueueHandle_t Gimbal_feedback_queue_handle; // 声明外部队列句柄

 Gimbal_cmd_send_t gimbal_cmd;

//云台PITCH重力补偿
static float pitch_gravity_factor = 0.0f;

static float pitch_gimbal;
static float pitch_speed;




/**
 * @brief 云台初始化
 */
static void Gimbal_motor_init(void) {

    //YAW电机
    Djimotor_init_config_t yaw_config = {   
        .motor_name = "yaw_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_ENABLED,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            
            //使用ins模块姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data->total_yaw),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_body.z),
            .angle_pid = {
                .kp = 30,
                .ki = 0,
                .kd = 0.8f,   // 加 D 项：角度环接近目标时减速，抑制过冲
                .deadband = 0.2f,
                .max_out = 650,   // 限制最大速度指令，防止电机惯性甩过目标
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_DIFFERENTIAL_GO_FIRST,
            },
            .speed_pid = {
                .kp = 90,//60
                .ki = 1.0,
                .kd = 0.01,
                .deadband = 0.2f,
                .max_out = 20000,
                .max_iout = 2000,
                // .feedfoward_coefficient = 0.05f,
                .target_ff_coef = 0.0f, // 目标值前馈系数 (实测调整)
                .LPF_coefficient = 0.0f,
                .integral_separation_threshold = 0.0f,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_FEEDFOWARD|PID_OUTPUT_FILTER,

            },
            

        },

        .can_init = {
            .can_handle = &hcan1,
            .can_id = 0x1FF,
            .tx_id = 1,
            .rx_id = 0x205, 
        },
    };

    yaw_motor = DJI_Motor_Init(&yaw_config);
//PITCH电机
    Djimotor_init_config_t pitch_config = {
        .motor_name = "pitch_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_ENABLED,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            //使用ins模块姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler.roll),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_body.x),
            .angle_pid = {
                .kp = 40.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .max_out = 1000.0f,
                .max_iout = 100.0f,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_DIFFERENTIAL_GO_FIRST,
            },
            .speed_pid = {
                .kp = 100.0f,
                .ki = 20.0f,
                .kd = 0.0f,
                .deadband = 0.0f,
                .max_out = 15000.0f,
                .max_iout = 2000.0f,
                // .LPF_coefficient = 0.9f,
                // 前馈参数
                .target_ff_coef = 0.2f, // 目标值前馈系数 (实测调整)
                // .feedforward_source = &pitch_gravity_factor, // cos 因子
                .feedfoward_coefficient = 5500.0f,           // 需要实测
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_FEEDFOWARD,

            },

        },
    .can_init = {
        .can_handle = &hcan2,
        .can_id = 0x1FF,
        .tx_id = 2,
        .rx_id = 0x206,
    },
    };
    pitch_motor = DJI_Motor_Init(&pitch_config);


}



/**
 * @brief 云台任务初始化
 */
void Gimbal_task_init(void) {
    // 获取ins模块的姿态数据指针
    gimbal_imu_data = Ins_get_data();

    if (gimbal_imu_data == NULL) {
        ERROR_CRITICAL("GIMBAL", "Failed to get IMU data pointer from INS module");
        return;
    }

    //初始化云台电机
    Gimbal_motor_init();

    //视觉初始化


    Vision_Comm_Init();
}

Gimbal_state_e Gimbal_get_state(void)
{
    if ((gimbal_imu_data == NULL) || (yaw_motor == NULL) || (pitch_motor == NULL)) {
        return GIMBAL_STATE_INIT;
    }

    if (gimbal_imu_data->state == INS_STATE_READY) {
        return GIMBAL_STATE_READY;
    }

    return GIMBAL_STATE_INIT;
}


/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd) {
        //只有当IMU就绪时才可以控制云台
        //安全保护
        if(gimbal_imu_data->state != INS_STATE_READY){
            return;
        }
        // =========================================================
   // =========================================================
    // 1. 视觉通信层：无脑收发 (在物理控制前执行，确保目标最新)
    // =========================================================
    
    // 极速解析 NUC 发来的最新预测指令 (非阻塞提取 FIFO)
    Vision_Comm_Parse_Task();

    uint32_t current_us = (uint32_t)(DWT_GetTimeline_s() * 1000000.0f);

    // 单位转换：Degree -> Radian，提供给 NUC
    // 注意：你的代码中使用 euler.roll 代指 pitch
    float pitch_rad = gimbal_imu_data->euler.roll * ANGLE_TO_RAD; 
    float yaw_rad   = gimbal_imu_data->total_yaw * ANGLE_TO_RAD;
    float pitch_v_rad = gimbal_imu_data->gyro_body.x * ANGLE_TO_RAD;
    float yaw_v_rad   = gimbal_imu_data->gyro_body.z * ANGLE_TO_RAD;

    // 疯狂发报：送出绝对时间戳、连续 Yaw/Pitch 弧度、弧度角速度
    Vision_Send_Pose(current_us, 
                     pitch_rad, 
                     yaw_rad, 
                     pitch_v_rad, 
                     yaw_v_rad, 
                     600,  // 此处需替换为裁判系统当前血量
                     600); // 此处需替换为裁判系统最大血量

    // =========================================================
    // 2. 云台物理控制层
    // =========================================================
        //重力补偿计算
        float pitch_rad2 = gimbal_imu_data->euler.pitch * (3.14159265f / 180.0f);
        pitch_gravity_factor = cosf(pitch_rad2);

        // 根据控制模式进行处理
        gimbal_cmd  = *cmd; 
        switch (cmd->gimbal_mode) { 
            // 电流零输入,失能云台电机
            case GIMBAL_ZERO_FORCE:
                Djimotor_set_status(yaw_motor, MOTOR_STOP);
                Djimotor_set_status(pitch_motor, MOTOR_STOP);
                Djimotor_set_target(yaw_motor, 0);
                Djimotor_set_target(pitch_motor, 0);    
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);
                break;

            //云台陀螺仪反馈模式
            case GIMBAL_GYRO_MODE:
                //使能电机
                Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
                Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

                //设置电机目标值
                // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f\r\n",cmd->yaw,gimbal_imu_data->total_yaw);

                Djimotor_set_target(yaw_motor, cmd->yaw);
                Djimotor_set_target(pitch_motor, cmd->pitch);
               
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);

                // Gimbal_pitch_rtt_vofa_print(cmd->pitch);
               // Uart_printf(test_uart,"pitch_target:%.2f,%.2f,.%2f\r\n",pitch_target_deg,gimbal_imu_data->euler.pitch,pitch_motor->motor_pid.speed_pid.Iout);
                break;
             //云台视觉模式
            case GIMBAL_VISION_MODE:
                Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
                Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

                {
                    const Vision_Ctrl_Data_t* v_cmd = Get_Vision_Ctrl_Data();
                    static uint32_t last_frame_id    = 0;
                    static float    abs_yaw_target   = 0.0f;
                    static float    abs_pitch_target = 0.0f;

                    // frame_id > 0 才说明真正收到过视觉包，避免启动时 last_valid_time=0 误判在线
                    if (Is_Vision_Online() && v_cmd->frame_id > 0) {
                        if (v_cmd->frame_id != last_frame_id) {
                            last_frame_id    = v_cmd->frame_id;
                            // NUC 发的是增量（当前位置到目标的偏差），每帧用真实 IMU 角度重新换算绝对目标
                            abs_yaw_target   = gimbal_imu_data->total_yaw  + v_cmd->target_yaw   * RAD_TO_ANGLE;
                            abs_pitch_target = gimbal_imu_data->euler.roll + v_cmd->target_pitch * RAD_TO_ANGLE;
                            ERROR_INFO("GIMBAL", "Vision frame %lu: delta_yaw=%.2f delta_pitch=%.2f -> abs_yaw=%.2f abs_pitch=%.2f",
                                       v_cmd->frame_id,
                                       v_cmd->target_yaw * RAD_TO_ANGLE, v_cmd->target_pitch * RAD_TO_ANGLE,
                                       abs_yaw_target, abs_pitch_target);
                        }

                        Djimotor_set_target(yaw_motor,   abs_yaw_target);
                        Djimotor_set_target(pitch_motor, abs_pitch_target);
                        // ERROR_INFO("GIMBAL", "target: yaw=%.2f pitch=%.2f imu_yaw=%.2f imu_pitch=%.2f",
                        //            abs_yaw_target, abs_pitch_target,
                        //            gimbal_imu_data->total_yaw, gimbal_imu_data->euler.roll);
                    }
                // else {
                //         // 视觉未就绪或掉线，回退到遥控目标
                //         abs_yaw_target   = cmd->yaw;
                //         abs_pitch_target = cmd->pitch;
                //         last_frame_id    = 0;
                //         Djimotor_set_target(yaw_motor,   cmd->yaw);
                //         Djimotor_set_target(pitch_motor, cmd->pitch);
                //     }
                    Djimotor_Calc_Output(yaw_motor);
                    Djimotor_Calc_Output(pitch_motor);
                }
                break;

            default:
                break;
        }
        //反馈数据
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        // 把实际执行的目标回写，供决策层模式切换时做无扰同步
        gimbal_feedback.active_yaw_target   = yaw_motor->motor_pid.pid_target;
        gimbal_feedback.active_pitch_target = pitch_motor->motor_pid.pid_target;
        xQueueOverwrite(Gimbal_feedback_queue_handle, &gimbal_feedback);
}


