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

#include "shell.h"
#include "shell_port.h"
#include <stdlib.h> 
#include <string.h> 
#include "lowpass_filter.h"
#include "bmi088.h" // 引用驱动头文件

//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//云台模块的姿态数据指针，指向ins模块的全局变量
static Ins_data_t *gimbal_imu_data;

// 发布给决策层的云台反馈信息
// static Publisher_t*gimbal_pub;
// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

extern QueueHandle_t Gimbal_feedback_queue_handle; // 新增：声明外部队列句柄

 Gimbal_cmd_send_t gimbal_cmd;
// static Lpf_t yaw_target_lpf; // Yaw 目标值的低通滤波器实例


/**
 * @brief 云台初始化
 */
static void Gimbal_motor_init(void) {

    //初始化YAW低通滤波器
    // LPF_Init(&yaw_target_lpf,0.001f,10.0f,0.0f); // 1000Hz控制频率，10Hz截止频率，初始值0



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
                .kp = 40,
                .ki = 0,
                .kd = 0,
                .deadband = 0.0f,
                .max_out = 300,
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL, // 角度环输出限幅 + 梯形积分
            },
            .speed_pid = {
                .kp = 60,
                .ki = 8.0,
                .kd = 0.0,
                .deadband = 0.0f,
                .max_out = 25000,
                .max_iout = 8000,
                .feedfoward_coefficient = 0.05f,
                .LPF_coefficient = 0.0f,
                .integral_separation_threshold = 0.0f,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_FEEDFOWARD,

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
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler.pitch),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_body.y),
            .angle_pid = {
                .kp = 30,
                .ki = 0,
                .kd = 0,
                .max_out = 500,
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL,
            },
            .speed_pid = {
                .kp = 60,
                .ki = 20,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 12000,
                .max_iout = 3000,
                .feedfoward_coefficient = 0.1f,
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
        while(1); 
    }

    //初始化云台电机
    Gimbal_motor_init();
}


/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd) {
    // 从消息中心获取最新的控制指令
        // 根据控制模式进行处理
        gimbal_cmd  = *cmd; // 复制一份本地变量，避免直接修改指针数据
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
                // Djimotor_set_status(yaw_motor, MOTOR_STOP);
                // Djimotor_set_status(pitch_motor, MOTOR_STOP);
                //设置电机目标值

                //低通平滑
                // // 1. 获取阶跃的遥控器目标
                // float raw_yaw_target = cmd->yaw;
                // // 2. 使用低通滤波器将阶梯变成平滑斜坡
                // float smooth_yaw_target = LPF_Calc(&yaw_target_lpf, raw_yaw_target);

                Djimotor_set_target(yaw_motor, cmd->yaw);

                // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f\r\n",cmd->yaw,gimbal_imu_data->total_yaw);

                // Djimotor_set_target(yaw_motor, smooth_yaw_target);  
                Djimotor_set_target(pitch_motor, cmd->pitch);
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);
                Uart_printf(test_uart,"pitch_target:%.2f,%.2f\r\n",cmd->pitch,gimbal_imu_data->euler.pitch);
                break;
                //云台视觉模式
            case GIMBAL_VISION_MODE:
                //根据视觉补充

                break;

            default:
                break;
        }


        //Uart_printf(test_uart, "pitch:%.2f,%.2f,%.2f\r\n", pitch_motor->motor_pid.pid_target, *(pitch_motor->motor_pid.other_angle_feedback_ptr),pitch_motor->motor_pid.speed_pid.Output);
    /***************************************测试SHELL改云台电机参数********************/
    // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f,%d,%f\r\n",cmd->yaw,yaw_motor->motor_measure.total_angle
    //     ,yaw_motor->out_current,yaw_motor->motor_pid.speed_pid.kp);
   // VOFA_Send(test_uart,cmd->yaw,yaw_motor->motor_measure.total_angle,yaw_motor->out_current);
        //反馈数据
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        //推送消息
        // 将当前的电机状态（编码器数据）发布给决策层，用于下一帧的闭环控制或逻辑判断
        // Pub_push_message(gimbal_pub, (void *) &gimbal_feedback);
        xQueueOverwrite(Gimbal_feedback_queue_handle, &gimbal_feedback);
}
/**
 * @brief 在线修改 Yaw 电机 PID 及限幅
 * @usage yaw_pid -s/-a <kp> <ki> <kd> [max_out] [max_iout]
 */
int set_yaw_pid_cmd(int argc, char *argv[])
{
    // 安全检查
    if (yaw_motor == NULL) {
        shellPrint(&shell, "Error: Yaw motor is NULL!\r\n");
        return -1;
    }

    // 参数数量检查
    if (argc < 5) {
        shellPrint(&shell, "Usage: yaw_pid -s(speed)/-a(angle) <kp> <ki> <kd> [max_out] [max_iout]\r\n");
        return -1;
    }

    Pid_instance_t *target_pid = NULL;

    char *mode_str = argv[1];
    char *type_name = "";

    // 4. 根据输入决定指针指向谁
    if (strcmp(mode_str, "-s") == 0) {
        // 指向速度环 PID
        target_pid = &(yaw_motor->motor_pid.speed_pid);
        type_name = "Speed";
    }
    else if (strcmp(mode_str, "-a") == 0) {
        // 指向角度环 PID
        target_pid = &(yaw_motor->motor_pid.angle_pid);
        type_name = "Angle";
    }
    else {
        shellPrint(&shell, "Error: Unknown mode '%s'. Use -s or -a\r\n", mode_str);
        return -1;
    }

    // 5. 修改参数 (通过指针操作)
    target_pid->kp = (float)atof(argv[2]);
    target_pid->ki = (float)atof(argv[3]);
    target_pid->kd = (float)atof(argv[4]);

    // 6. 修改限幅 (如果有输入的话)
    if (argc >= 6) target_pid->max_out  = (float)atof(argv[5]);
    if (argc >= 7) target_pid->max_iout = (float)atof(argv[6]);

    // 7. 打印反馈
    shellPrint(&shell, "[Gimbal] Set Yaw %s PID Success!\r\n", type_name);
    shellPrint(&shell, "  Kp: %.3f, Ki: %.3f, Kd: %.3f\r\n",
               target_pid->kp, target_pid->ki, target_pid->kd);
    shellPrint(&shell, "  MaxOut: %.0f, MaxIOut: %.0f\r\n",
               target_pid->max_out, target_pid->max_iout);

    return 0;
}

// 导出命令
// 注意：虽然函数在 gimbal.c，但 Letter-Shell 会通过链接脚本自动找到它，无论它在哪里
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), yaw_pid, set_yaw_pid_cmd, Tune Yaw PID);

