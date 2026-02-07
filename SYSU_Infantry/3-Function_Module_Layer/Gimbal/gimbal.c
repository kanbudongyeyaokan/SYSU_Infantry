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


//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//云台模块的姿态数据指针，指向ins模块的全局变量
static Ins_data_t *gimbal_imu_data;

// 发布给决策层的云台反馈信息
static Publisher_t*gimbal_pub;
// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

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
            .angle_source = MOTOR_FEEDBACK,
            .speed_source = MOTOR_FEEDBACK,
            //使用ins模块姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler.yaw),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_body.z),
            .angle_pid = {
                .kp = 12,
                .ki = 0,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 500,
                .max_iout = 100,

            },
            .speed_pid = {
                .kp = 40,
                .ki = 2.0,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 30000,
                .max_iout = 15000,
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
                .kp = 10,
                .ki = 0,
                .kd = 0,
                .max_out = 500,
                .max_iout = 100,
                //可补充
            },
            .speed_pid = {
                .kp = 5,
                .ki = 0,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 2500,
                .max_iout = 20000,
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

    //初始化云台电机
    Gimbal_motor_init();

    // 注册底盘反馈信息发布者
    gimbal_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));

}


/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd) {
    // 从消息中心获取最新的控制指令
        // 根据控制模式进行处理
        switch (cmd->gimbal_mode) {
            // 电流零输入,失能云台电机
            case GIMBAL_ZERO_FORCE:
                Djimotor_set_status(yaw_motor, MOTOR_STOP);
                Djimotor_set_status(pitch_motor, MOTOR_STOP);
                break;

            //云台陀螺仪反馈模式
            case GIMBAL_GYRO_MODE:
                //使能电机
                Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
                Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

                //设置电机目标值
                Djimotor_set_target(yaw_motor, cmd->yaw);
                Djimotor_set_target(pitch_motor, cmd->pitch);
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);

                break;
            //云台视觉模式
            case GIMBAL_VISION_MODE:
                //根据视觉补充

                break;

            default:
                break;
        }

    /***************************************测试SHELL改云台电机参数********************/
    // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f,%d,%f\r\n",cmd->yaw,yaw_motor->motor_measure.total_angle
    //     ,yaw_motor->out_current,yaw_motor->motor_pid.speed_pid.kp);
   // VOFA_Send(test_uart,cmd->yaw,yaw_motor->motor_measure.total_angle,yaw_motor->out_current);
        //反馈数据
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        //推送消息
        Pub_push_message(gimbal_pub, (void *) &gimbal_feedback);
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

