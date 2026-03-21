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

#include "SEGGER_RTT.h"

// 用于控制 RTT 打印的频率
enum {
    GIMBAL_PITCH_RTT_HZ = 500U,
    GIMBAL_PITCH_RTT_PERIOD_MS = 1000U / GIMBAL_PITCH_RTT_HZ,
    GIMBAL_PITCH_VOFA_RTT_CHANNEL = 1U,
    GIMBAL_PITCH_VOFA_RTT_BUFFER_SIZE = 1024U,
    GIMBAL_YAW_RTT_HZ = 500U,
    GIMBAL_YAW_RTT_PERIOD_MS = 1000U / GIMBAL_YAW_RTT_HZ,
    GIMBAL_YAW_VOFA_RTT_CHANNEL = GIMBAL_PITCH_VOFA_RTT_CHANNEL,
    GIMBAL_YAW_VOFA_RTT_BUFFER_SIZE = 1024U,
};

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

//云台PITCH重力补偿
static float pitch_gravity_factor = 0.0f;

// 用于 Cortex-Debug 在线改 pitch 目标角 与 RTT 不同端口打印
volatile float gimbal_pitch_gravity_test_target_deg = 0.0f;

static char gimbal_pitch_vofa_rtt_buffer[GIMBAL_PITCH_VOFA_RTT_BUFFER_SIZE];
static uint32_t gimbal_pitch_rtt_last_print_tick = 0U;
static uint8_t gimbal_pitch_rtt_channel_ready = 0U;
static char gimbal_yaw_vofa_rtt_buffer[GIMBAL_YAW_VOFA_RTT_BUFFER_SIZE];
static uint32_t gimbal_yaw_rtt_last_print_tick = 0U;
static uint8_t gimbal_yaw_rtt_channel_ready = 0U;

static void Gimbal_pitch_rtt_init(void) {
    if (gimbal_pitch_rtt_channel_ready == 0U) {
        SEGGER_RTT_ConfigUpBuffer(GIMBAL_PITCH_VOFA_RTT_CHANNEL,
                                  "VOFA-PITCH",
                                  gimbal_pitch_vofa_rtt_buffer,
                                  sizeof(gimbal_pitch_vofa_rtt_buffer),
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP);
        gimbal_pitch_rtt_channel_ready = 1U;
    }
}

static void Gimbal_yaw_rtt_init(void) {
    if (gimbal_yaw_rtt_channel_ready == 0U) {
        SEGGER_RTT_ConfigUpBuffer(GIMBAL_YAW_VOFA_RTT_CHANNEL,
                                  "VOFA-PITCH",
                                  gimbal_yaw_vofa_rtt_buffer,
                                  sizeof(gimbal_yaw_vofa_rtt_buffer),
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP);
        gimbal_yaw_rtt_channel_ready = 1U;
    }
}

static void Gimbal_pitch_rtt_vofa_print(float pitch_target_deg) {
    if (pitch_motor == NULL || gimbal_imu_data == NULL) {
        return;
    }

    const float pitch_measure_deg = gimbal_imu_data->euler.pitch;

    Gimbal_pitch_rtt_init();

    uint32_t now = HAL_GetTick();
    if ((now - gimbal_pitch_rtt_last_print_tick) < GIMBAL_PITCH_RTT_PERIOD_MS) {
        return;
    }
    gimbal_pitch_rtt_last_print_tick = now;

    char rtt_line[128];
    int len = snprintf(rtt_line, sizeof(rtt_line),
                       "%.3f,%.3f,%.5f,%.3f,%.3f,%.3f\r\n",
                       pitch_target_deg,
                       pitch_measure_deg,
                       pitch_gravity_factor,
                       pitch_motor->motor_pid.speed_pid.Pout,
                       pitch_motor->motor_pid.speed_pid.Iout,
                       pitch_motor->motor_pid.speed_pid.Output);
    if (len > 0) {
        SEGGER_RTT_WriteString(GIMBAL_PITCH_VOFA_RTT_CHANNEL, rtt_line);
    }
}

static void Gimbal_yaw_rtt_vofa_print(float yaw_target_deg) {
    if (yaw_motor == NULL || gimbal_imu_data == NULL) {
        return;
    }

    const float yaw_measure_deg = gimbal_imu_data->total_yaw;

    Gimbal_yaw_rtt_init();

    uint32_t now = HAL_GetTick();
    if ((now - gimbal_yaw_rtt_last_print_tick) < GIMBAL_YAW_RTT_PERIOD_MS) {
        return;
    }
    gimbal_yaw_rtt_last_print_tick = now;

    char rtt_line[128];
    int len = snprintf(rtt_line, sizeof(rtt_line),
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                       yaw_target_deg,
                       yaw_measure_deg,
                       yaw_motor->motor_pid.angle_pid.Pout,
                       yaw_motor->motor_pid.speed_pid.Pout,
                       yaw_motor->motor_pid.speed_pid.Iout,
                       yaw_motor->motor_pid.speed_pid.Output);
    if (len > 0) {
        SEGGER_RTT_WriteString(GIMBAL_YAW_VOFA_RTT_CHANNEL, rtt_line);
    }
}


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
                .kd = 0.1,
                .deadband = 0.0f,
                .max_out = 800,
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL, // 角度环输出限幅 + 梯形积分
            },
            .speed_pid = {
                .kp = 90,
                .ki = 8.0,
                .kd = 0.0,
                .deadband = 0.0f,
                .max_out = 25000,
                .max_iout = 10000,
                .feedfoward_coefficient = 0.5f,
                .target_ff_coef = 0.2f, // 目标值前馈系数 (实测调整)
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
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler.pitch),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_body.x),
            .angle_pid = {
                .kp = 40.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .max_out = 900.0f,
                .max_iout = 100.0f,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_DIFFERENTIAL_GO_FIRST,
            },
            .speed_pid = {
                .kp = 100.0f,
                .ki = 22.0f,
                .kd = 0.0f,
                .deadband = 0.1f,
                .max_out = 20000.0f,
                .max_iout = 8000.0f,
                // .LPF_coefficient = 0.9f,
                // 前馈参数
                .target_ff_coef = 0.25f, // 目标值前馈系数 (实测调整)
                .feedforward_source = &pitch_gravity_factor, // cos 因子
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
    Gimbal_pitch_rtt_init();
    Gimbal_yaw_rtt_init();

    Vision_Comm_Init();
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

        //重力补偿计算
        float pitch_rad = gimbal_imu_data->euler.pitch * (3.14159265f / 180.0f);
        pitch_gravity_factor = cosf(pitch_rad);

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
                // Djimotor_set_status(yaw_motor, MOTOR_STOP);
                // Djimotor_set_status(pitch_motor, MOTOR_STOP);
                //设置电机目标值

            
                // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f\r\n",cmd->yaw,gimbal_imu_data->total_yaw);
                Djimotor_set_target(yaw_motor, cmd->yaw);
                Djimotor_set_target(pitch_motor, cmd->pitch);
                // Djimotor_set_target(pitch_motor, 0);
               
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);

                // Gimbal_pitch_rtt_vofa_print(cmd->pitch);
                Gimbal_yaw_rtt_vofa_print(cmd->yaw);
               // Uart_printf(test_uart,"pitch_target:%.2f,%.2f,.%2f\r\n",pitch_target_deg,gimbal_imu_data->euler.pitch,pitch_motor->motor_pid.speed_pid.Iout);
                break;
                //云台视觉模式
            case GIMBAL_VISION_MODE:
                Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
                Djimotor_set_status(pitch_motor, MOTOR_ENABLED);
                
                if (Is_Vision_Online()) {
                    const Infantry_Vision_Rx_Data_t* v = Get_Vision_Data();
                    
                    Djimotor_set_target(yaw_motor, cmd->yaw + v->yaw_angle);
                    Djimotor_set_target(pitch_motor, cmd->pitch + v->pitch_angle);
                } else {
                    ERROR_CRITICAL("GIMBAL", "Vision data not available, cannot enter VISION_MODE");
                    Djimotor_set_target(yaw_motor, cmd->yaw);
                    Djimotor_set_target(pitch_motor, cmd->pitch);
                }
                
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);
                // Gimbal_pitch_rtt_vofa_print(cmd->pitch);
                break;

            default:
                break;
        }

        //Uart_printf(test_uart,"yaw_speed:%.2f,pitch_speed:%.2f\r\n",gimbal_imu_data->gyro_body.z,gimbal_imu_data->gyro_body.y);

        //Uart_printf(test_uart, "pitch:%.2f,%.2f,%.2f\r\n", pitch_motor->motor_pid.pid_target, *(pitch_motor->motor_pid.other_angle_feedback_ptr),pitch_motor->motor_pid.speed_pid.Output);
    /***************************************测试SHELL改云台电机参数********************/
    // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f,%d,%f\r\n",cmd->yaw,yaw_motor->motor_measure.total_angle
    //     ,yaw_motor->out_current,yaw_motor->motor_pid.speed_pid.kp);
   // VOFA_Send(test_uart,cmd->yaw,yaw_motor->motor_measure.total_angle,yaw_motor->out_current);
        //反馈数据
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        //ERROR_INFO("GIMBAL", "Yaw single round angle: %.2f", gimbal_feedback.yaw_motor_single_round_angle);
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

