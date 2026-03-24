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

// 用于控制 RTT 打印的频率
enum {
    GIMBAL_PITCH_RTT_HZ = 500U,
    GIMBAL_PITCH_RTT_PERIOD_MS = 1000U / GIMBAL_PITCH_RTT_HZ,
    GIMBAL_PITCH_VOFA_RTT_CHANNEL = 1U,
    GIMBAL_PITCH_VOFA_RTT_BUFFER_SIZE = 1024U,
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

static float pitch_gimbal;
static float pitch_speed;


// 无扰切换核心时间参数：
// 1. blend   决定 active_ref 从旧目标过渡到新目标的总时间
// 2. ff blend 决定视觉速度前馈的渐入/渐出时间
// 3. ref tau 决定视觉绝对角的一阶滤波强度
// 4. i hold  决定切换初期冻结积分的时长
// 这一组参数优先保证“不过冲、不抽搐”，再去追求切换速度。
#define GIMBAL_MODE_BLEND_TIME_S       0.08f
#define GIMBAL_VISION_FF_BLEND_TIME_S  0.05f
#define GIMBAL_VISION_REF_FILTER_TAU_S 0.03f
#define GIMBAL_I_HOLD_TIME_S           0.03f
#define GIMBAL_CONTROL_DT_MIN_S        0.0001f
#define GIMBAL_CONTROL_DT_MAX_S        0.02f
#define GIMBAL_VISION_YAW_FF_GAIN      30.0f
#define GIMBAL_VISION_PITCH_FF_GAIN    30.0f

// 视觉解算系到 IMU 世界系的小角度偏差补偿量
volatile float gimbal_vision_yaw_bias_deg = 0.0f;
volatile float gimbal_vision_pitch_bias_deg = 0.0f;

// 无扰切换统一状态：
// - active_ref 是真正送给底层 PID 的唯一目标
// - transition_* 负责在模式切换时保证目标连续
// - vision_filtered_* 负责对视觉绝对角做轻量滤波
// - vision_ff_blend 负责视觉速度前馈的渐入渐出
// - *_ki_saved + integral_hold_active 负责短时冻结积分但保留历史 Iout
typedef struct {
    bool initialized;
    bool vision_source_active;
    bool vision_filter_initialized;
    bool transition_active;
    bool integral_hold_active;
    gimbal_mode_e last_mode;

    // 当前统一目标 active_ref，任何模式下都只允许改这里
    float active_yaw_target;
    float active_pitch_target;

    // S 曲线过渡器的起点、终点和累计时间
    float transition_from_yaw;
    float transition_from_pitch;
    float transition_to_yaw;
    float transition_to_pitch;
    float transition_elapsed_s;

    // 视觉绝对角在进入 active_ref 之前的滤波结果
    float vision_filtered_yaw;
    float vision_filtered_pitch;

    // 视觉速度前馈渐入系数，以及积分冻结剩余时间
    float vision_ff_blend;
    float integral_hold_time_s;

    // 保存最近一拍视觉给出的角速度，供前馈渐入阶段使用
    float last_vision_yaw_rate;
    float last_vision_pitch_rate;

    // 冻结积分时只把 Ki 临时置零，恢复时再写回原参数
    float yaw_speed_ki_saved;
    float yaw_angle_ki_saved;
    float pitch_speed_ki_saved;
    float pitch_angle_ki_saved;

    // 统一用 DWT 统计控制周期，避免不同路径各算各的 dt
    uint32_t dwt_counter;
} Gimbal_bumpless_state_t;

static Gimbal_bumpless_state_t gimbal_bumpless_state = {
    .last_mode = GIMBAL_ZERO_FORCE,
};

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

static void Gimbal_pitch_rtt_vofa_print(float pitch_target_deg) {
    if (pitch_motor == NULL || gimbal_imu_data == NULL) {
        return;
    }

    const float pitch_measure_deg = gimbal_imu_data->euler.roll;

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
                .kd = 0,
                .deadband = 0.2f,
                .max_out = 800,
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL, // 角度环输出限幅 + 梯形积分
            },
            .speed_pid = {
                .kp = 90,//60
                .ki = 10.0,
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
    Gimbal_pitch_rtt_init();

    //暂时注释掉视觉初始化
    // Vision_Comm_Init();
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
    // 视觉通信层：无脑收发 (在物理控制前执行，确保目标最新)
    // =========================================================
    
    // 极速解析 NUC 发来的最新预测指令 (非阻塞)
    // Vision_Comm_Parse_Task();

    // uint32_t current_us = (uint32_t)(DWT_GetTimeline_s() * 1000000.0f);

    //     // 疯狂发报：送出绝对时间戳、连续 Yaw 角、纯净角速度、以及当前血量
    //     // TODO: 如果你已经接入了裁判系统，把这里的 600 替换成真正的裁判系统全局变量！
    // Vision_Send_Pose(current_us,
    //                      gimbal_imu_data->euler.roll,
    //                      gimbal_imu_data->total_yaw,
    //                      gimbal_imu_data->gyro_body.x,
    //                      gimbal_imu_data->gyro_body.z,
    //                      600,  // 测试用 Current HP
    //                      600); // 测试用 Maximum HP

    // Rtt_Printf(1,"pitch:%.2f,yaw:%.2f,pitch_speed:%.2f,yaw_speed:%.2f\r\n",gimbal_imu_data->euler.roll,
    // gimbal_imu_data->euler.yaw,gimbal_imu_data->gyro_body.x,gimbal_imu_data->gyro_body.z);

    // =========================================================
    // 2. 云台物理控制层
    // =========================================================
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

                if (Is_Vision_Online()) {
                    // 获取 NUC 的预测数据
                    const Vision_Ctrl_Data_t* v_cmd = Get_Vision_Ctrl_Data();

                    // 绝对坐标系追踪：直接把预测的世界坐标扔给 PID
                    Djimotor_set_target(yaw_motor, v_cmd->target_yaw);
                    Djimotor_set_target(pitch_motor, v_cmd->target_pitch);

                    // 计算基础 PID 输出 (包含 PITCH 重力补偿)
                    Djimotor_Calc_Output(yaw_motor);
                    Djimotor_Calc_Output(pitch_motor);

                    // 在电流层直接叠加上视觉速度前馈！
                    yaw_motor->out_current += (int16_t)(30.0f * v_cmd->target_yaw_v);
                    pitch_motor->out_current += (int16_t)(30.0f * v_cmd->target_pitch_v);

                } else {
                    //视觉掉线，云台瞬间停止在当前绝对角度
                    Djimotor_set_target(yaw_motor, gimbal_imu_data->total_yaw);
                    Djimotor_set_target(pitch_motor, gimbal_imu_data->euler.pitch);

                    Djimotor_Calc_Output(yaw_motor);
                    Djimotor_Calc_Output(pitch_motor);
                }
                break;

            default:
                break;
        }
        //反馈数据
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        xQueueOverwrite(Gimbal_feedback_queue_handle, &gimbal_feedback);
}


