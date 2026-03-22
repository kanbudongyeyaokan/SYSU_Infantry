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

static float Gimbal_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Gimbal_smoothstep01(float x)
{
    // 用 S 曲线而不是线性插值，避免切换开始和结束时目标角速度出现折点
    x = Gimbal_clampf(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static float Gimbal_lpf_step(float current, float target, float tau_s, float dt_s)
{
    // 视觉绝对角更新通常带抖动，这里做一阶滤波，降低切换后头几拍的目标抖动
    if (tau_s <= 0.0f) {
        return target;
    }

    dt_s = Gimbal_clampf(dt_s, GIMBAL_CONTROL_DT_MIN_S, GIMBAL_CONTROL_DT_MAX_S);
    float alpha = dt_s / (tau_s + dt_s);
    alpha = Gimbal_clampf(alpha, 0.0f, 1.0f);
    return current + alpha * (target - current);
}

static float Gimbal_unwrap_to_nearest(float angle_deg, float reference_deg)
{
    // 把视觉单圈角对齐到当前多圈 yaw 附近，避免 179/-180 一类的伪跳变
    float unwrapped = angle_deg;

    while ((unwrapped - reference_deg) > 180.0f) {
        unwrapped -= 360.0f;
    }
    while ((unwrapped - reference_deg) < -180.0f) {
        unwrapped += 360.0f;
    }

    return unwrapped;
}

static void Gimbal_init_bumpless_state(float measured_yaw, float measured_pitch)
{
    // 初始化时直接把 active_ref 对齐当前实测姿态：
    // 1. 第一拍不追历史目标
    // 2. 退出失能后也不会立刻回跳到旧指令
    gimbal_bumpless_state.initialized = true;
    gimbal_bumpless_state.vision_source_active = false;
    gimbal_bumpless_state.vision_filter_initialized = false;
    gimbal_bumpless_state.transition_active = false;
    gimbal_bumpless_state.integral_hold_active = false;
    gimbal_bumpless_state.last_mode = GIMBAL_ZERO_FORCE;
    gimbal_bumpless_state.active_yaw_target = measured_yaw;
    gimbal_bumpless_state.active_pitch_target = measured_pitch;
    gimbal_bumpless_state.transition_from_yaw = measured_yaw;
    gimbal_bumpless_state.transition_from_pitch = measured_pitch;
    gimbal_bumpless_state.transition_to_yaw = measured_yaw;
    gimbal_bumpless_state.transition_to_pitch = measured_pitch;
    gimbal_bumpless_state.transition_elapsed_s = 0.0f;
    gimbal_bumpless_state.vision_filtered_yaw = measured_yaw;
    gimbal_bumpless_state.vision_filtered_pitch = measured_pitch;
    gimbal_bumpless_state.vision_ff_blend = 0.0f;
    gimbal_bumpless_state.integral_hold_time_s = 0.0f;
    gimbal_bumpless_state.last_vision_yaw_rate = 0.0f;
    gimbal_bumpless_state.last_vision_pitch_rate = 0.0f;
    DWT_GetDeltaT(&gimbal_bumpless_state.dwt_counter);
}

static void Gimbal_start_transition(float target_yaw, float target_pitch)
{
    // 锁存切换瞬间的 active_ref 作为过渡起点。
    // 后面即使视觉目标继续刷新，也是在“旧目标 -> 新目标”的连续轨迹上逼近。
    gimbal_bumpless_state.transition_active = true;
    gimbal_bumpless_state.transition_elapsed_s = 0.0f;
    gimbal_bumpless_state.transition_from_yaw = gimbal_bumpless_state.active_yaw_target;
    gimbal_bumpless_state.transition_from_pitch = gimbal_bumpless_state.active_pitch_target;
    gimbal_bumpless_state.transition_to_yaw = target_yaw;
    gimbal_bumpless_state.transition_to_pitch = target_pitch;
}

static void Gimbal_set_integral_hold(bool enable)
{
    if ((yaw_motor == NULL) || (pitch_motor == NULL)) {
        return;
    }

    if (enable) {
        // 仅冻结 Ki，不清空历史 Iout。
        // 这样可以保留维持当前姿态所需的偏置力矩，避免切换时因为积分清零导致力矩塌陷。
        if (!gimbal_bumpless_state.integral_hold_active) {
            gimbal_bumpless_state.yaw_speed_ki_saved = yaw_motor->motor_pid.speed_pid.ki;
            gimbal_bumpless_state.yaw_angle_ki_saved = yaw_motor->motor_pid.angle_pid.ki;
            gimbal_bumpless_state.pitch_speed_ki_saved = pitch_motor->motor_pid.speed_pid.ki;
            gimbal_bumpless_state.pitch_angle_ki_saved = pitch_motor->motor_pid.angle_pid.ki;
            gimbal_bumpless_state.integral_hold_active = true;
        }

        yaw_motor->motor_pid.speed_pid.ki = 0.0f;
        yaw_motor->motor_pid.angle_pid.ki = 0.0f;
        pitch_motor->motor_pid.speed_pid.ki = 0.0f;
        pitch_motor->motor_pid.angle_pid.ki = 0.0f;
        return;
    }

    if (gimbal_bumpless_state.integral_hold_active) {
        // 过渡窗口结束后恢复原 Ki，之前累积下来的 Iout 会继续自然参与闭环
        yaw_motor->motor_pid.speed_pid.ki = gimbal_bumpless_state.yaw_speed_ki_saved;
        yaw_motor->motor_pid.angle_pid.ki = gimbal_bumpless_state.yaw_angle_ki_saved;
        pitch_motor->motor_pid.speed_pid.ki = gimbal_bumpless_state.pitch_speed_ki_saved;
        pitch_motor->motor_pid.angle_pid.ki = gimbal_bumpless_state.pitch_angle_ki_saved;
        gimbal_bumpless_state.integral_hold_active = false;
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
                .kp = 60,
                .ki = 10.0,
                .kd = 0.0,
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
#if 0
static void Gimbal_handle_command_legacy(Gimbal_cmd_send_t *cmd) {
        //只有当IMU就绪时才可以控制云台
        //安全保护
        if(gimbal_imu_data->state != INS_STATE_READY){
            return;
        }
        // =========================================================
    // 1. 视觉通信层：无脑收发 (在物理控制前执行，确保目标最新)
    // =========================================================
    
    // A. 极速解析 NUC 发来的最新预测指令 (非阻塞)
    Vision_Comm_Parse_Task();

    // B. 分频发送当前绝对位姿 (1000Hz 降频到 500Hz 发送，防串口阻塞)
    static uint8_t vision_tx_divider = 0;
    if (++vision_tx_divider >= 2) { 
        vision_tx_divider = 0;
        
        uint32_t current_us = (uint32_t)(DWT_GetTimeline_s() * 1000000.0f);
        
        // 疯狂发报：送出绝对时间戳、连续 Yaw 角、纯净角速度
        Vision_Send_Pose(current_us, 
                         gimbal_imu_data->euler.pitch, 
                         gimbal_imu_data->total_yaw,   // 必须是累加的多圈 Yaw
                         gimbal_imu_data->euler.roll,
                         gimbal_imu_data->gyro_body.x, // Pitch 轴纯净角速度
                         gimbal_imu_data->gyro_body.z); // Yaw 轴纯净角速度
    }

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
                // Djimotor_set_status(yaw_motor, MOTOR_STOP);
                // Djimotor_set_status(pitch_motor, MOTOR_STOP);
                //设置电机目标值

            
                // Uart_printf(test_uart,"<yaw_target>:%.2f,%.2f\r\n",cmd->yaw,gimbal_imu_data->total_yaw);
                Djimotor_set_target(yaw_motor, cmd->yaw);
                Djimotor_set_target(pitch_motor, cmd->pitch);
                // Djimotor_set_target(pitch_motor, 0);
               
                Djimotor_Calc_Output(yaw_motor);
                Djimotor_Calc_Output(pitch_motor);

                Gimbal_pitch_rtt_vofa_print(cmd->pitch);
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
                    // 你的 Pitch 已经占用了 feedforward_source 做重力补偿，
                    // 最优雅的解法是算完 PID 后，手动在底层电流上加一把推力。
                    // 这里的系数(比如 30.0f) 需要实车调参，越高对敌方移动的响应越暴力
                    yaw_motor->out_current += (int16_t)(30.0f * v_cmd->target_yaw_v);
                    pitch_motor->out_current += (int16_t)(30.0f * v_cmd->target_pitch_v);
                
                } else {
                    //视觉掉线，云台瞬间停止在当前绝对角度
                    // ERROR_WARN("GIMBAL", "Vision Offline! Hold position.");
                    Djimotor_set_target(yaw_motor, gimbal_imu_data->total_yaw);
                    Djimotor_set_target(pitch_motor, gimbal_imu_data->euler.pitch);
                    
                    Djimotor_Calc_Output(yaw_motor);
                    Djimotor_Calc_Output(pitch_motor);
                }
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
#endif
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd) {
        if ((cmd == NULL) || (gimbal_imu_data == NULL)) {
            return;
        }

        if (gimbal_imu_data->state != INS_STATE_READY) {
            return;
        }

        pitch_gimbal = gimbal_imu_data->euler.pitch;
        pitch_speed = gimbal_imu_data->gyro_body.x;

        Vision_Comm_Parse_Task();

        static uint8_t vision_tx_divider = 0;
        if (++vision_tx_divider >= 2) {
            vision_tx_divider = 0;

            uint32_t current_us = (uint32_t)(DWT_GetTimeline_s() * 1000000.0f);
            Vision_Send_Pose(current_us,
                             gimbal_imu_data->euler.pitch,
                             gimbal_imu_data->total_yaw,
                             gimbal_imu_data->euler.roll,
                             gimbal_imu_data->gyro_body.x,
                             gimbal_imu_data->gyro_body.z);
        }

        float dt = DWT_GetDeltaT(&gimbal_bumpless_state.dwt_counter);
        dt = Gimbal_clampf(dt, GIMBAL_CONTROL_DT_MIN_S, GIMBAL_CONTROL_DT_MAX_S);

        const float measured_yaw = gimbal_imu_data->total_yaw;
        const float measured_pitch = gimbal_imu_data->euler.pitch;

        if (!gimbal_bumpless_state.initialized) {
            Gimbal_init_bumpless_state(measured_yaw, measured_pitch);
        }

        float pitch_rad = measured_pitch * (3.14159265f / 180.0f);
        pitch_gravity_factor = cosf(pitch_rad);

        gimbal_cmd = *cmd;

        if (cmd->gimbal_mode == GIMBAL_ZERO_FORCE) {
            // 进入失能时把 active_ref 收回当前实测姿态，防止恢复时追旧目标
            Gimbal_set_integral_hold(false);

            gimbal_bumpless_state.active_yaw_target = measured_yaw;
            gimbal_bumpless_state.active_pitch_target = measured_pitch;
            gimbal_bumpless_state.transition_active = false;
            gimbal_bumpless_state.vision_source_active = false;
            gimbal_bumpless_state.vision_filter_initialized = false;
            gimbal_bumpless_state.vision_ff_blend = 0.0f;
            gimbal_bumpless_state.integral_hold_time_s = 0.0f;
            gimbal_bumpless_state.last_vision_yaw_rate = 0.0f;
            gimbal_bumpless_state.last_vision_pitch_rate = 0.0f;

            Djimotor_set_status(yaw_motor, MOTOR_STOP);
            Djimotor_set_status(pitch_motor, MOTOR_STOP);
            Djimotor_set_target(yaw_motor, 0.0f);
            Djimotor_set_target(pitch_motor, 0.0f);
            Djimotor_Calc_Output(yaw_motor);
            Djimotor_Calc_Output(pitch_motor);
        } else {
            const bool leaving_zero_force = (gimbal_bumpless_state.last_mode == GIMBAL_ZERO_FORCE);
            const bool vision_online = Is_Vision_Online();
            const bool want_vision_source = (cmd->gimbal_mode == GIMBAL_VISION_MODE) && vision_online;
            const Vision_Ctrl_Data_t *v_cmd = want_vision_source ? Get_Vision_Ctrl_Data() : NULL;

            // desired_* 是当前模式希望追踪的“源目标”，
            // active_* 则是过渡器处理后真正下发给 PID 的“执行目标”。
            float desired_yaw = cmd->yaw;
            float desired_pitch = cmd->pitch;

            if (want_vision_source && (v_cmd != NULL)) {
                // 视觉绝对角先做参考系补差，再对齐到当前多圈 yaw 附近，
                // 最后再进入低通滤波，这样能同时压住坐标失配和单圈角跳变。
                float raw_yaw = Gimbal_unwrap_to_nearest(
                    v_cmd->target_yaw + gimbal_vision_yaw_bias_deg,
                    gimbal_bumpless_state.active_yaw_target);
                float raw_pitch = v_cmd->target_pitch + gimbal_vision_pitch_bias_deg;

                if (!gimbal_bumpless_state.vision_filter_initialized) {
                    gimbal_bumpless_state.vision_filtered_yaw = raw_yaw;
                    gimbal_bumpless_state.vision_filtered_pitch = raw_pitch;
                    gimbal_bumpless_state.vision_filter_initialized = true;
                } else {
                    gimbal_bumpless_state.vision_filtered_yaw = Gimbal_lpf_step(
                        gimbal_bumpless_state.vision_filtered_yaw,
                        raw_yaw,
                        GIMBAL_VISION_REF_FILTER_TAU_S,
                        dt);
                    gimbal_bumpless_state.vision_filtered_pitch = Gimbal_lpf_step(
                        gimbal_bumpless_state.vision_filtered_pitch,
                        raw_pitch,
                        GIMBAL_VISION_REF_FILTER_TAU_S,
                        dt);
                }

                desired_yaw = gimbal_bumpless_state.vision_filtered_yaw;
                desired_pitch = gimbal_bumpless_state.vision_filtered_pitch;
                gimbal_bumpless_state.last_vision_yaw_rate = v_cmd->target_yaw_v;
                gimbal_bumpless_state.last_vision_pitch_rate = v_cmd->target_pitch_v;
            } else {
                gimbal_bumpless_state.vision_filter_initialized = false;
            }

            if (leaving_zero_force) {
                // 失能恢复时把目标重新收回到当前姿态，防止恢复使能的第一拍追旧目标
                gimbal_bumpless_state.active_yaw_target = measured_yaw;
                gimbal_bumpless_state.active_pitch_target = measured_pitch;
                gimbal_bumpless_state.transition_active = false;
                gimbal_bumpless_state.vision_source_active = false;
            }

            if ((want_vision_source != gimbal_bumpless_state.vision_source_active) || leaving_zero_force) {
                // 进入视觉、退出视觉、或者从失能恢复，都按“目标源变化”处理：
                // 先启动 S 曲线过渡，再短时冻结积分，避免 Setpoint 和控制力矩同时跳变。
                gimbal_bumpless_state.vision_source_active = want_vision_source;
                Gimbal_start_transition(desired_yaw, desired_pitch);
                gimbal_bumpless_state.integral_hold_time_s = GIMBAL_I_HOLD_TIME_S;
            }

            if (gimbal_bumpless_state.transition_active) {
                // 过渡期间允许终点持续跟踪最新视觉点，避免 blend 结束后再补一拍大追踪
                gimbal_bumpless_state.transition_to_yaw = desired_yaw;
                gimbal_bumpless_state.transition_to_pitch = desired_pitch;
                gimbal_bumpless_state.transition_elapsed_s += dt;

                float blend = Gimbal_smoothstep01(
                    gimbal_bumpless_state.transition_elapsed_s / GIMBAL_MODE_BLEND_TIME_S);

                gimbal_bumpless_state.active_yaw_target =
                    gimbal_bumpless_state.transition_from_yaw +
                    (gimbal_bumpless_state.transition_to_yaw - gimbal_bumpless_state.transition_from_yaw) * blend;
                gimbal_bumpless_state.active_pitch_target =
                    gimbal_bumpless_state.transition_from_pitch +
                    (gimbal_bumpless_state.transition_to_pitch - gimbal_bumpless_state.transition_from_pitch) * blend;

                if (blend >= 1.0f) {
                    gimbal_bumpless_state.transition_active = false;
                }
            } else {
                gimbal_bumpless_state.active_yaw_target = desired_yaw;
                gimbal_bumpless_state.active_pitch_target = desired_pitch;
            }

            // 视觉速度前馈单独渐入渐出，避免切换瞬间电流突变
            {
                const float ff_target = want_vision_source ? 1.0f : 0.0f;
                const float ff_step = (GIMBAL_VISION_FF_BLEND_TIME_S > 0.0f) ?
                    (dt / GIMBAL_VISION_FF_BLEND_TIME_S) : 1.0f;

                if (ff_target > gimbal_bumpless_state.vision_ff_blend) {
                    gimbal_bumpless_state.vision_ff_blend = Gimbal_clampf(
                        gimbal_bumpless_state.vision_ff_blend + ff_step, 0.0f, 1.0f);
                } else {
                    gimbal_bumpless_state.vision_ff_blend = Gimbal_clampf(
                        gimbal_bumpless_state.vision_ff_blend - ff_step, 0.0f, 1.0f);
                }
            }

            if (gimbal_bumpless_state.integral_hold_time_s > 0.0f) {
                // 切换初期只冻结 Ki，不改 Iout，本质上是“保留偏置力矩、暂停继续积分”
                gimbal_bumpless_state.integral_hold_time_s -= dt;
                Gimbal_set_integral_hold(true);
            } else {
                Gimbal_set_integral_hold(false);
            }

            Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
            Djimotor_set_status(pitch_motor, MOTOR_ENABLED);
            Djimotor_set_target(yaw_motor, gimbal_bumpless_state.active_yaw_target);
            Djimotor_set_target(pitch_motor, gimbal_bumpless_state.active_pitch_target);
            Djimotor_Calc_Output(yaw_motor);
            Djimotor_Calc_Output(pitch_motor);

            if (gimbal_bumpless_state.vision_ff_blend > 0.0f) {
                yaw_motor->out_current += (int16_t)(
                    GIMBAL_VISION_YAW_FF_GAIN *
                    gimbal_bumpless_state.vision_ff_blend *
                    gimbal_bumpless_state.last_vision_yaw_rate);
                pitch_motor->out_current += (int16_t)(
                    GIMBAL_VISION_PITCH_FF_GAIN *
                    gimbal_bumpless_state.vision_ff_blend *
                    gimbal_bumpless_state.last_vision_pitch_rate);
            } else if (!want_vision_source) {
                gimbal_bumpless_state.last_vision_yaw_rate = 0.0f;
                gimbal_bumpless_state.last_vision_pitch_rate = 0.0f;
            }

            gimbal_cmd.yaw = gimbal_bumpless_state.active_yaw_target;
            gimbal_cmd.pitch = gimbal_bumpless_state.active_pitch_target;
            Gimbal_pitch_rtt_vofa_print(gimbal_bumpless_state.active_pitch_target);
        }

        gimbal_bumpless_state.last_mode = cmd->gimbal_mode;

        // 把当前真正执行的 active_ref 回传给决策层。
        // 决策层在视觉期间持续回写这个值，退出视觉时手动目标才能无缝接管。
        gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
        gimbal_feedback.yaw_motor_total_angle = yaw_motor->motor_measure.total_angle;
        gimbal_feedback.imu_yaw_total_angle = measured_yaw;
        gimbal_feedback.imu_yaw_rate = gimbal_imu_data->gyro_body.z;
        gimbal_feedback.imu_pitch_angle = measured_pitch;
        gimbal_feedback.active_yaw_target = gimbal_bumpless_state.active_yaw_target;
        gimbal_feedback.active_pitch_target = gimbal_bumpless_state.active_pitch_target;
        xQueueOverwrite(Gimbal_feedback_queue_handle, &gimbal_feedback);
}
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

