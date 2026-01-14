/**
 * @file    gimbal.c
 * @brief   云台功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 2.0
 *
 * @note    云台电机初始化 + 上电归中状态机
 */

#include "cmsis_os.h"
#include <stdbool.h>
#include <math.h>

#include "gimbal.h"

#include <stdio.h>
#include <main.h>

#include "dji_motor.h"
#include "decision_making.h"
#include "message_center.h"
#include "ins.h"
#include "robot_definitions.h"

// ==================== 归中参数配置 ====================
#define ZEROING_THRESHOLD       5.0f   // 归中误差阈值（度）
#define ZEROING_STABLE_COUNT    30   // 连续满足条件的次数
#define ZEROING_MAX_SPEED       200.0f  // 归中时最大速度限制

// ==================== 云台状态机 ====================
static Gimbal_state_e gimbal_state = GIMBAL_STATE_INIT;
static uint32_t zeroing_stable_counter = 0;
static float zeroing_target_angle = 0.0f;  // 归中目标（编码器单圈角度）

// ==================== 云台电机 ====================
static Djimotor_device_t *yaw_motor, *pitch_motor;

// ==================== 调试变量（标定用）====================
// 在调试器中查看此变量以获取当前 Yaw 编码器值
// 将云台对齐底盘后读取此值，填入 YAW_CHASSIS_ALIGN_ECD
volatile uint16_t g_yaw_ecd_debug = 0;

// 云台模块的姿态数据指针，指向ins模块的全局变量
static attitude_t *gimbal_imu_data;

// 订阅决策层发来的云台控制指令
static Subscriber_t *gimbal_sub;
// 存储决策层发来的控制命令
static Gimbal_cmd_send_t gimbal_cmd_recv;

// 发布给决策层的云台反馈信息
static Publisher_t *gimbal_pub;
// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

// ==================== 内部函数声明 ====================
static void Gimbal_motor_init(void);
static void Gimbal_switch_to_encoder_feedback(void);
static void Gimbal_switch_to_imu_feedback(void);
static float Gimbal_calc_shortest_path_error(float target, float current);

/**
 * @brief 获取当前云台状态
 */
Gimbal_state_e Gimbal_get_state(void) {
    return gimbal_state;
}

/**
 * @brief 云台电机初始化 - 初始使用编码器反馈用于归中
 */
static void Gimbal_motor_init(void) {
    // YAW电机 - 初始化时使用编码器反馈用于归中
    Djimotor_init_config_t yaw_config = {
        .motor_name = "yaw_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_STOP,  // 初始停止，等待 IMU Ready
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = MOTOR_FEEDBACK,  // 初始使用编码器反馈
            .speed_source = MOTOR_FEEDBACK,
            .angle_pid = {
                .kp = 15,
                .ki = 0.0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = ZEROING_MAX_SPEED,  // 归中时限速
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
            },
            .speed_pid = {
                .kp = 20,
                .ki = 0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = 28000,
                .max_iout = 8000,
                .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
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

    // PITCH电机 - 直接使用 IMU 反馈（Pitch 不需要归中）
    Djimotor_init_config_t pitch_config = {
        .motor_name = "pitch_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_STOP,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler_angles.pitch),
            .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_raw.pitch),
            .angle_pid = {
                .kp = 20,
                .ki = 0,
                .kd = 0.0,
                .max_out = 100,
                .max_iout = 100,
                .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
            },
            .speed_pid = {
                .kp = 20,
                .ki = 3.0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = 10000,
                .max_iout = 800,
                .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
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

    // 计算归中目标角度
    zeroing_target_angle = YAW_ALIGN_ANGLE;
}

/**
 * @brief 切换 YAW 电机到编码器反馈模式（用于归中）
 */
static void Gimbal_switch_to_encoder_feedback(void) {
    Djimotor_controller_init_t encoder_ctrl = {
        .close_loop = ANGLE_AND_SPEED_LOOP,
        .angle_source = MOTOR_FEEDBACK,
        .speed_source = MOTOR_FEEDBACK,
        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,
        .angle_pid = {
            .kp = 15,
            .ki = 0.0,
            .kd = 0.0,
            .deadband = 0.1f,
            .max_out = ZEROING_MAX_SPEED,
            .max_iout = 100,
            .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
        },
        .speed_pid = {
            .kp = 20,
            .ki = 0,
            .kd = 0.0,
            .deadband = 0.1f,
            .max_out = 28000,
            .max_iout = 8000,
            .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
        },
    };
    Djimotor_change_controller(yaw_motor, encoder_ctrl);
}

/**
 * @brief 切换 YAW 电机到 IMU 反馈模式（用于正常控制）
 */
static void Gimbal_switch_to_imu_feedback(void) {
    Djimotor_controller_init_t imu_ctrl = {
        .close_loop = ANGLE_AND_SPEED_LOOP,
        .angle_source = OTHER_FEEDBACK,
        .speed_source = OTHER_FEEDBACK,
        .other_angle_feedback_ptr = &(gimbal_imu_data->yaw_total_angle),
        .other_speed_feedback_ptr = &(gimbal_imu_data->yaw_rate_dps),
        .angle_pid = {
            .kp = 20,
            .ki = 0.0,
            .kd = 0.0,
            .deadband = 0.1f,
            .max_out = 500,
            .max_iout = 100,
            .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_FEEDFOWARD | PID_DIFFERENTIAL_GO_FIRST,
        },
        .speed_pid = {
            .kp = 20,
            .ki = 0,
            .kd = 0.0,
            .deadband = 0.1f,
            .max_out = 28000,
            .max_iout = 8000,
            .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_FEEDFOWARD | PID_DIFFERENTIAL_GO_FIRST,
        },
    };
    Djimotor_change_controller(yaw_motor, imu_ctrl);
}

/**
 * @brief 计算最短路径误差（用于单圈编码器归中）
 * @param target 目标角度 (0~360)
 * @param current 当前角度 (0~360)
 * @return 误差 (-180~+180)
 */
static float Gimbal_calc_shortest_path_error(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

/**
 * @brief 云台任务初始化
 */
void Gimbal_task_init(void) {
    // 获取 INS 模块的姿态数据指针
    gimbal_imu_data = get_attitude_data();

    // 初始化云台电机
    Gimbal_motor_init();

    // 订阅决策层发来的控制指令
    gimbal_sub = Sub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));

    // 注册云台反馈信息发布者
    gimbal_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));

    // 初始状态
    gimbal_state = GIMBAL_STATE_INIT;
    zeroing_stable_counter = 0;
}

/**
 * @brief 处理云台控制指令 - 包含状态机
 */
void Gimbal_handle_command(void) {
    Imu_state_e imu_state = ins_get_state();
    float current_encoder_angle = yaw_motor->motor_measure.current_angle;
    float error;

    // ==================== 状态机处理 ====================
    switch (gimbal_state) {
        // -------------------- 初始化状态 --------------------
        case GIMBAL_STATE_INIT:
            // 等待 IMU Ready
            Djimotor_set_status(yaw_motor, MOTOR_STOP);
            Djimotor_set_status(pitch_motor, MOTOR_STOP);

            if (imu_state == IMU_STATE_READY) {
                // IMU 就绪，进入归中状态
                gimbal_state = GIMBAL_STATE_ZEROING;
                zeroing_stable_counter = 0;
                
                // 确保使用编码器反馈
                Gimbal_switch_to_encoder_feedback();
                
                printf("[Gimbal] IMU Ready, start zeroing. Target: %.2f\n", zeroing_target_angle);
            }
            break;

        // -------------------- 归中状态 --------------------
        case GIMBAL_STATE_ZEROING:
            // IMU 掉线则回到初始化状态
            if (imu_state != IMU_STATE_READY) {
                gimbal_state = GIMBAL_STATE_INIT;
                Djimotor_set_status(yaw_motor, MOTOR_STOP);
                break;
            }

            // 使能电机
            Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
            Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

            // 计算最短路径误差（使用单圈角度避免 0°/360° 跳变）
            error = Gimbal_calc_shortest_path_error(zeroing_target_angle, current_encoder_angle);

            // 设置目标：当前多圈位置 + 最短路径误差
            // 误差用单圈角度计算（找最短路径），目标用多圈坐标（匹配 PID 反馈源）
            Djimotor_set_target(yaw_motor, yaw_motor->motor_measure.total_angle + error);
            Djimotor_set_target(pitch_motor, 0);  // Pitch 归零到水平

            // 检查是否归中完成
            if (fabsf(error) < ZEROING_THRESHOLD) {
                zeroing_stable_counter++;
                if (zeroing_stable_counter >= ZEROING_STABLE_COUNT) {
                    // 归中完成，切换到 IMU 反馈
                    Gimbal_switch_to_imu_feedback();
                    
                    // 立即设置目标为当前 IMU 值，防止切换瞬间抖动
                    Djimotor_set_target(yaw_motor, gimbal_imu_data->yaw_total_angle);
                    Djimotor_set_target(pitch_motor, gimbal_imu_data->euler_angles.pitch);
                    
                    gimbal_state = GIMBAL_STATE_READY;
                }
            } else {
                zeroing_stable_counter = 0;
            }
            break;

        // -------------------- 就绪状态（正常控制）--------------------
        case GIMBAL_STATE_READY:
            // IMU 掉线则回到初始化状态
            if (imu_state != IMU_STATE_READY) {
                gimbal_state = GIMBAL_STATE_INIT;
                Djimotor_set_status(yaw_motor, MOTOR_STOP);
                Djimotor_set_status(pitch_motor, MOTOR_STOP);
                break;
            }

            // 确保电机始终使能
            Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
            Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

            // 从消息中心获取最新的控制指令
            if (Sub_get_message(gimbal_sub, (void *)(&gimbal_cmd_recv))) {
                switch (gimbal_cmd_recv.gimbal_mode) {
                    case GIMBAL_ZERO_FORCE:
                        Djimotor_set_status(yaw_motor, MOTOR_STOP);
                        Djimotor_set_status(pitch_motor, MOTOR_STOP);
                        break;

                    case GIMBAL_GYRO_MODE:
                        Djimotor_set_target(yaw_motor, gimbal_cmd_recv.yaw);
                        Djimotor_set_target(pitch_motor, gimbal_cmd_recv.pitch);
                        break;

                    case GIMBAL_VISION_MODE:
                        // TODO: 视觉模式
                        break;

                    default:
                        break;
                }
            }
            break;

        default:
            gimbal_state = GIMBAL_STATE_INIT;
            break;
    }

    // ==================== 更新反馈数据 ====================
    gimbal_feedback.yaw_motor_single_round_angle = current_encoder_angle;
    gimbal_feedback.yaw_motor_total_angle = yaw_motor->motor_measure.total_angle;
    gimbal_feedback.imu_yaw_total_angle = gimbal_imu_data->yaw_total_angle;
    gimbal_feedback.imu_yaw_rate = gimbal_imu_data->yaw_rate_dps;
    gimbal_feedback.imu_state = imu_state;
    
    // 更新调试变量（供标定 YAW_CHASSIS_ALIGN_ECD 使用）
    g_yaw_ecd_debug = yaw_motor->motor_measure.current_ecd;

    // 推送消息
    if (gimbal_pub != NULL) {
        Pub_push_message(gimbal_pub, (void *)&gimbal_feedback);
    }
}
