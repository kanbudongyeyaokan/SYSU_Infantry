/**
 * @file    shoot.c
 * @brief   发射机构功能模块源文件
 * @author  SYSU电控组 (Optimized)
 * @date    2025-09-27
 * @version 1.1
 *
 * @note    发射机构电机初始化与控制逻辑（含非阻塞单发与连发修正）
 */

#include "shoot.h"
#include "bsp_can.h"
#include "message_center.h"
#include "decision_making.h"
#include "robot_task.h"
// 引入FreeRTOS头文件以使用 xTaskGetTickCount()
#include "error_handler.h"
#include "FreeRTOS.h"
#include "task.h"

#define ONE_BULLET_DELTA_ANGLE    36.0f  // 单发子弹拨弹盘转动角度 (10孔盘为36度)
#define REDUCTION_RATIO_LOADER    36.0f  // M2006电机减速比
#define SINGLE_SHOOT_INTERVAL_MS  200    // 单发连续触发的时间间隔(ms)，200ms = 5Hz点射
#define FRICTION_WHEEL_TARGET_RPM 6700   // 摩擦轮目标转速(RPM)，对应弹速约24.9m/s

// 堵转检测参数
#define JAM_DETECT_TIME_MS        250   // 堵转判定时间(ms) - 0.5秒
#define JAM_CURRENT_THRESHOLD     8000  // 堵转电流阈值(mA)
#define JAM_CURRENT_SAMPLES       10     // 电流采样次数
#define REVERSE_ANGLE             108.0f  // 反转角度(3颗弹) - 增大冲程

/****************接收决策层的射击控制信息********************/
// 存储决策层发来的控制命令
static Shoot_cmd_send_t shoot_cmd_recv;
static float M2006_last_angle;
static loader_mode_e loader_last_mode;

// 单发逻辑使用的时间戳和绝对角度记录
static uint32_t last_single_shoot_time = 0;
static float loader_target_angle = 0.0f;

// 堵转检测变量
static uint32_t jam_detect_start_time = 0;
static float jam_detect_start_angle = 0.0f;
static bool is_reversing = false;
static int16_t current_samples[JAM_CURRENT_SAMPLES] = {0};
static uint8_t current_sample_index = 0;

/****************发送给决策层的射击反馈信息******************/
// 发布给决策层的射击反馈信息
static Publisher_t *shoot_feedback_pub;
// 存储发送给决策层的反馈信息
static Shoot_feedback_info_t shoot_feedback;

extern QueueHandle_t Shoot_feedback_queue_handle;   //发射机构反馈信息队列句柄

Shoot_cmd_send_t shoot_test_cmd;

/****************发射机构电机实例**************************/
static Djimotor_device_t *shoot_motors[3] = {0};
Djimotor_device_t shoot_test_motor;

void Shoot_motors_init(void)
{
    static Djimotor_init_config_t cfg[3] = {
        {
            .motor_name = "FRICTION_L",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 22, // 20
                    .ki = 1,  // 1
                    .kd = 0.1,
                    .max_iout = 2000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = 20000,
                },
            },
            .can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
        },
        {
            .motor_name = "FRICTION_R",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
                .speed_source = MOTOR_FEEDBACK,
                .speed_pid = {
                    .kp = 22, // 20
                    .ki = 1,  // 1
                    .kd = 0.1,
                    .max_iout = 2000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = 20000,
                },
            },
            .can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
        },
        {
            .motor_name = "LOADER",
            .motor_type = M2006,
            .motor_status = MOTOR_STOP,
            .deadzone_compensation = 100,
            .motor_controller_init = {
                .close_loop = ANGLE_AND_SPEED_LOOP,
                .angle_source = MOTOR_FEEDBACK,
                .speed_source = MOTOR_FEEDBACK,
                .angle_pid = {
                    .kp = 8,   // 10
                    .ki = 1, // 1
                    .kd = 0,
                    .max_iout = 1000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = 2000,
                },
                .speed_pid = {
                    .kp = 20,   // 10 -> 20 (提高响应)
                    .ki = 5,    // 2 -> 5 (增强积分)
                    .kd = 0,
                    .max_iout = 6000,  // 3000 -> 6000
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = 20000,  // 10000 -> 20000 (M2006最大输出)
                },
            },
            .can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
        }
    };

    for (int i = 0; i < 3; i++) {
        shoot_motors[i] = DJI_Motor_Init(&cfg[i]);
    }
    shoot_cmd_recv.loader_mode = LOAD_STOP;
    loader_last_mode = LOAD_STOP;
    shoot_cmd_recv.shoot_mode = SHOOT_OFF;
}

//发射任务初始化
void Shoot_task_init(void) {
    Shoot_motors_init();
}

//发射任务处理控制命令
void Shoot_handle_command(Shoot_cmd_send_t *cmd) {
    shoot_test_cmd = *cmd;
    uint32_t current_time;
    uint32_t burst_interval_ms;

    // 处理摩擦轮 (SHOOT_MODE)
    if (cmd->shoot_mode == SHOOT_OFF) {
        // 关闭摩擦轮
        Djimotor_set_status(shoot_motors[0], MOTOR_STOP);
        Djimotor_set_status(shoot_motors[1], MOTOR_STOP);
        Djimotor_set_target(shoot_motors[0], 0);
        Djimotor_set_target(shoot_motors[1], 0);
    }
    else {
        // 开启摩擦轮
        Djimotor_set_status(shoot_motors[0], MOTOR_ENABLED);
        Djimotor_set_status(shoot_motors[1], MOTOR_ENABLED);
        Djimotor_set_target(shoot_motors[0], FRICTION_WHEEL_TARGET_RPM);
        Djimotor_set_target(shoot_motors[1], -FRICTION_WHEEL_TARGET_RPM);
    }

    // 2. 处理拨弹盘 (LOADER_MODE)
    switch (cmd->loader_mode)
    {
        case LOAD_STOP:
            shoot_motors[2]->motor_pid.close_loop = SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            Djimotor_set_target(shoot_motors[2], 0);
            break;

        case LOAD_1_BULLET:
            shoot_motors[2]->motor_pid.close_loop = ANGLE_AND_SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);

            if (loader_last_mode != LOAD_1_BULLET) {
                loader_target_angle = shoot_motors[2]->motor_measure.current_angle;
            }

            current_time = xTaskGetTickCount();
            if (current_time - last_single_shoot_time >= SINGLE_SHOOT_INTERVAL_MS) {
                loader_target_angle -= ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
                last_single_shoot_time = current_time;
            }

            // 防止目标角度超前实际角度超过1颗子弹，避免阻力消除后暴射
            {
                float max_backlog = ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * 1.2f;
                float actual_angle = shoot_motors[2]->motor_measure.total_angle;
                if ((actual_angle - loader_target_angle) > max_backlog) {
                    loader_target_angle = actual_angle - max_backlog;
                }
            }
            Djimotor_set_target(shoot_motors[2], loader_target_angle);
            break;

        case LOAD_BURSTFIRE:
            shoot_motors[2]->motor_pid.close_loop = ANGLE_AND_SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);

            // 刚切入连发模式时初始化堵转检测
            if (loader_last_mode != LOAD_BURSTFIRE) {
                jam_detect_start_time = xTaskGetTickCount();
                jam_detect_start_angle = shoot_motors[2]->motor_measure.total_angle;
                loader_target_angle = shoot_motors[2]->motor_measure.total_angle;
                is_reversing = false;
            }
            if (is_reversing) {
                // ERROR_INFO("SHOOT", "reversing... target=%.1f, total=%.1f",
                //     loader_target_angle, shoot_motors[2]->motor_measure.total_angle);
                // 反转完成检测：使用多圈角度比较
                float reverse_error = loader_target_angle - shoot_motors[2]->motor_measure.total_angle;
                if (reverse_error < 0) reverse_error = -reverse_error;
                if (reverse_error < 40.0f * REDUCTION_RATIO_LOADER) {  // 10° -> 30°，不等完全到位
                    is_reversing = false;
                    ERROR_INFO("SHOOT", "reverse complete!");
                    loader_target_angle = shoot_motors[2]->motor_measure.total_angle;
                    jam_detect_start_time = xTaskGetTickCount();
                    last_single_shoot_time = xTaskGetTickCount() + 100;  // 延迟100ms再继续供弹
                }
            } else {
                // 正常供弹逻辑
                burst_interval_ms = (cmd->shoot_rate > 0) ? (1000 / cmd->shoot_rate) : 125;
                current_time = xTaskGetTickCount();
                if (current_time - last_single_shoot_time >= burst_interval_ms) {
                    loader_target_angle -= ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
                    last_single_shoot_time = current_time;
                }

                // 堵转检测：采集电流并计算平均值
                int16_t current_mA = shoot_motors[2]->motor_measure.real_current;
                if (current_mA < 0) current_mA = -current_mA;

                // 更新电流采样缓冲区
                current_samples[current_sample_index] = current_mA;
                current_sample_index = (current_sample_index + 1) % JAM_CURRENT_SAMPLES;

                // 计算平均电流
                int32_t current_sum = 0;
                for (int i = 0; i < JAM_CURRENT_SAMPLES; i++) {
                    current_sum += current_samples[i];
                }
                int16_t current_avg = current_sum / JAM_CURRENT_SAMPLES;

                ERROR_INFO("SHOOT", "avg=%d, thresh=%d, time_diff=%d, detect_time=%d",
                    current_avg, JAM_CURRENT_THRESHOLD,
                    (int)(current_time - jam_detect_start_time), JAM_DETECT_TIME_MS);

                if (current_avg > JAM_CURRENT_THRESHOLD) {
                    ERROR_INFO("SHOOT", "current over threshold!");
                    if (current_time - jam_detect_start_time >= JAM_DETECT_TIME_MS) {
                        ERROR_INFO("SHOOT","jam detected! avg_current=%d", current_avg);
                        is_reversing = true;
                        loader_target_angle = shoot_motors[2]->motor_measure.total_angle + REVERSE_ANGLE * REDUCTION_RATIO_LOADER;
                    }
                } else {
                    jam_detect_start_time = current_time;
                }
            }

            // 防止目标角度超前实际角度超过1颗子弹，避免阻力消除后暴射
            if (!is_reversing) {
                float max_backlog = ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * 1.2f;
                float actual_angle = shoot_motors[2]->motor_measure.total_angle;
                if ((actual_angle - loader_target_angle) > max_backlog) {
                    loader_target_angle = actual_angle - max_backlog;
                }
            }

            Djimotor_set_target(shoot_motors[2], loader_target_angle);
            break;

        default:
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            break;
    }

    loader_last_mode = cmd->loader_mode;

    Djimotor_Calc_Output(shoot_motors[0]);
    Djimotor_Calc_Output(shoot_motors[1]);
    Djimotor_Calc_Output(shoot_motors[2]);

    // 打印拨弹电机电流
    //ERROR_INFO("SHOOT", "loader_current=%d", shoot_motors[2]->motor_measure.real_current);

    xQueueOverwrite(Shoot_feedback_queue_handle, &shoot_feedback);
}