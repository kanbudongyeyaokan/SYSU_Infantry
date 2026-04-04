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
#include "algorithm_pid.h"
#include "error_handler.h"
#include "chassis_ramp.h"

#include "power_meter.h"

#include "chassis_scurve.h"

#define CHASSIS_FOLLOW_YAW_GAIN 0.5f
#define CHASSIS_FOLLOW_WZ_LIMIT 200.0f
#define CHASSIS_ROTATE_WZ 750.0f
#define CHASSIS_MOTOR_PID_MAX_OUT 15000.0f
// 云台0位（offset_angle=0）对应的底盘运动学坐标系偏转角（度）2
// 全向轮轴线与底盘前方夹角，需与 robot_definitions.h 中 YAW_CHASSIS_ALIGN_ECD 所对应的物理方向保持一致
// YAW_CHASSIS_ALIGN_ECD=1990 → 0度方向 → CHASSIS_FORWARD_ANGLE=0
// YAW_CHASSIS_ALIGN_ECD=1050 → 45度方向 → CHASSIS_FORWARD_ANGLE=45
#define CHASSIS_FORWARD_ANGLE 0.0f

//底盘斜坡规划步长
#define CHASSIS_RAMP_STEP 6.0f // 斜坡步长，越大越猛，越小越顺滑



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

//底盘跟随云台，用于计算WZ控制量
static Pid_instance_t chassis_follow_pid;

// 声明底盘斜坡控制器实例
// static Chassis_Ramp_t chassis_ramp;

static Chassis_SCurve_t chassis_scurve; // 新增 S 曲线实例


/*********************************底盘方法接口**************************************/
/**
 * @brief 底盘任务初始化
 */
void Chassis_task_init(void) {
    //底盘模块初始化
    Chassis_init();

    // Chassis_Ramp_Init(&chassis_ramp, CHASSIS_RAMP_STEP);
    Chassis_SCurve_Init(&chassis_scurve); // 新增 S 曲线初始化
    //超电初始化
    // SuperCap_Comm_Init(&hcan2);

    //底盘功率控制初始化
    Chassis_Power_Control_Init();
    //SuperCap_Comm_Init(&hcan1);
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

    // 跟随云台速度 PID
    Pid_init_t follow_pid_config = {
        .kp = 13.0f,      // 比例系数，如果跟车太慢就加大，太快发抖就减小
        .ki = 0.0f,     // 通常底盘跟随不需要积分，给 0 即可
        .kd = 0.5f,     // 微分系数，极其重要！给一点 D 项可以提供阻尼，防止底盘到位时来回摆动
        .max_out = 1200.0f,  // 对应原来的 CHASSIS_FOLLOW_WZ_LIMIT
        .max_iout = 200.0f,   // 没用到 I 就不管
        .deadband = 4.0f,   // 死区，误差绝对值小于这个值时不输出，防止底盘一直微调
        .optimization = PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST, // 微分先行：对 measure 微分，避免死区边界 D 暴冲
    };
    Pid_init(&chassis_follow_pid, &follow_pid_config);

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
    PowerMeter_Init(&hcan1);
    Chassis_Power_Control_Init();
}

/**
 * @brief 底盘逻辑更新函数
 * @param cmd 指向接收到的指令结构体
 */
void Chassis_Update_Control(const Chassis_cmd_send_t *cmd)
{
    test_cmd = *cmd;

    // 刚切入跟随模式时，重置 PID 防止突变
    static chassis_mode_e last_chassis_mode = CHASSIS_ZERO_FORCE;
    if (cmd->chassis_mode == CHASSIS_FOLLOW_GIMBAL && last_chassis_mode != CHASSIS_FOLLOW_GIMBAL) {
        Pid_reset(&chassis_follow_pid); // 刚切入跟随模式时，重置 PID 防止突变
    }
    last_chassis_mode = cmd->chassis_mode;

    // 底盘斜坡规划：
    Chassis_cmd_send_t cmd_solved = *cmd;
    if (cmd->chassis_mode == CHASSIS_ZERO_FORCE) {
        // 失能时复位斜坡控制器，防止重使能时车子突然窜出去
        // Chassis_Ramp_Reset(&chassis_ramp);
        //Chassis_SCurve_Reset(&chassis_scurve); // 更换为 S 曲线复位
        cmd_solved.vx = 0.0f;
        cmd_solved.vy = 0.0f;
    } else {
        // 调用库函数进行平滑处理，直接将结果写入 cmd_solved 的 vx 和 vy
        // Chassis_Ramp_Update(&chassis_ramp, cmd->vx, cmd->vy, &cmd_solved.vx, &cmd_solved.vy);
        Chassis_SCurve_Update(&chassis_scurve, cmd->vx, cmd->vy, &cmd_solved.vx, &cmd_solved.vy);
        // cmd_solved.vx = cmd->vx;
        // cmd_solved.vy = cmd->vy;
    }

    switch (cmd_solved.chassis_mode)
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

            // 传入处理过斜坡的 cmd_solved
            Chassis_kinematics_solve(&cmd_solved, &chassis_output);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;

        case CHASSIS_FOLLOW_GIMBAL:
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
            }

            // measure=offset_angle, target=0（消除偏差），取负使 wz 方向正确（追赶方向与偏差方向一致）
            float pid_out = -Pid_calculate(&chassis_follow_pid, cmd_solved.offset_angle, 0.0f);
            float K_ff = 1200.0f;
            cmd_solved.wz = (cmd_solved.cmd_yaw * K_ff) + pid_out;
            // static uint32_t chassis_log_tick = 0;
            // if (xTaskGetTickCount() - chassis_log_tick >= 100) {
            //     chassis_log_tick = xTaskGetTickCount();
            //     ERROR_INFO("CHASSIS_FOLLOW",
            //         "offset=%.2f P=%.1f D=%.1f pid=%.1f wz=%.1f",
            //         cmd_solved.offset_angle,
            //         chassis_follow_pid.Pout,
            //         chassis_follow_pid.Dout,
            //         pid_out,
            //         cmd_solved.wz);
            // }
            
            // 矢量变换：将遥控器速度指令从云台坐标系转换到底盘坐标系
            // -offset_angle: 补偿云台当前偏转角
            // -CHASSIS_FORWARD_ANGLE: 补偿云台0位与底盘运动学轴线的固定夹角
            float theta = (-cmd_solved.offset_angle - CHASSIS_FORWARD_ANGLE) * (M_PI / 180.0f);
            float cos_theta = arm_cos_f32(theta);
            float sin_theta = arm_sin_f32(theta);

            // 必须使用临时变量，否则算出新的 vx 会污染后续 vy 的计算
            float temp_vx = cmd_solved.vx * cos_theta - cmd_solved.vy * sin_theta;
            float temp_vy = cmd_solved.vx * sin_theta + cmd_solved.vy * cos_theta;
            cmd_solved.vx = temp_vx;
            cmd_solved.vy = temp_vy;

            // 传入处理好的 cmd_solved
            Chassis_kinematics_solve(&cmd_solved, &chassis_output);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;
        }

        case CHASSIS_ROTATE:
        {
            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_status(chassis_motors[i], MOTOR_ENABLED);
            }

            // 直接操作 cmd_solved
            cmd_solved.wz = CHASSIS_ROTATE_WZ;

            float angle_error = (cmd_solved.offset_angle - CHASSIS_FORWARD_ANGLE);
            float cos_theta = arm_cos_f32(angle_error * MATH_DEG2RAD);
            float sin_theta = arm_sin_f32(angle_error * MATH_DEG2RAD);

            float temp_vx = cmd_solved.vx * cos_theta - cmd_solved.vy * sin_theta;
            float temp_vy = cmd_solved.vx * sin_theta + cmd_solved.vy * cos_theta;
            cmd_solved.vx = temp_vx;
            cmd_solved.vy = temp_vy;
            Chassis_kinematics_solve(&cmd_solved, &chassis_output);

            for (uint8_t i = 0; i < 4; i++) {
                Djimotor_set_target(chassis_motors[i], chassis_output.motor_speed[i]);
                Djimotor_Calc_Output(chassis_motors[i]);
            }
            break;
        }
        default:
            break;
    }

    // 在 1kHz 控制周期末统一做动态功率控制与等比例电流限幅
    float power = PowerMeter_GetPower();
    //float power = SuperCap_Get_Chassis_Power();
    //从超级电容模块获取当前功率
    //去除注释时记得去chassis_init那里初始化supercap
    if (PowerMeter_IsOnline()) {
        Chassis_Power_Control(chassis_motors, power);
    } else {
        Chassis_Power_Control(chassis_motors, -1);
        //-1 代表功率计无效
    }


    // 反馈底盘数据回决策层
    chassis_feedback.chassis_wz = cmd_solved.wz;
    xQueueOverwrite(Chassis_feedback_queue_handle, &chassis_feedback);
}

/**
 * @brief 全向轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_omni_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output) {
    //目前以电池所在位置为后方，其对面为正前方

    // === 速度归一化：保证各方向最大速度一致 ===
    // 问题：斜向移动时 vx + vy 会叠加，导致斜向比正向快
    // 解决：当合速度超过正向最大速度时，按比例缩放
    float vx = cmd->vx;
    float vy = cmd->vy;

    float speed_magnitude = sqrtf(vx * vx + vy * vy);
    float max_component = fmaxf(fabsf(vx), fabsf(vy));

    if (speed_magnitude > max_component && speed_magnitude > 0.001f) { // 避免接近零时除法溢出
        float scale = max_component / speed_magnitude;
        vx *= scale;
        vy *= scale;
    }

    // 计算各轮子线速度 (rad/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] = -vx - vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 右前轮
    wheel_linear_speed[1] = -vx + vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 左后轮
    wheel_linear_speed[2] = vx + vy
                            - cmd->wz * (chassis_params.half_wheel_base + chassis_params.half_track_width) *
                            MATH_DEG2RAD; // 左前轮
    wheel_linear_speed[3] = vx - vy
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
