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
#include "error_handler.h"
#include "message_center.h"
#include "decision_making.h"
#include "robot_task.h"
// 引入FreeRTOS头文件以使用 xTaskGetTickCount()
#include "FreeRTOS.h"
#include "task.h"

#define ONE_BULLET_DELTA_ANGLE  36.0f    // 单发子弹拨弹盘转动角度 (10孔盘为36度)
#define REDUCTION_RATIO_LOADER  36.0f    // M2006电机减速比
#define SINGLE_SHOOT_INTERVAL_MS 200     // 单发连续触发的时间间隔(ms)，200ms = 5Hz点射

/****************接收决策层的射击控制信息********************/
// 存储决策层发来的控制命令
static Shoot_cmd_send_t shoot_cmd_recv;
static float M2006_last_angle;
static loader_mode_e loader_last_mode;

// 单发逻辑使用的时间戳和绝对角度记录
static uint32_t last_single_shoot_time = 0;   
static float loader_target_angle = 0.0f;      

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
                    .kp = 10,   // 10
                    .ki = 2, // 1
                    .kd = 0,
                    .max_iout = 3000,
                    .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                    .max_out = 10000,
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
        Djimotor_set_target(shoot_motors[0], 6700);  // 上摩擦轮
        Djimotor_set_target(shoot_motors[1], -6700); // 下摩擦轮
    }

    // 2. 处理拨弹盘 (LOADER_MODE)
    switch (cmd->loader_mode)
    {
        case LOAD_STOP:
            shoot_motors[2]->motor_pid.close_loop = SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            Djimotor_set_target(shoot_motors[2], 0);
            
            // 停转时同步目标角度，防止下次切入位置环时疯转
            loader_target_angle = shoot_motors[2]->motor_measure.current_angle;
            break;

        case LOAD_1_BULLET: // 连续单发（固定频率点射）
            shoot_motors[2]->motor_pid.close_loop = ANGLE_AND_SPEED_LOOP; 
            Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);

            // 刚切入单发模式时，同步当前角度防止突变
            if (loader_last_mode != LOAD_1_BULLET) {
                loader_target_angle = shoot_motors[2]->motor_measure.current_angle;
            }

            // 非阻塞式时间戳判定 (替代osDelay)
            uint32_t current_time = xTaskGetTickCount(); 
            if (current_time - last_single_shoot_time >= SINGLE_SHOOT_INTERVAL_MS) {
                // 累加一发子弹的角度（考虑减速比）
                // 注意：由于M2006旋转方向问题，这里用减法还是加法取决于你的机械安装，如果不转或反转请改为 +=
                loader_target_angle -= ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER; 
                last_single_shoot_time = current_time; 
            }
            
            Djimotor_set_target(shoot_motors[2], loader_target_angle);
            break;

        case LOAD_BURSTFIRE: // 连发
            shoot_motors[2]->motor_pid.close_loop = SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
            
            // 速度环单位换算: cmd->shoot_rate (发/秒) -> 目标RPM
            // RPM = 发频 * 60 / (360/单发角度) * 减速比
            float target_rpm = cmd->shoot_rate * 60.0f / (360.0f / ONE_BULLET_DELTA_ANGLE) * REDUCTION_RATIO_LOADER;
            
            // 同理，方向视机械安装而定，可能需要去掉负号
            Djimotor_set_target(shoot_motors[2], -target_rpm);
            
            // 在连发时同步角度变量，保证切回单发或停止时的平滑度
            loader_target_angle = shoot_motors[2]->motor_measure.current_angle;
            break;

        default:
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            break;
    }

    // 更新上次模式
    loader_last_mode = cmd->loader_mode;

    // 算发分离：计算 PID 输出
    // 摩擦轮
    Djimotor_Calc_Output(shoot_motors[0]);
    Djimotor_Calc_Output(shoot_motors[1]);
    // 拨弹盘
    Djimotor_Calc_Output(shoot_motors[2]);

    // 其他反馈赋值
    xQueueOverwrite(Shoot_feedback_queue_handle, &shoot_feedback);
}