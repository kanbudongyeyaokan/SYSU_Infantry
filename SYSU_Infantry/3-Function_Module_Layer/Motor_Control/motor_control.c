/**
 * @file    motor_control.c
 * @brief   电机控制模块源文件
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    实现电机控制所需的功能
 */

#include "motor_control.h"
#include <stdio.h>
#include "decision_making.h"

// 定义底盘电机 - 声明为全局变量供内联函数使用
Djimotor_device_t *chassis_motors[4]; // 4个底盘电机
Djimotor_device_t *gimbal_motors[2];  // 2个云台电机(yaw和pitch)
Djimotor_device_t *shoot_motors[3];   // 3个发射机构电机(2个摩擦轮+1个拨弹)

// 订阅决策层发来的控制指令
static Subscriber_t *chassis_cmd_sub;
static Subscriber_t *gimbal_cmd_sub;
static Subscriber_t *shoot_cmd_sub;

// 存储决策层发来的控制命令
static Chassis_cmd_send_t chassis_cmd;
static Gimbal_cmd_send_t gimbal_cmd;
static Shoot_cmd_send_t shoot_cmd;

// 发布给决策层的反馈信息
static Publisher_t *chassis_feedback_pub;
static Publisher_t *gimbal_feedback_pub;
static Publisher_t *shoot_feedback_pub;

// 存储发送给决策层的反馈信息
static Chassis_feedback_info_t chassis_feedback;
static Gimbal_feedback_info_t gimbal_feedback;
static Shoot_feedback_info_t shoot_feedback;

/**
 * @brief 电机初始化函数
 */
static void Motor_init(void)
{
    // 初始化底盘电机
    Djimotor_init_config_t chassis_config[4] = {
        {
            .motor_name = "CHASSIS_FR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan1,
                .can_id = 0x200,
                .tx_id = 1,
                .rx_id = 0x201
            }
        },
        {
            .motor_name = "CHASSIS_FL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan1,
                .can_id = 0x200,
                .tx_id = 2,
                .rx_id = 0x202
            }
        },
        {
            .motor_name = "CHASSIS_BL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan1,
                .can_id = 0x200,
                .tx_id = 3,
                .rx_id = 0x203
            }
        },
        {
            .motor_name = "CHASSIS_BR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan1,
                .can_id = 0x200,
                .tx_id = 4,
                .rx_id = 0x204
            }
        }
    };

    // 初始化云台电机
    Djimotor_init_config_t gimbal_config[2] = {
        {
            .motor_name = "GIMBAL_YAW",
            .motor_type = GM6020,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = ANGLE_LOOP,
            },
            .can_init = {
                .can_handle = &hcan2,
                .can_id = 0x1FF,
                .tx_id = 1,
                .rx_id = 0x205
            }
        },
        {
            .motor_name = "GIMBAL_PITCH",
            .motor_type = GM6020,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = ANGLE_LOOP,
            },
            .can_init = {
                .can_handle = &hcan2,
                .can_id = 0x1FF,
                .tx_id = 2,
                .rx_id = 0x206
            }
        }
    };

    // 初始化发射机构电机
    Djimotor_init_config_t shoot_config[3] = {
        {
            .motor_name = "FRICTION_L",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan2,
                .can_id = 0x200,
                .tx_id = 3,
                .rx_id = 0x207
            }
        },
        {
            .motor_name = "FRICTION_R",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = SPEED_LOOP,
            },
            .can_init = {
                .can_handle = &hcan2,
                .can_id = 0x200,
                .tx_id = 4,
                .rx_id = 0x208
            }
        },
        {
            .motor_name = "LOADER",
            .motor_type = M2006,
            .motor_status = MOTOR_ENABLED,
            .motor_controller_init = {
                .close_loop = ANGLE_LOOP,
            },
            .can_init = {
                .can_handle = &hcan2,
                .can_id = 0x1FF,
                .tx_id = 3,
                .rx_id = 0x207
            }
        }
    };

    // 初始化底盘电机
    for (int i = 0; i < 4; i++) {
        chassis_motors[i] = DJI_Motor_Init(&chassis_config[i]);
    }

    // 初始化云台电机
    for (int i = 0; i < 2; i++) {
        gimbal_motors[i] = DJI_Motor_Init(&gimbal_config[i]);
    }

    // 初始化发射机构电机
    for (int i = 0; i < 3; i++) {
        shoot_motors[i] = DJI_Motor_Init(&shoot_config[i]);
    }

    // 设置电机场景配置
    // 示例：为底盘电机设置不同场景的PID参数
    for (int i = 0; i < 4; i++) {
        // 场景1：键盘鼠标模式
        Djimotor_scene_config_t keyboard_mouse_config = {
            .close_loop = SPEED_LOOP,
            .speed_pid = {
                .kp = 15.0f,
                .ki = 0.5f,
                .kd = 0.0f,
                .max_out = 16000.0f,
                .max_iout = 5000.0f,
                .deadband = 10.0f,
                .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT,
                .LPF_coefficient = 0.85f,
                .feedfoward_coefficient = 0.0f
            }
        };
        Djimotor_update_scene_config(chassis_motors[i], SCENE_KEYBOARD_MOUSE, &keyboard_mouse_config);

        // // 场景2：遥控器控制模式
        // Djimotor_scene_config_t remote_control_config = {
        //     .close_loop = SPEED_LOOP,
        //     .speed_pid = {
        //         .kp = 10.0f,
        //         .ki = 0.3f,
        //         .kd = 0.0f,
        //         .max_out = 10000.0f,
        //         .max_iout = 3000.0f,
        //         .deadband = 10.0f,
        //         .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT,
        //         .LPF_coefficient = 0.85f,
        //         .feedfoward_coefficient = 0.0f
        //     }
        // };
        // Djimotor_update_scene_config(chassis_motors[i], SCENE_REMOTE_CONTROL, &remote_control_config);

        // // 场景3：小陀螺模式
        // Djimotor_scene_config_t spinning_top_config = {
        //     .close_loop = SPEED_LOOP,
        //     .speed_pid = {
        //         .kp = 20.0f,
        //         .ki = 0.8f,
        //         .kd = 0.1f,
        //         .max_out = 16000.0f,
        //         .max_iout = 8000.0f,
        //         .deadband = 5.0f,
        //         .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT,
        //         .LPF_coefficient = 0.7f,
        //         .feedfoward_coefficient = 0.0f
        //     }
        // };
        // Djimotor_update_scene_config(chassis_motors[i], SCENE_SPINNING_TOP, &spinning_top_config);

        // // 场景4：底盘跟随云台模式
        // Djimotor_scene_config_t follow_gimbal_config = {
        //     .close_loop = SPEED_LOOP,
        //     .speed_pid = {
        //         .kp = 18.0f,
        //         .ki = 0.6f,
        //         .kd = 0.05f,
        //         .max_out = 16000.0f,
        //         .max_iout = 6000.0f,
        //         .deadband = 8.0f,
        //         .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT,
        //         .LPF_coefficient = 0.8f,
        //         .feedfoward_coefficient = 0.0f
        //     }
        // };
        // Djimotor_update_scene_config(chassis_motors[i], SCENE_FOLLOW_GIMBAL, &follow_gimbal_config);
    }

    // 类似地，可以为云台和发射机构电机设置不同场景的PID参数
}

/**
 * @brief 消息中心初始化
 */
static void Message_init(void)
{
    // 订阅决策层发来的控制指令
    chassis_cmd_sub = Sub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
    gimbal_cmd_sub = Sub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));
    shoot_cmd_sub = Sub_register("shoot_cmd", sizeof(Shoot_cmd_send_t));

    // 注册反馈信息发布者
    chassis_feedback_pub = Pub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));
    gimbal_feedback_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));
    shoot_feedback_pub = Pub_register("shoot_feedback", sizeof(Shoot_feedback_info_t));
}

/**
 * @brief 电机控制模块初始化
 * @note  初始化电机设备和消息中心通信
 */
void Motor_control_init(void)
{
    Motor_init();
    Message_init();
}

/**
 * @brief 处理底盘控制指令
 * @note  订阅底盘控制指令并控制底盘电机
 */
void Motor_control_handle_chassis_cmd(void)
{
    // 从消息中心获取最新指令
    if (Sub_get_message(chassis_cmd_sub, &chassis_cmd)) {
        
        // 根据控制模式切换电机场景
        Djimotor_scene_e scene;
        switch (chassis_cmd.chassis_mode) {
            case CHASSIS_FOLLOW_GIMBAL:
                scene = SCENE_FOLLOW_GIMBAL;
                break;
            case CHASSIS_ROTATE:
                scene = SCENE_SPINNING_TOP;
                break;
            case CHASSIS_NO_FOLLOW:
                scene = SCENE_KEYBOARD_MOUSE;  // 或其他合适的场景
                break;
            default:
                scene = SCENE_DEFAULT;
                break;
        }
        
        // 为所有底盘电机切换场景
        for (int i = 0; i < 4; i++) {
            Djimotor_switch_scene(chassis_motors[i], scene);
        }

        // 使用全向轮运动学模型
        float motor_speed[4];
        
        // 假设全向轮布局为经典的"十字布局"（90度间隔）
        // 轮子布局（从俯视图看）：
        //    1(前)
        //  2(左) 0(右)
        //    3(后)
        //
        // 全向轮运动学模型（轮子方向为90度间隔）
        // 右轮(0) = +vy - wz  （贡献侧向移动和旋转）
        // 前轮(1) = +vx - wz  （贡献前向移动和旋转）
        // 左轮(2) = -vy - wz  （贡献侧向移动和旋转）
        // 后轮(3) = -vx - wz  （贡献前向移动和旋转）
        
        // 旋转补偿系数，根据底盘尺寸调整
        float rotate_ratio = 1.0f;
        float wheel_radius = 0.076f; // 轮子半径，单位：米
        float chassis_radius = 0.2f; // 底盘半径（中心到轮子距离），单位：米
        
        // 计算旋转分量
        float rotate_compensation = rotate_ratio * chassis_cmd.wz * chassis_radius / wheel_radius;
        
        motor_speed[0] = chassis_cmd.vy - rotate_compensation;  // 右轮
        motor_speed[1] = chassis_cmd.vx - rotate_compensation;  // 前轮
        motor_speed[2] = -chassis_cmd.vy - rotate_compensation; // 左轮
        motor_speed[3] = -chassis_cmd.vx - rotate_compensation; // 后轮

        // 设置电机目标速度
        for (int i = 0; i < 4; i++) {
            Djimotor_set_target(chassis_motors[i], motor_speed[i]);
        }
    }
}

/**
 * @brief 处理云台控制指令
 * @note  订阅云台控制指令并控制云台电机
 */
void Motor_control_handle_gimbal_cmd(void)
{
    // 从消息中心获取最新指令
    if (Sub_get_message(gimbal_cmd_sub, &gimbal_cmd)) {
        
        // 根据云台模式设置云台电机控制
        switch (gimbal_cmd.gimbal_mode) {
            case GIMBAL_GYRO_MODE:
                // 设置YAW和PITCH电机的目标角度
                Djimotor_set_target(gimbal_motors[0], gimbal_cmd.yaw);    // YAW
                Djimotor_set_target(gimbal_motors[1], gimbal_cmd.pitch);  // PITCH
                break;
                
            case GIMBAL_VISION_MODE:
                // 视觉模式下的控制逻辑
                // ...
                break;
                
            case GIMBAL_ZERO_FORCE:
            default:
                // 零力矩模式，不控制电机
                Djimotor_set_target(gimbal_motors[0], 0);
                Djimotor_set_target(gimbal_motors[1], 0);
                break;
        }
    }
}

/**
 * @brief 处理发射机构控制指令
 * @note  订阅发射控制指令并控制发射机构电机
 */
void Motor_control_handle_shoot_cmd(void)
{
    // 从消息中心获取最新指令
    if (Sub_get_message(shoot_cmd_sub, &shoot_cmd)) {
        
        // 根据发射模式控制摩擦轮
        switch (shoot_cmd.shoot_mode) {
            case SHOOT_ON:
                // 摩擦轮开启 - 设置为正向高速旋转
                Djimotor_set_target(shoot_motors[0], 5000);  // 左摩擦轮
                Djimotor_set_target(shoot_motors[1], -5000); // 右摩擦轮（反向转动）
                break;
                
            case SHOOT_OFF:
            default:
                // 摩擦轮关闭
                Djimotor_set_target(shoot_motors[0], 0);
                Djimotor_set_target(shoot_motors[1], 0);
                break;
        }
        
        // 根据拨弹模式控制拨弹电机
        static float loader_angle = 0;
        switch (shoot_cmd.loader_mode) {
            case LOAD_1_BULLET:
                // 单发 - 旋转固定角度
                loader_angle += 36.0f;  // 假设每颗子弹需要36度旋转
                Djimotor_set_target(shoot_motors[2], loader_angle);
                break;
                
            case LOAD_BURSTFIRE:
                // 连发 - 持续旋转
                loader_angle += 3.6f;  // 每次增加一定角度
                Djimotor_set_target(shoot_motors[2], loader_angle);
                break;
                
            case LOAD_REVERSE:
                // 反转 - 可用于处理卡弹
                loader_angle -= 10.0f;
                Djimotor_set_target(shoot_motors[2], loader_angle);
                break;
                
            case LOAD_STOP:
            default:
                // 保持当前位置
                Djimotor_set_target(shoot_motors[2], loader_angle);
                break;
        }
    }
}

/**
 * @brief 收集电机反馈信息并发布
 * @note  收集所有电机的反馈数据并通过消息中心发布
 */
void Motor_control_collect_feedback(void)
{
    // 收集底盘反馈信息
    // 例如：底盘角速度可以从电机反馈获取
    float chassis_wz = 0;
    for (int i = 0; i < 4; i++) {
        Djimotor_measure_t measure = Djimotor_get_measure(chassis_motors[i]);
        // 根据轮子的速度计算底盘角速度
        // 这里是简化计算，实际应该根据运动学模型
        chassis_wz += measure.angular_velocity / 4.0f;
    }
    chassis_feedback.chassis_wz = chassis_wz;
    Pub_push_message(chassis_feedback_pub, &chassis_feedback);
    
    // 收集云台反馈信息
    Djimotor_measure_t yaw_measure = Djimotor_get_measure(gimbal_motors[0]);
    gimbal_feedback.yaw_motor_angle = yaw_measure.current_angle;
    Pub_push_message(gimbal_feedback_pub, &gimbal_feedback);
    
    // 收集发射机构反馈信息
    // 假设枪管热量从其他系统获取
    shoot_feedback.gun_rest_heat = 100; // 假设值
    Pub_push_message(shoot_feedback_pub, &shoot_feedback);
}

// 内联函数已移至头文件中实现
