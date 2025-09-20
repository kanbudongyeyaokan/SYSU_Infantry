/**
 * @file    chassis.c
 * @brief   底盘功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    实现底盘运动学解算功能，支持多种底盘类型
 */

#include "chassis.h"
#include "bsp_can.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 底盘参数存储
static Chassis_params_t chassis_params = {0};

// 底盘电机实例（对外可见）
Djimotor_device_t *chassis_motors[4] = {0};

void Chassis_motors_init(void)
{
    Djimotor_init_config_t cfg[4] = {
        {
            .motor_name = "CHASSIS_FR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
        },{
            .motor_name = "CHASSIS_FL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
        },{
            .motor_name = "CHASSIS_BL",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
        },{
            .motor_name = "CHASSIS_BR",
            .motor_type = M3508,
            .motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {.close_loop = OPEN_LOOP},
            .can_init = {.can_handle = &hcan1, .can_id = 0x200, .tx_id = 4, .rx_id = 0x204}
        }
    };

    for (int i = 0; i < 4; i++) {
        chassis_motors[i] = DJI_Motor_Init(&cfg[i]);
    }

    // 场景配置迁移
    for (int i = 0; i < 4; i++) {
        Djimotor_scene_config_t no_follow = {
            .close_loop = OPEN_LOOP,
            .speed_pid = {.kp = 15.f, .ki = 0.5f, .kd = 0.f, .max_out = 16000.f, .max_iout = 5000.f, .deadband = 10.f, .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT, .LPF_coefficient = 0.85f}
        };
        Djimotor_update_scene_config(chassis_motors[i], CHASSIS_NO_FOLLOW, &no_follow);

        Djimotor_scene_config_t follow = {
            .close_loop = OPEN_LOOP,
            .speed_pid = {.kp = 18.f, .ki = 0.6f, .kd = 0.05f, .max_out = 16000.f, .max_iout = 6000.f, .deadband = 8.f, .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT, .LPF_coefficient = 0.8f}
        };
        Djimotor_update_scene_config(chassis_motors[i], CHASSIS_FOLLOW_GIMBAL, &follow);

        Djimotor_scene_config_t rotate = {
            .close_loop = OPEN_LOOP,
            .speed_pid = {.kp = 20.f, .ki = 0.8f, .kd = 0.1f, .max_out = 16000.f, .max_iout = 8000.f, .deadband = 5.f, .optimization = PID_OUTPUT_FILTER | PID_OUTPUT_LIMIT, .LPF_coefficient = 0.7f}
        };
        Djimotor_update_scene_config(chassis_motors[i], CHASSIS_ROTATE, &rotate);

        Djimotor_scene_config_t zero = {
            .close_loop = OPEN_LOOP,
            .speed_pid = {0}
        };
        Djimotor_update_scene_config(chassis_motors[i], CHASSIS_ZERO_FORCE, &zero);
    }
}

/**
 * @brief 全向轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_omni_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 全向轮布局（从俯视图看）：
    //    1(前)
    //  2(左) 0(右)
    //    3(后)
    // 
    // 全向轮运动学模型（轮子方向为90度间隔）
    // 右轮(0) = +vy - wz*R  （贡献侧向移动和旋转）
    // 前轮(1) = +vx - wz*R  （贡献前向移动和旋转）
    // 左轮(2) = -vy - wz*R  （贡献侧向移动和旋转）
    // 后轮(3) = -vx - wz*R  （贡献前向移动和旋转）
    
    float rotate_compensation = cmd->wz * chassis_params.chassis_radius / chassis_params.wheel_radius;
    
    // 计算各轮子线速度 (m/s)
    float wheel_linear_speed[4];
    wheel_linear_speed[0] = cmd->vy - rotate_compensation;   // 右轮
    wheel_linear_speed[1] = cmd->vx - rotate_compensation;   // 前轮
    wheel_linear_speed[2] = -cmd->vy - rotate_compensation;  // 左轮
    wheel_linear_speed[3] = -cmd->vx - rotate_compensation;  // 后轮
    
    // 转换为角速度 (rad/s) 再转换为 rpm
    for (int i = 0; i < 4; i++) {
        float angular_velocity = wheel_linear_speed[i] / chassis_params.wheel_radius; // rad/s
        output->motor_speed[i] = angular_velocity * 60.0f / (2.0f * M_PI); // rpm
    }
    
    output->motor_count = 4;
}

/**
 * @brief 麦克纳姆轮底盘运动学解算
 * @param cmd 底盘控制指令  
 * @param output 解算输出结果
 */
static void Chassis_mecanum_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 麦克纳姆轮布局（从俯视图看）：
    // 左前(0)  右前(1)
    // 左后(2)  右后(3)
    //
    // 麦克纳姆轮运动学模型
    // 左前轮 = vx - vy - wz*(wheelbase + track_width)/2
    // 右前轮 = vx + vy + wz*(wheelbase + track_width)/2  
    // 左后轮 = vx + vy - wz*(wheelbase + track_width)/2
    // 右后轮 = vx - vy + wz*(wheelbase + track_width)/2
    
    float L = chassis_params.wheel_base;      // 轮距
    float W = chassis_params.track_width;     // 轮宽
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
    
    output->motor_count = 4;
}

/**
 * @brief 差速轮底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_differential_kinematics(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 差速轮布局：
    // 左轮(0)  右轮(1)
    //
    // 差速轮运动学模型（只支持前进和旋转，不支持侧向移动）
    // 左轮 = vx - wz * wheelbase / 2
    // 右轮 = vx + wz * wheelbase / 2
    
    float L = chassis_params.wheel_base;
    
    // 计算各轮子线速度 (m/s)
    float wheel_linear_speed[2];
    wheel_linear_speed[0] = cmd->vx - cmd->wz * L / 2.0f; // 左轮
    wheel_linear_speed[1] = cmd->vx + cmd->wz * L / 2.0f; // 右轮
    
    // 转换为角速度 (rad/s) 再转换为 rpm
    for (int i = 0; i < 2; i++) {
        float angular_velocity = wheel_linear_speed[i] / chassis_params.wheel_radius; // rad/s
        output->motor_speed[i] = angular_velocity * 60.0f / (2.0f * M_PI); // rpm
    }
    
    // 清零未使用的电机
    output->motor_speed[2] = 0.0f;
    output->motor_speed[3] = 0.0f;
    output->motor_count = 2;
}

/**
 * @brief 底盘功能模块初始化
 * @param params 底盘物理参数
 */
void Chassis_init(const Chassis_params_t *params)
{
    chassis_params = *params;
    
    // 设置默认参数（如果未提供）
    if (chassis_params.wheel_radius == 0.0f) {
        chassis_params.wheel_radius = 0.076f;  // 默认轮子半径76mm
    }
    
    if (chassis_params.chassis_radius == 0.0f) {
        chassis_params.chassis_radius = 0.2f;  // 默认底盘半径200mm
    }
    
    if (chassis_params.wheel_base == 0.0f) {
        chassis_params.wheel_base = 0.4f;      // 默认轮距400mm
    }
    
    if (chassis_params.track_width == 0.0f) {
        chassis_params.track_width = 0.3f;     // 默认轮宽300mm
    }
}

/**
 * @brief 底盘运动学解算
 * @param cmd 底盘控制指令
 * @param output 解算输出结果
 */
void Chassis_kinematics_solve(const Chassis_cmd_send_t *cmd, Chassis_output_t *output)
{
    // 清零输出
    for (int i = 0; i < 4; i++) {
        output->motor_speed[i] = 0.0f;
    }
    output->motor_count = 0;
    
    // 根据底盘类型调用对应的解算函数
    switch (chassis_params.chassis_type) {
        case CHASSIS_TYPE_OMNI:
            Chassis_omni_kinematics(cmd, output);
            break;
            
        case CHASSIS_TYPE_MECANUM:
            Chassis_mecanum_kinematics(cmd, output);
            break;
            
        case CHASSIS_TYPE_DIFFERENTIAL:
            Chassis_differential_kinematics(cmd, output);
            break;
            
        default:
            // 未知底盘类型，保持静止
            output->motor_count = 4;
            break;
    }
}

/**
 * @brief 设置底盘类型
 * @param chassis_type 底盘类型
 */
void Chassis_set_type(chassis_type_e chassis_type)
{
    chassis_params.chassis_type = chassis_type;
}

/**
 * @brief 获取当前底盘类型
 * @return chassis_type_e 当前底盘类型
 */
chassis_type_e Chassis_get_type(void)
{
    return chassis_params.chassis_type;
}
