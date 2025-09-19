#include "dji_motor.h"

/**
 * @brief 切换电机控制场景
 * 
 * @param motor 电机设备指针
 * @param scene 目标场景
 */
void Djimotor_switch_scene(Djimotor_device_t *motor, chassis_mode_e scene) {
    if (motor == NULL || scene >= MAX_MOTOR_SCENES) {
        return;
    }

    // 如果已经是当前场景则不需要切换
    if (motor->motor_pid.current_scene == scene) {
        return;
    }

    // 保存当前场景索引
    motor->motor_pid.current_scene = scene;
    
    // 获取目标场景的配置
    Djimotor_scene_config_t *scene_config = &motor->motor_pid.scene_configs[scene];
    
    // 更新控制模式
    motor->motor_pid.close_loop = scene_config->close_loop;
    
    // 更新PID参数 - 电流环
    Pid_init(&motor->motor_pid.current_pid, &scene_config->current_pid);
    
    // 更新PID参数 - 速度环
    Pid_init(&motor->motor_pid.speed_pid, &scene_config->speed_pid);
    
    // 更新PID参数 - 角度环
    Pid_init(&motor->motor_pid.angle_pid, &scene_config->angle_pid);
}

/**
 * @brief 更新指定场景的PID配置
 * 
 * @param motor 电机设备指针
 * @param scene 目标场景
 * @param config 新的场景配置
 */
void Djimotor_update_scene_config(Djimotor_device_t *motor, chassis_mode_e scene, Djimotor_scene_config_t *config) {
    if (motor == NULL || config == NULL || scene >= MAX_MOTOR_SCENES) {
        return;
    }
    
    // 更新场景配置
    motor->motor_pid.scene_configs[scene] = *config;
    
    // 如果当前正在使用该场景，则立即应用新配置
    if (motor->motor_pid.current_scene == scene) {
        Djimotor_switch_scene(motor, scene);
    }
}

/**
 * @brief 获取电机当前使用的场景
 * 
 * @param motor 电机设备指针
 * @return Djimotor_scene_e 当前场景
 */
chassis_mode_e Djimotor_get_current_scene(Djimotor_device_t *motor) {
    if (motor == NULL) {
        return CHASSIS_ZERO_FORCE;
    }
    return motor->motor_pid.current_scene;
}