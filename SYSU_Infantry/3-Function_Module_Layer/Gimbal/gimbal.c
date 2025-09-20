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
// Created by 26524 on 2025/9/16.
//
//
#include "gimbal.h"
#include "dji_motor.h"


//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//获取角度数据 未实现
//static attitude_t *gimbal_imu_data;



/**
 * @brief 云台初始化
 */
void Gimbal_motor_init(void) {
    Djimotor_init_config_t yaw_config = {
        .motor_name = "yaw_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_ENABLED,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            //完善ins task后修正下面两行
            //.other_angle_feedback_ptr = gimbal_imu_data->
            //.other_speed_feedback_ptr = gimbal_imu_data->
            .angle_pid = {
                .kp = 8,
                .ki = 0,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 500,
                .max_iout = 100,
                //可补充
            },
            .speed_pid = {
                .kp = 50,
                .ki = 200,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 3000,
                .max_iout = 20000,
            },

        },
        .can_init = {
            .can_handle = &hcan2,
            .can_id = 0x1FF,
            .tx_id = 1,
            .rx_id = 0x205,
        },
    };

    yaw_motor = DJI_Motor_Init(&yaw_config);

    Djimotor_init_config_t pitch_config = {
        .motor_name = "pitch_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_ENABLED,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            //完善ins task后修正下面两行
            //.other_angle_feedback_ptr = gimbal_imu_data->
            //.other_speed_feedback_ptr = gimbal_imu_data->
            .angle_pid = {
                .kp = 10,
                .ki = 0,
                .kd = 0,
                .max_out = 500,
                .max_iout = 100,
                //可补充
            },
            .speed_pid = {
                .kp = 50,
                .ki = 350,
                .kd = 0,
                .deadband = 0.1f,
                .max_out = 2500,
                .max_iout = 20000,
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
 * @brief 获取yaw轴电机指针
 * @return yaw轴电机指针
 */
Djimotor_device_t* Get_yaw_motor(void) {
    return yaw_motor;
}

/**
 * @brief 获取pitch轴电机指针
 * @return pitch轴电机指针
 */
Djimotor_device_t* Get_pitch_motor(void) {
    return pitch_motor;
}
