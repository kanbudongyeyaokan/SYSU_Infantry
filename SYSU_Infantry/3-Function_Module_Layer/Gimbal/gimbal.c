/**
 * @file    gimbal.c
 * @brief   云台功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    云台电机初始化
 */

#include "cmsis_os.h"
#include <stdbool.h>

#include "gimbal.h"

#include <stdio.h>

#include "dji_motor.h"
#include "decision_making.h"
#include "message_center.h"
#include "ins.h"
#include "robot_definitions.h"


//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//云台模块的姿态数据指针，指向ins模块的全局变量
static attitude_t *gimbal_imu_data;

// 订阅决策层发来的云台控制指令
static Subscriber_t *gimbal_sub;
// 存储决策层发来的控制命令
static Gimbal_cmd_send_t gimbal_cmd_send;

// 发布给决策层的云台反馈信息
static Publisher_t*gimbal_pub;
// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

// 记录 INS 是否已进入 READY 状态
static bool gimbal_ins_ready = false;



/**
 * @brief 云台初始化
 */
static void Gimbal_motor_init(void) {
    //YAW电机
    Djimotor_init_config_t yaw_config = {
        .motor_name = "yaw_motor",
        .motor_type = GM6020,
        .motor_status = MOTOR_STOP,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = MOTOR_FEEDBACK,
            //使用ins模块姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data->yaw_total_angle),
            // .other_speed_feedback_ptr = &(gimbal_imu_data->yaw_rate_dps),
            .angle_pid = {
                .kp = 20,
                .ki = 0.0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = 500,
                .max_iout = 100,
               // .feedfoward_coefficient = 0.2,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_FEEDFOWARD|PID_DIFFERENTIAL_GO_FIRST,
                //可补充
            },
            .speed_pid = {
                .kp = 20,
                .ki = 0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = 28000,
                .max_iout = 8000,
               // .feedfoward_coefficient = 0.2,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_FEEDFOWARD|PID_DIFFERENTIAL_GO_FIRST,
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
        .motor_status = MOTOR_STOP,
        .motor_controller_init = {
            .close_loop = ANGLE_AND_SPEED_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = MOTOR_FEEDBACK,

            //使用ins模块姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data->euler_angles.pitch),
            // .other_speed_feedback_ptr = &(gimbal_imu_data->gyro_raw.pitch),
            .angle_pid = {
                .kp = 18,
                .ki = 1,
                .kd = 0.0,
                .max_out = 100,
                .max_iout = 100,
                //.feedfoward_coefficient = 0.2,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_DIFFERENTIAL_GO_FIRST,
                //可补充
            },
            .speed_pid = {
                .kp = -15,
                .ki = 0.0,
                .kd = 0.0,
                .deadband = 0.1f,
                .max_out = 10000,
                .max_iout = 800,
              //  .feedfoward_coefficient = 0.2,
                .optimization = PID_OUTPUT_LIMIT|PID_TRAPEZOID_INTERGRAL|PID_DIFFERENTIAL_GO_FIRST,
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

    // 利用上述配置云台上电后回到零位
    Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
    Djimotor_set_status(pitch_motor, MOTOR_ENABLED);
  //  Djimotor_set_target(yaw_motor, YAW_ALIGN_ANGLE);
  //  Djimotor_set_target(pitch_motor, PITCH_HORIZON_ANGLE);

   // osDelay(2000); // 等待2秒到达位置

    // 之后更改云台配置（直接修改成员，避免重新建结构体）
   // Djimotor_set_status(yaw_motor, MOTOR_STOP);
   // Djimotor_set_status(pitch_motor, MOTOR_STOP);

    // yaw: 切换到 IMU yaw_total_angle 作为角度反馈，并更新 PID 及限幅
    /*
    yaw_motor->motor_pid.close_loop = ANGLE_AND_SPEED_LOOP;
    yaw_motor->motor_pid.angle_source = OTHER_FEEDBACK;
    yaw_motor->motor_pid.speed_source = MOTOR_FEEDBACK;
    yaw_motor->motor_pid.other_angle_feedback_ptr = &(gimbal_imu_data->yaw_total_angle);

    Pid_reset(&yaw_motor->motor_pid.angle_pid);
    Pid_reset(&yaw_motor->motor_pid.speed_pid);

    // pitch: 切换到 IMU pitch 作为角度反馈，并更新 PID 及限幅
    pitch_motor->motor_pid.close_loop = ANGLE_AND_SPEED_LOOP;
    pitch_motor->motor_pid.angle_source = OTHER_FEEDBACK;
    pitch_motor->motor_pid.speed_source = MOTOR_FEEDBACK;
    pitch_motor->motor_pid.other_angle_feedback_ptr = &(gimbal_imu_data->euler_angles.pitch);

    Pid_reset(&pitch_motor->motor_pid.angle_pid);
    Pid_reset(&pitch_motor->motor_pid.speed_pid);

    // 重新设定目标并使能
    Djimotor_set_target(yaw_motor, YAW_ALIGN_ANGLE);
    Djimotor_set_target(pitch_motor, PITCH_HORIZON_ANGLE);
    Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
    Djimotor_set_status(pitch_motor, MOTOR_ENABLED);
*/
}



/**
 * @brief 云台任务初始化
 */
void Gimbal_task_init(void) {
    // 获取ins模块的姿态数据指针
    gimbal_imu_data = get_attitude_data();
    
    //初始化云台电机
    Gimbal_motor_init();

    // 订阅决策层发来的控制指令
    gimbal_sub = Sub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));

    // 注册底盘反馈信息发布者
    gimbal_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));

}


/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(void) {
    Imu_state_e imu_state = ins_get_state();
   // Djimotor_set_target(yaw_motor, 300);
   // Djimotor_set_target(pitch_motor, 30);
  //  printf("motor_yaw:%.2f,motor_pitch:%.2f\r\n",yaw_motor->motor_measure.total_angle,pitch_motor->motor_measure.total_angle);
  //  printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n",gimbal_imu_data->euler_angles.yaw,gimbal_imu_data->euler_angles.pitch,gimbal_imu_data->euler_angles.roll);
    if (imu_state == IMU_STATE_READY) {
        if (!gimbal_ins_ready) {
            gimbal_ins_ready = true;
            // 首次进入 READY，重新对齐机械零位并使能闭环
        }
    } else {
        if (gimbal_ins_ready) {
            gimbal_ins_ready = false;
            // IMU 未就绪时停机，避免错误姿态参与闭环
            Djimotor_set_status(yaw_motor, MOTOR_STOP);
            Djimotor_set_status(pitch_motor, MOTOR_STOP);
        }
    }

    // 从消息中心获取最新的控制指令
     if(Sub_get_message(gimbal_sub, (void *) (&gimbal_cmd_send))) {
     //    printf("cmd_yaw:%f,cmd_pitch:%f\r\n",gimbal_cmd_send.yaw,gimbal_cmd_send.pitch);
        // 根据控制模式进行处理
        switch (gimbal_cmd_send.gimbal_mode) {
            // 电流零输入,失能云台电机
            case GIMBAL_ZERO_FORCE:
                Djimotor_set_status(yaw_motor, MOTOR_STOP);
                Djimotor_set_status(pitch_motor, MOTOR_STOP);
                break;

            //云台陀螺仪反馈模式
            case GIMBAL_GYRO_MODE:

                //使能电机
                Djimotor_set_status(yaw_motor, MOTOR_ENABLED);
                Djimotor_set_status(pitch_motor, MOTOR_ENABLED);

                //设置电机目标值
                Djimotor_set_target(yaw_motor, gimbal_cmd_send.yaw);
                yaw_motor->motor_pid.speed_feedforward = gimbal_cmd_send.chassis_wz; // 底盘角速度补偿
                // 注意正负号以及单位
                Djimotor_set_target(pitch_motor, gimbal_cmd_send.pitch);
                break;
            //云台视觉模式
            case GIMBAL_VISION_MODE:
                //根据视觉补充

                break;

            default:
                break;
        }

    }

    //反馈数据

    gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
    gimbal_feedback.yaw_motor_total_angle = yaw_motor->motor_measure.total_angle;
    gimbal_feedback.imu_yaw_total_angle = gimbal_imu_data->yaw_total_angle;
    gimbal_feedback.imu_yaw_rate = gimbal_imu_data->yaw_rate_dps;
    gimbal_feedback.imu_state = imu_state;

    //推送消息
    if (gimbal_pub != NULL) {
        Pub_push_message(gimbal_pub, (void *) &gimbal_feedback);
    }

}
