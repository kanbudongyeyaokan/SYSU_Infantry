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
#include "dji_motor.h"
#include "decision_making.h"
#include "message_center.h"
#include "ins.h"
#include "robot_definitions.h"

//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

//云台模块的姿态数据副本
static attitude_t gimbal_imu_data;

// 订阅决策层发来的云台控制指令
static Subscriber_t *gimbal_sub;
// 存储决策层发来的控制命令
static Gimbal_cmd_send_t gimbal_cmd_send;

// 发布给决策层的云台反馈信息
static Publisher_t*gimbal_pub;
// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;



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
            .close_loop = OPEN_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            //使用本地姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data.euler_angles.yaw),
            .other_speed_feedback_ptr = &(gimbal_imu_data.gyro_raw.yaw),
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
            .close_loop = OPEN_LOOP,
            .angle_source = OTHER_FEEDBACK,
            .speed_source = OTHER_FEEDBACK,
            //使用本地姿态数据作为反馈
            .other_angle_feedback_ptr = &(gimbal_imu_data.euler_angles.pitch),
            .other_speed_feedback_ptr = &(gimbal_imu_data.gyro_raw.pitch),
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
 * @brief 云台任务初始化
 */
void Gimbal_task_init(void) {
    //初始化云台电机
    Gimbal_motor_init();

    // 订阅决策层发来的控制指令
    gimbal_sub = Sub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));

    // 注册底盘反馈信息发布者
    gimbal_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));

    //云台归零
    Djimotor_set_target(yaw_motor, YAW_ALIGN_ANGLE);
    Djimotor_set_target(pitch_motor,PITCH_HORIZON_ANGLE);
}

/**
 * @brief 更新云台模块的姿态数据
 * @note 该函数应该在云台任务中定期调用，从ins模块获取最新的姿态数据
 */
static void Gimbal_update_imu_data(void) {
    // 从ins模块获取最新姿态数据的指针
    const attitude_t *latest_attitude = get_attitude_data();
    
    // 将数据复制到本地静态变量（值拷贝，而非指针）
    if (latest_attitude != NULL) {
        gimbal_imu_data = *latest_attitude;
    }
}


/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(void) {
    // 首先更新本地的姿态数据
    Gimbal_update_imu_data();
    
    // 从消息中心获取最新的控制指令
    if (Sub_get_message(gimbal_sub, (void *) (&gimbal_cmd_send))) {
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
                Djimotor_set_target(pitch_motor, gimbal_cmd_send.pitch);
                break;
                //云台视觉模式
            case GIMBAL_VISION_MODE:
                //根据视觉补充

                break;

            default:
                break;
        }
        //反馈数据
        gimbal_feedback.yaw_motor_angle = yaw_motor->motor_measure.current_angle;

        //推送消息
        Pub_push_message(gimbal_pub, (void *) &gimbal_feedback);
    }
}
