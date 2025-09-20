/**
* @file    Gimbal_task.c
 * @brief   云台控制任务源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    负责底盘运动学解算和控制量传递
 */

#include "Gimbal_task.h"
#include "gimbal.h"
#include "decision_making.h"
#include "message_center.h"
#include "dji_motor.h"
#include "cmsis_os.h"

//云台电机
static Djimotor_device_t *yaw_motor, *pitch_motor;

// 订阅决策层发来的底盘控制指令
static Subscriber_t *gimbal_sub;

// 发布给决策层的底盘反馈信息
static Publisher_t*gimbal_pub;

// 存储发送给决策层的反馈信息
static Gimbal_feedback_info_t gimbal_feedback;

// 存储决策层发来的控制命令
static Gimbal_cmd_send_t gimbal_cmd_send;

/**
 * @brief 云台任务初始化
 */
static void Gimbal_task_init(void) {
    //初始化云台电机
    Gimbal_init();

    // 订阅决策层发来的控制指令
    gimbal_sub = Sub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));

    // 注册底盘反馈信息发布者
    gimbal_pub = Pub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));
}

/**
 * @brief 获取云台电机指针
 */
static void Get_motor(void) {
    yaw_motor = Get_yaw_motor();
    pitch_motor = Get_pitch_motor();
}

/**
 * @brief 处理云台控制指令
 */
static void Gimbal_handle_command(void) {
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

/**
 * @brief 云台控制任务函数
 * @param argument 任务参数（未使用）
 * @note 按照应用层设计，此任务只负责调用功能模块层接口执行控制
 */
void Gimbal_control_task(void const *argument) {
    //任务初始化
    Gimbal_task_init();

    //获取电机指针
    Get_motor();

    for (;;) {
        //处理控制指令
        Gimbal_handle_command();

        //控制频率多少来着,忘了
        osDelay(2);
    }
}
