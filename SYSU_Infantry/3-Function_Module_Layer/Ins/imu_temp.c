/**
 * @Author  SYSU电控组
 * @file    imu_temp.c
 * @brief   IMU恒温控制模块
 * @note    基于RoboMaster开发板C型：加热电阻连接在 TIM10_CH1 (PF6)
 *
 */
#include "imu_temp.h"
#include "algorithm_pid.h"
#include "tim.h"
#include "main.h"

// 目标温度：通常设置为高于环境温度
// BMI088 在恒温下零漂最稳定
#define IMU_TEMP_TARGET 40.0f

// PWM 最大值
// ARR 为 1000
#define PWM_MAX_VALUE   1000.0f

static Pid_instance_t temp_pid;

/**
 * @brief 初始化IMU恒温控制
 * @note  必须在 task 开始前调用，且需确保 MX_TIM10_Init 已执行
 */
void Imu_Temp_Init(void)
{
    //初始化PID参数
    Pid_init_t pid_conf = {
        .kp = 800.0f,      // 比例系数
        .ki = 1.0f,         // 积分系数 (加热过程积分不能太大，否则超调严重)
        .kd = 0.0f,         // 微分系数
        .max_out = PWM_MAX_VALUE,   // 最大输出限制 (PWM满占空比)
        .max_iout = 200.0f,         // 积分限幅
        .deadband = 0.1f,           // 死区
        .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL // 启用输出限幅和梯形积分
    };

    Pid_init(&temp_pid, &pid_conf);

    //开启 TIM10 Channel 1 的 PWM 输出
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

/**
 * @brief 执行恒温控制
 * @param current_temp 当前IMU温度 (单位: 摄氏度)
 * @note  建议调用频率：50Hz ~ 100Hz (温度变化慢，不需要 1kHz 那么快)
 */
void Imu_Temp_Control(float current_temp)
{
    // 计算PID输出
    // 注意：加热是单向控制，只能加热不能制冷
    float pid_out = Pid_calculate(&temp_pid, current_temp, IMU_TEMP_TARGET);
    // 限制输出范围 [0, PWM_MAX]
    if (pid_out < 0.0f) {
        pid_out = 0.0f;
    } else if (pid_out > PWM_MAX_VALUE) {
        pid_out = PWM_MAX_VALUE;
    }
    // 3. 设置 PWM 占空比
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint16_t)pid_out);
}