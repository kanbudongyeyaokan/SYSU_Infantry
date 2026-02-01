/**
 * @file    gimbal.c
 * @brief   云台功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-20
 * @version 1.0
 *
 * @note    云台电机初始化
 */

#ifndef SYSU_INFANTRY_GIMBAL_H
#define SYSU_INFANTRY_GIMBAL_H
#include "decision_making.h"

/**
 * @brief 云台状态枚举 - 用于上电归中状态机
 */
typedef enum {
    GIMBAL_STATE_INIT = 0,   // 初始化状态，等待 IMU Ready
    GIMBAL_STATE_ZEROING,    // 归中状态，使用编码器反馈回到机械零位
    GIMBAL_STATE_READY       // 就绪状态，使用 IMU 反馈进行正常控制
} Gimbal_state_e;

/**
 * @brief 获取当前云台状态
 */
Gimbal_state_e Gimbal_get_state(void);

/**
 * @brief 云台初始化
 */
void Gimbal_task_init(void);

/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd);
#endif //SYSU_INFANTRY_GIMBAL_H

