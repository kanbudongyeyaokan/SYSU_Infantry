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

/**
 * @brief 云台初始化
 */
void Gimbal_task_init(void);

/**
 * @brief 处理云台控制指令
 */
void Gimbal_handle_command(void);
#endif //SYSU_INFANTRY_GIMBAL_H