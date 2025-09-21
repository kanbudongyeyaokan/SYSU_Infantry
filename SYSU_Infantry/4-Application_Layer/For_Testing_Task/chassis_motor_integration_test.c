/**
 * @file    
 * @brief   底盘电机集成测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    用于测试底盘->话题数据->电机驱动的完整数据流框架
 */

#include <stdint.h>
#include "chassis_motor_integration_test.h"
#include "motor_task.h"
#include "Chassis_task.h"
#include "message_center.h"
#include "decision_making.h"
#include "chassis.h"
#include "dji_motor.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 发布者：向底盘任务发布控制指令，形成闭环链路：
// integration_test -> (message_center: "chassis_cmd") -> Chassis_task -> Djimotor_* -> CAN 发送

void Chassis_motor_integration_test_task(void const *argument)
{
	// 注册发布者，话题名称需与底盘任务订阅一致
	Publisher_t *chassis_cmd_pub = Pub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));

	// 简单的圆轨迹测试：通过恒定的前进速度和恒定的自转角速度实现
	// 该任务用于模拟“决策层”输出，验证消息中心、底盘解算与电机驱动的完整链路
	Chassis_cmd_send_t cmd = {0};
	const float forward_speed = 5.0f;    // 恒定的前进速度
	const float rotation_speed = 0.785f; // 恒定的自转角速度 (rad/s)，约等于 2*PI/8s
	const unsigned int dt_ms = 10u;      // 10ms 周期

    // 设置底盘模式与控制量
	cmd.chassis_mode = CHASSIS_NO_FOLLOW; // 通常自转时使用无云台跟随模式
	cmd.offset_angle = 0.0f;

	// 设置恒定的速度指令
	cmd.vx = 0.0;    // 保持向前运动
	cmd.vy = 0.0f;             // 不进行侧向平移
	cmd.wz = rotation_speed;   // 保持原地自转

	for (;;)
	{
		// 直接发布恒定的指令即可，底盘会因为同时前进和自转而走出圆形轨迹
		Pub_push_message(chassis_cmd_pub, &cmd);

		// 周期调度
		osDelay(dt_ms);
	}
}

