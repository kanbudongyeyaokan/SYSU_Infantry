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

	// 简单的圆轨迹测试：vx, vy 正弦/余弦变化；wz 置零
	// 该任务用于模拟“决策层”输出，验证消息中心、底盘解算与电机驱动的完整链路
	Chassis_cmd_send_t cmd = {0};
	const float v = 6.0f;        // 水平合速度幅值（单位按底盘实现约定）
	const float dt_s = 0.01f;    // 发布周期：10ms
	const unsigned int dt_ms = 10u;  // 10ms
	float theta = 0.0f;          // 相位

    // 设置底盘模式与控制量
	cmd.chassis_mode = CHASSIS_NO_FOLLOW;
	cmd.offset_angle = 0.0f;

	for (;;)
	{
		

		// 圆形轨迹：vx = v*cos(theta), vy = v*sin(theta)
		cmd.vx = v * cosf(theta);
		cmd.vy = v * sinf(theta);
		cmd.wz = 0.0f; // 不叠加原地旋转

		// 发布到消息中心前打印调试信息
		// printf("[PUB][chassis_cmd] mode=%d vx=%.3f vy=%.3f wz=%.3f\r\n", (int)cmd.chassis_mode, cmd.vx, cmd.vy, cmd.wz);

		// 发布到消息中心，供 Chassis_task 订阅处理，并打印被推送的订阅者数量
		uint8_t pushed = Pub_push_message(chassis_cmd_pub, &cmd);
		// printf("[PUB][chassis_cmd] pushed=%u\r\n", (unsigned)pushed);

		// 推进相位，控制“转圈”的角速度（此处约等于 2π/8s 的角速度）
		theta += (2.0f * (float)M_PI) * dt_s / 8.0f;
		if (theta > (2.0f * (float)M_PI))
		{
			theta -= 2.0f * (float)M_PI;
		}

		// 周期调度
		osDelay(dt_ms);
	}
}

