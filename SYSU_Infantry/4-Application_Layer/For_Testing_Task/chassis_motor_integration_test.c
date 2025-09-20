/**
 * @file    chassis_motor_integration_test.c
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

// 发布者/订阅者实例
static Publisher_t *chassis_cmd_pub = NULL;
static Subscriber_t *chassis_feedback_sub = NULL;

// 本测试用到的缓存对象
static Chassis_cmd_send_t cmd = {0};
static Chassis_feedback_info_t feedback = {0};

// 打印辅助
#define TEST_LOG(fmt, ...)                              \
	do {                                                \
		if (debug_uart)                                 \
			Uart_printf(debug_uart, fmt, ##__VA_ARGS__); \
	} while (0)

// 简单的限幅
static float clampf(float v, float lo, float hi) {
	return (v < lo) ? lo : (v > hi ? hi : v);
}

// 初始化发布/订阅
static void test_pubsub_init(void)
{
	// 与 Chassis_task 保持一致的话题名
	chassis_cmd_pub = Pub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
	chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));

	TEST_LOG("[TEST] pub-sub ready: cmd->'chassis_cmd' size=%d, sub<-'chassis_feedback' size=%d\r\n",
			 (int)sizeof(Chassis_cmd_send_t), (int)sizeof(Chassis_feedback_info_t));
}

// 发布一帧底盘控制指令（频率建议>=100Hz）
static void publish_cmd(const Chassis_cmd_send_t *c)
{
	if (chassis_cmd_pub)
		(void)Pub_push_message(chassis_cmd_pub, (void *)c);
}

// 读取一帧反馈（非阻塞）
static uint8_t try_read_feedback(Chassis_feedback_info_t *fb)
{
	if (!chassis_feedback_sub) return 0;
	return Sub_get_message(chassis_feedback_sub, (void *)fb);
}

// 测试流程：分阶段输出 vx/vy/wz，观察电机响应
// 备注：vx/vy 单位取决于底盘模块实现，建议从小值开始，逐步验证
void Chassis_motor_integration_test_task(void const *argument)
{
	// 变量声明置顶（兼容 C90/C89）
	unsigned int t_period_ms;  // 100Hz 发布周期
	unsigned int t_tick;
	unsigned int t_phase_ms;
	uint8_t t_phase;

	(void)argument;

	// 初始化 pub-sub
	test_pubsub_init();

	// 初始指令：零输入 + 不跟随模式
	cmd.vx = 0.0f;
	cmd.vy = 0.0f;
	cmd.wz = 0.0f;
	cmd.offset_angle = 0.0f;
	cmd.chassis_mode = CHASSIS_NO_FOLLOW;

	TEST_LOG("[TEST] Chassis-Motor integration test start.\r\n");
	TEST_LOG("[TEST] Phase plan: idle -> vx -> vy -> wz -> mix -> stop\r\n");
	TEST_LOG("[TEST] Tips: ensure Chassis_control_task(500Hz) & Motor_control_task(1kHz) are running.\r\n");

	// 时间基准（ms）
	t_period_ms = 10;  // 100Hz 发布
	t_tick = 0;
	t_phase_ms = 0;
	t_phase = 0;

	for (;;)
	{
		// 发布当前控制量
		publish_cmd(&cmd);

		// 读取反馈（如果有）
		if (try_read_feedback(&feedback)) {
			TEST_LOG("[FBK] wz=%.3f\r\n", feedback.chassis_wz);
		}

		// 状态机（每个阶段运行一定时间）
		switch (t_phase)
		{
		case 0: // Idle 2s
			if (t_phase_ms == 0) TEST_LOG("[PH0] idle 2s\r\n");
			if (t_phase_ms >= 2000) { t_phase = 1; t_phase_ms = 0; }
			break;
		case 1: // 直行 vx（小值） 3s
			if (t_phase_ms == 0) TEST_LOG("[PH1] vx=+0.2, vy=0, wz=0 for 3s\r\n");
			cmd.vx = 0.2f; cmd.vy = 0.0f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 2; t_phase_ms = 0; }
			break;
		case 2: // 横移 vy（小值） 3s
			if (t_phase_ms == 0) TEST_LOG("[PH2] vx=0, vy=+0.2, wz=0 for 3s\r\n");
			cmd.vx = 0.0f; cmd.vy = 0.2f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 3; t_phase_ms = 0; }
			break;
		case 3: // 原地小陀螺 wz（小值） 3s
			if (t_phase_ms == 0) TEST_LOG("[PH3] vx=0, vy=0, wz=+0.5 for 3s\r\n");
			cmd.vx = 0.0f; cmd.vy = 0.0f; cmd.wz = 0.5f;
			if (t_phase_ms >= 3000) { t_phase = 4; t_phase_ms = 0; }
			break;
		case 4: // 复合运动 3s
			if (t_phase_ms == 0) TEST_LOG("[PH4] vx=+0.2, vy=+0.2, wz=0 for 3s\r\n");
			cmd.vx = 0.2f; cmd.vy = 0.2f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 5; t_phase_ms = 0; }
			break;
		case 5: // 停止并保持
		default:
			if (t_phase_ms == 0) TEST_LOG("[PH5] stop, keep publishing zeros\r\n");
			cmd.vx = 0.0f; cmd.vy = 0.0f; cmd.wz = 0.0f;
			break;
		}

		// 限幅保护（若底层期望为 m/s & rad/s，可按需调整限幅）
		cmd.vx = clampf(cmd.vx, -1.0f, 1.0f);
		cmd.vy = clampf(cmd.vy, -1.0f, 1.0f);
		cmd.wz = clampf(cmd.wz, -2.0f, 2.0f);

		// 时间推进
	osDelay(t_period_ms);
	t_tick += t_period_ms;
	t_phase_ms += t_period_ms;
	}
}
