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

// 本地运行状态统计
static unsigned long s_pub_frames = 0;       // 已发布帧数
static unsigned long s_pub_zero_deliver = 0; // 连续“无订阅者接收”次数
static unsigned long s_fbk_no_data = 0;      // 连续“无新反馈”次数
static uint8_t  s_phase_last_logged = 0xFF;// 上一次打印的阶段编号

// 打印辅助：使用 printf 重定向（需在工程中已完成 printf 的底层重定向）
#define TEST_LOG(...)            \
	do {                        \
		printf(__VA_ARGS__);    \
	} while (0)

// 简易分隔与标题，优化串口可读性
#define LOG_LINE()                  TEST_LOG("------------------------------------------------------------\r\n")
#define LOG_SECTION(title)          do { LOG_LINE(); TEST_LOG("[SECTION] %s\r\n", (title)); LOG_LINE(); } while (0)
#define LOG_SUBSECTION(title)       do { TEST_LOG("---------------- %s ----------------\r\n", (title)); } while (0)

// 简单的限幅
static float clampf(float v, float lo, float hi) {
	return (v < lo) ? lo : (v > hi ? hi : v);
}

// 初始化发布/订阅
static void test_pubsub_init(void)
{
	LOG_SECTION("PUB-SUB INIT");
	// 与 Chassis_task 保持一致的话题名
	chassis_cmd_pub = Pub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
	chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));

	// 详细信息打印（包含指针与内部字段，辅助定位是否真的被注册与是否存在订阅者）
	TEST_LOG("[TEST] pub-sub ready: cmd->'chassis_cmd' size=%d, sub<-'chassis_feedback' size=%d\r\n",
			 (int)sizeof(Chassis_cmd_send_t), (int)sizeof(Chassis_feedback_info_t));
	TEST_LOG("[TEST] pub  ptr=%p, sub ptr=%p\r\n", (void*)chassis_cmd_pub, (void*)chassis_feedback_sub);
	if (chassis_cmd_pub) {
		TEST_LOG("[TEST] pub  internals: data_len=%u, first_subs=%p, registered=%u\r\n",
				(unsigned)chassis_cmd_pub->data_len,
				(void*)chassis_cmd_pub->first_subs,
				(unsigned)chassis_cmd_pub->pub_registered_flag);
	} else {
		TEST_LOG("[WARN] Pub_register('chassis_cmd') returned NULL. Topic create failed.\r\n");
	}
	if (chassis_feedback_sub) {
		TEST_LOG("[TEST] sub  internals: data_len=%u, next_subs_queue=%p\r\n",
				(unsigned)chassis_feedback_sub->data_len,
				(void*)chassis_feedback_sub->next_subs_queue);
	} else {
		TEST_LOG("[WARN] Sub_register('chassis_feedback') returned NULL. Topic subscribe failed.\r\n");
	}
	TEST_LOG("[HINT] If delivered=0 later: ensure 'Chassis_task' is running and topic names match exactly.\r\n");
	LOG_LINE();
}

// 发布一帧底盘控制指令（频率建议>=100Hz）
static void publish_cmd(const Chassis_cmd_send_t *c)
{
	if (!chassis_cmd_pub) {
		// 仅首次或偶尔提示，避免刷屏
		static uint8_t warned = 0;
		if (!warned) {
			LOG_SUBSECTION("PUBLISH ERROR");
			TEST_LOG("[ERR ] publish: chassis_cmd_pub=NULL. Pub_register failed or not called.\r\n");
			warned = 1;
		}
		return;
	}
	uint8_t delivered = Pub_push_message(chassis_cmd_pub, (void *)c);
	s_pub_frames++;
	if (delivered == 0) {
		s_pub_zero_deliver++;
		// 若持续 1s（约100帧）都没人接收，则输出一次详细提示
		if ((s_pub_zero_deliver % 100) == 1) {
				LOG_SUBSECTION("PUBLISH WARN: NO SUBSCRIBER");
			TEST_LOG("[WARN] publish delivered=0 (no subscribers). Frames=%lu, vx=%.3f vy=%.3f wz=%.3f\r\n",
					(unsigned long)s_pub_frames, c->vx, c->vy, c->wz);
			TEST_LOG("[HINT] Check: Chassis_task Sub_register('chassis_cmd') started? Topic name exact?\r\n");
		}
	} else {
		if (s_pub_zero_deliver) {
				LOG_SUBSECTION("PUBLISH INFO: RECOVERED");
			TEST_LOG("[INFO] publish delivered=%u, recovered from previous no-subscriber state (streak=%lu).\r\n",
					(unsigned)delivered, (unsigned long)s_pub_zero_deliver);
			s_pub_zero_deliver = 0;
		}
	}
}

// 读取一帧反馈（非阻塞）
static uint8_t try_read_feedback(Chassis_feedback_info_t *fb)
{
	if (!chassis_feedback_sub) return 0;
	uint8_t got = Sub_get_message(chassis_feedback_sub, (void *)fb);
	if (got) {
		// 收到反馈，清零计数并打印要点
		s_fbk_no_data = 0;
		TEST_LOG("[FBK] wz=%.3f\r\n", fb->chassis_wz);
	} else {
		s_fbk_no_data++;
		// 每 1s 提醒一次无反馈
		if ((s_fbk_no_data % 100) == 1) {
				LOG_SUBSECTION("NO FEEDBACK");
			TEST_LOG("[INFO] no new chassis_feedback yet (streak=%lu).\r\n", (unsigned long)s_fbk_no_data);
			TEST_LOG("[HINT] Ensure Chassis_task publishes 'chassis_feedback' periodically.\r\n");
		}
	}
	return got;
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
	TEST_LOG("[HINT] No CAN motors connected is OK for this test; expect no physical motion.\r\n");
	LOG_LINE();

	// 时间基准（ms）
	t_period_ms = 10;  // 100Hz 发布
	t_tick = 0;
	t_phase_ms = 0;
	t_phase = 0;

	for (;;)
	{
		// 发布当前控制量
		publish_cmd(&cmd);

		// 读取反馈（如果有）。内部已带节流日志
		(void)try_read_feedback(&feedback);

		// 状态机（每个阶段运行一定时间）
		switch (t_phase)
		{
		case 0: // Idle 2s
			if (t_phase_ms == 0) {
		LOG_SECTION("PHASE 0: IDLE");
				TEST_LOG("[PH0] idle 2s (keep zeros).\r\n");
				s_phase_last_logged = 0;
			}
			if (t_phase_ms >= 2000) { t_phase = 1; t_phase_ms = 0; }
			break;
		case 1: // 直行 vx（小值） 3s
			if (t_phase_ms == 0) {
		LOG_SECTION("PHASE 1: VX");
				TEST_LOG("[PH1] enter: vx=+0.2, vy=0, wz=0 for 3s\r\n");
				s_phase_last_logged = 1;
			}
			cmd.vx = 0.2f; cmd.vy = 0.0f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 2; t_phase_ms = 0; }
			break;
		case 2: // 横移 vy（小值） 3s
			if (t_phase_ms == 0) {
		LOG_SECTION("PHASE 2: VY");
				TEST_LOG("[PH2] enter: vx=0, vy=+0.2, wz=0 for 3s\r\n");
				s_phase_last_logged = 2;
			}
			cmd.vx = 0.0f; cmd.vy = 0.2f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 3; t_phase_ms = 0; }
			break;
		case 3: // 原地小陀螺 wz（小值） 3s
			if (t_phase_ms == 0) {
		LOG_SECTION("PHASE 3: WZ");
				TEST_LOG("[PH3] enter: vx=0, vy=0, wz=+0.5 for 3s\r\n");
				s_phase_last_logged = 3;
			}
			cmd.vx = 0.0f; cmd.vy = 0.0f; cmd.wz = 0.5f;
			if (t_phase_ms >= 3000) { t_phase = 4; t_phase_ms = 0; }
			break;
		case 4: // 复合运动 3s
			if (t_phase_ms == 0) {
		LOG_SECTION("PHASE 4: MIX");
				TEST_LOG("[PH4] enter: vx=+0.2, vy=+0.2, wz=0 for 3s\r\n");
				s_phase_last_logged = 4;
			}
			cmd.vx = 0.2f; cmd.vy = 0.2f; cmd.wz = 0.0f;
			if (t_phase_ms >= 3000) { t_phase = 5; t_phase_ms = 0; }
			break;
		case 5: // 停止并保持
		default:
			if (t_phase_ms == 0 && s_phase_last_logged != 5) {
		LOG_SECTION("PHASE 5: STOP");
				TEST_LOG("[PH5] enter: stop, keep publishing zeros (hold)\r\n");
				s_phase_last_logged = 5;
			}
			cmd.vx = 0.0f; cmd.vy = 0.0f; cmd.wz = 0.0f;
			break;
		}

		// 限幅保护（若底层期望为 m/s & rad/s，可按需调整限幅）
		cmd.vx = clampf(cmd.vx, -1.0f, 1.0f);
		cmd.vy = clampf(cmd.vy, -1.0f, 1.0f);
		cmd.wz = clampf(cmd.wz, -2.0f, 2.0f);

		// 心跳：每 1s 打印一次当前状态（阶段、命令、计数），辅助确认任务未阻塞
		if ((t_phase_ms % 1000) == 0) {
                LOG_SUBSECTION("HEARTBEAT");
			TEST_LOG("[HB ] t=%lums, phase=%u, cmd(vx=%.3f vy=%.3f wz=%.3f), pub_frames=%lu\r\n",
					(unsigned long)t_tick, (unsigned)t_phase, cmd.vx, cmd.vy, cmd.wz,
					(unsigned long)s_pub_frames);
#if (INCLUDE_uxTaskGetStackHighWaterMark)
			{
				UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
				TEST_LOG("[HB ] stack watermark=%lu (words)\r\n", (unsigned long)watermark);
			}
#endif
		}

		// 时间推进
	osDelay(t_period_ms);
	t_tick += t_period_ms;
	t_phase_ms += t_period_ms;
	}
}
