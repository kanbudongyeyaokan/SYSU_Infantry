/**
* @file    .c
 * @brief   发射机构功能模块源文件
 * @author  SYSU电控组
 * @date    2025-09-27
 * @version 1.0
 *
 * @note    发射机构电机初始化
 */

#include "shoot.h"
#include "bsp_can.h"
#include "message_center.h"
#include "decision_making.h"

/****************接收决策层的射击控制信息********************/
// 订阅决策层发来的底盘控制指令
static Subscriber_t *shoot_cmd_sub;
// 存储决策层发来的控制命令
static Shoot_cmd_send_t shoot_cmd_recv;

/****************发送给决策层的射击反馈信息******************/
// 发布给决策层的射击反馈信息
static Publisher_t *shoot_feedback_pub;
// 存储发送给决策层的反馈信息
static Shoot_feedback_info_t shoot_feedback;

/****************发射机构电机实例**************************/
static Djimotor_device_t *shoot_motors[3] = {0};

void Shoot_motors_init(void)
{
	Djimotor_init_config_t cfg[3] = {
		{
			.motor_name = "FRICTION_L",
			.motor_type = M3508,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = OPEN_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
		},{
			.motor_name = "FRICTION_R",
			.motor_type = M3508,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = OPEN_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
		},{
			.motor_name = "LOADER",
			.motor_type = M2006,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = OPEN_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
		}
	};

	for (int i = 0; i < 3; i++) {
		shoot_motors[i] = DJI_Motor_Init(&cfg[i]);
	}
}

//发射任务初始化
void Shoot_task_init(void) {
	Shoot_motors_init();
	shoot_cmd_sub = Sub_register("shoot_cmd",sizeof(Shoot_cmd_send_t));
	shoot_feedback_pub = Pub_register("shoot_feedback",sizeof(Shoot_feedback_info_t));
}

//发射任务处理控制命令
void Shoot_handle_command(void)
{
	// 从消息中心获取最新的发射机构控制指令

	/*仅仅做测试用，后续修改或删除*/
	if (Sub_get_message(shoot_cmd_sub, &shoot_cmd_recv))
	{
		if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
			//关闭摩擦轮
			Djimotor_set_status(shoot_motors[0], MOTOR_STOP);
			Djimotor_set_status(shoot_motors[1], MOTOR_STOP);
			Djimotor_set_target(shoot_motors[0],0);
			Djimotor_set_target(shoot_motors[1],0);
		}
		else
		{
			//开启摩擦轮
			Djimotor_set_status(shoot_motors[0], MOTOR_ENABLED);
			Djimotor_set_status(shoot_motors[1], MOTOR_ENABLED);
			Djimotor_set_target(shoot_motors[0],1000);
			Djimotor_set_target(shoot_motors[1],1000);
		}

		if (shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE) {
			Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
			Djimotor_set_target(shoot_motors[2],1000);
		}


	}
}