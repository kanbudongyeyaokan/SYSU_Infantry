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
#define ONE_BULLET_DELTA_ANGLE  36.0f
#define REDUCTION_RATIO_LOADER 36.0f
/****************接收决策层的射击控制信息********************/
// 订阅决策层发来的底盘控制指令
static Subscriber_t *shoot_cmd_sub;
// 存储决策层发来的控制命令
static Shoot_cmd_send_t shoot_cmd_recv;
static float M2006_last_angle;
static loader_mode_e loader_last_mode;
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
            .deadzone_compensation = 500,
			.motor_controller_init = {
				.close_loop = SPEED_LOOP,
				.speed_source = MOTOR_FEEDBACK,
				.speed_pid = {
                .kp = 22, // 20
                .ki = 1, // 1
                .kd = 0.1,
                .max_iout = 2000,
                .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 10000,
				
            },
			
            .current_pid = {
                .kp = 0.7, // 0.7
                .ki = 0.1, // 0.1
                .kd = 0,
				.max_iout = 2000,
                .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 10000,
                
            },
			},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 1, .rx_id = 0x201}
		},{
			.motor_name = "FRICTION_R",
			.motor_type = M3508,
			.motor_status = MOTOR_ENABLED,
            .deadzone_compensation = 500,
            .motor_controller_init = {
				.close_loop = SPEED_LOOP,
				.speed_source = MOTOR_FEEDBACK,
			.speed_pid = {
                .kp = 22, // 20
                .ki = 1, // 1
                .kd = 0.1,
                .max_iout = 2000,
                .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 10000,
				
            },
			
            .current_pid = {
                .kp = 0.7, // 0.7
                .ki = 0.1, // 0.1
                .kd = 0.1,
				.max_iout = 2000,
                .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 10000,
                
            },
		},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
		},{
			.motor_name = "LOADER",
			.motor_type = M2006,
			.motor_status = MOTOR_STOP,
            .deadzone_compensation = 100,
            .motor_controller_init = {
				.close_loop = SPEED_LOOP,
				.angle_source = MOTOR_FEEDBACK,
				.speed_source = MOTOR_FEEDBACK,
				 .angle_pid = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .kp = 10, // 10
                .ki = 0.5,
                .kd = 5,
              .max_iout = 1000,
              .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 2000,
            },
            .speed_pid = {
                .kp = 10, // 10
                .ki = 0.5, // 1
                .kd = 1,
               .max_iout = 1500,
                .optimization = PID_TRAPEZOID_INTERGRAL | PID_OUTPUT_LIMIT | PID_DIFFERENTIAL_GO_FIRST,
                .max_out = 2000,
            },
			},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 3, .rx_id = 0x203}
		}
	};

	for (int i = 0; i < 3; i++) {
		shoot_motors[i] = DJI_Motor_Init(&cfg[i]);
	}
    shoot_cmd_recv.loader_mode=LOAD_STOP;
    loader_last_mode=LOAD_STOP;
    shoot_cmd_recv.shoot_mode = SHOOT_OFF;
}

//发射任务初始化
void Shoot_task_init(void) {
	Shoot_motors_init();
    
	shoot_cmd_sub = Sub_register("shoot_cmd",sizeof(Shoot_cmd_send_t));
	shoot_feedback_pub = Pub_register("shoot_feedback",sizeof(Shoot_feedback_info_t));
}

//发射任务处理控制命令
void Shoot_handle_command(void) {
	// 从消息中心获取最新的发射机构控制指令

	/*仅仅做测试用，后续修改或删除*/
	if (Sub_get_message(shoot_cmd_sub, &shoot_cmd_recv)) {
		if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
			//关闭摩擦轮
			Djimotor_set_status(shoot_motors[0], MOTOR_STOP);
			Djimotor_set_status(shoot_motors[1], MOTOR_STOP);
			Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
			Djimotor_set_target(shoot_motors[0],0);
			Djimotor_set_target(shoot_motors[1],0);
			Djimotor_set_target(shoot_motors[2],0);

		}
		else
		{
			//开启摩擦轮
			Djimotor_set_status(shoot_motors[0], MOTOR_ENABLED);
			Djimotor_set_status(shoot_motors[1], MOTOR_ENABLED);
			Djimotor_set_target(shoot_motors[0],2500);//上
			Djimotor_set_target(shoot_motors[1], -2500);//下
		}

		switch (shoot_cmd_recv.loader_mode)
		{
			// 停止拨盘
			case LOAD_STOP:
				// 切换到速度环
				shoot_motors[2]->motor_pid.close_loop =  SPEED_LOOP;
				Djimotor_set_target(shoot_motors[2],0);         // 同时设定参考值为0,这样停止的速度最快
				Djimotor_set_status(shoot_motors[2], MOTOR_STOP);

				break;
				// 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
			case LOAD_1_BULLET:
				if(loader_last_mode == shoot_cmd_recv.loader_mode)   {
					break; // 如果上次模式和这次一样,说明是持续按下,不做处理
				}

				else{
					shoot_motors[2]->motor_pid.close_loop =  ANGLE_LOOP;                                             // 切换到角度环
				   Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);

                   Djimotor_set_target(shoot_motors[2], shoot_motors[2]->motor_measure.current_angle - ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
                
					break;
					} 
				// 连发模式,对速度闭环,射频后续修改为可变,目前固定为80发/min
			case LOAD_BURSTFIRE:
				shoot_motors[2]->motor_pid.close_loop =  SPEED_LOOP;                                             // 切换到角度环
				Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
				Djimotor_set_target(shoot_motors[2], -(360/10)*(shoot_cmd_recv.shoot_rate*10)*REDUCTION_RATIO_LOADER /360);
				break;
				// x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 10
				
				// 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)-(shoot_cmd_recv.shoot_rate * 360  )/ 1
				// 也有可能需要从switch-case中独立出来
				/*  case LOAD_REVERSE:
						shoot_motors[2]->motor_pid.close_loop =  SPEED_LOOP;                                             // 切换到角度环
						Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
						Djimotor_set_target(shoot_motors[2], -5);
				//     // ...
					 break; */
			 // 未知模式,停止运行,检查指针越界,内存溢出等问题
			default:
				shoot_motors[2]->motor_pid.close_loop =  SPEED_LOOP;
				Djimotor_set_status(shoot_motors[0], MOTOR_STOP);
				Djimotor_set_status(shoot_motors[1], MOTOR_STOP);
				Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
				Djimotor_set_target(shoot_motors[0],0);
				Djimotor_set_target(shoot_motors[1],0);
				Djimotor_set_target(shoot_motors[2],0);
				break;


		}

		loader_last_mode = shoot_cmd_recv.loader_mode;
		Pub_push_message(shoot_feedback_pub, (void *)& shoot_feedback);
	}
}
