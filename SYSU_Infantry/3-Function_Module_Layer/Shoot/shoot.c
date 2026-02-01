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
		},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 2, .rx_id = 0x202}
		},{
			.motor_name = "LOADER",
			.motor_type = M2006,
			.motor_status = MOTOR_STOP,
            .deadzone_compensation = 100,
            .motor_controller_init = {
				.close_loop = ANGLE_AND_SPEED_LOOP,
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

	shoot_feedback_pub = Pub_register("shoot_feedback",sizeof(Shoot_feedback_info_t));
}

//发射任务处理控制命令
void Shoot_handle_command(Shoot_cmd_send_t *cmd) {

    // 1. 处理摩擦轮 (SHOOT_MODE)
    if (cmd->shoot_mode == SHOOT_OFF) {
        // 关闭摩擦轮
        Djimotor_set_status(shoot_motors[0], MOTOR_STOP);
        Djimotor_set_status(shoot_motors[1], MOTOR_STOP);
        Djimotor_set_target(shoot_motors[0], 0);
        Djimotor_set_target(shoot_motors[1], 0);
    }
    else {
        // 开启摩擦轮
        Djimotor_set_status(shoot_motors[0], MOTOR_ENABLED);
        Djimotor_set_status(shoot_motors[1], MOTOR_ENABLED);
        Djimotor_set_target(shoot_motors[0], 2500);  // 上摩擦轮
        Djimotor_set_target(shoot_motors[1], -2500); // 下摩擦轮
    }

    // 2. 处理拨弹盘 (LOADER_MODE)
    switch (cmd->loader_mode)
    {
        case LOAD_STOP:
            shoot_motors[2]->motor_pid.close_loop = SPEED_LOOP;
            Djimotor_set_target(shoot_motors[2], 0);
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            break;

        case LOAD_1_BULLET: // 单发
            // 边沿检测：只有当模式发生变化时才触发
            if (loader_last_mode != cmd->loader_mode) {
                shoot_motors[2]->motor_pid.close_loop = ANGLE_LOOP;
                Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
                Djimotor_set_target(shoot_motors[2], shoot_motors[2]->motor_measure.current_angle - ONE_BULLET_DELTA_ANGLE);
            }
            break;

        case LOAD_BURSTFIRE: // 连发
            shoot_motors[2]->motor_pid.close_loop = SPEED_LOOP;
            Djimotor_set_status(shoot_motors[2], MOTOR_ENABLED);
            // 注意：这里引用 cmd->shoot_rate
            Djimotor_set_target(shoot_motors[2], -cmd->shoot_rate * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
            break;

        default:
            Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
            break;
    }

    // 更新上次模式
    loader_last_mode = cmd->loader_mode;

    // 3. [核心] 算发分离：计算 PID
    // 摩擦轮
    Djimotor_Calc_Output(shoot_motors[0]);
    Djimotor_Calc_Output(shoot_motors[1]);
    // 拨弹盘
    Djimotor_Calc_Output(shoot_motors[2]);

    // 其他反馈赋值
    Pub_push_message(shoot_feedback_pub, (void *)&shoot_feedback);
}