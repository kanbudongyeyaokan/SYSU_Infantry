// Shoot motors initialization and exposure

#include "shoot.h"
#include "bsp_can.h"

Djimotor_device_t *shoot_motors[3] = {0};

void Shoot_motors_init(void)
{
	Djimotor_init_config_t cfg[3] = {
		{
			.motor_name = "FRICTION_L",
			.motor_type = M3508,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = SPEED_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 3, .rx_id = 0x207}
		},{
			.motor_name = "FRICTION_R",
			.motor_type = M3508,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = SPEED_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x200, .tx_id = 4, .rx_id = 0x208}
		},{
			.motor_name = "LOADER",
			.motor_type = M2006,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = ANGLE_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x1FF, .tx_id = 3, .rx_id = 0x207}
		}
	};

	for (int i = 0; i < 3; i++) {
		shoot_motors[i] = DJI_Motor_Init(&cfg[i]);
	}
}