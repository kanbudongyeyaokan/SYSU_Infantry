// Gimbal motors initialization and exposure

#include "gimbal.h"
#include "bsp_can.h"

Djimotor_device_t *gimbal_motors[2] = {0};

void Gimbal_motors_init(void)
{
	Djimotor_init_config_t cfg[2] = {
		{
			.motor_name = "GIMBAL_YAW",
			.motor_type = GM6020,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = ANGLE_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x1FF, .tx_id = 1, .rx_id = 0x205}
		},{
			.motor_name = "GIMBAL_PITCH",
			.motor_type = GM6020,
			.motor_status = MOTOR_ENABLED,
			.motor_controller_init = {.close_loop = ANGLE_LOOP},
			.can_init = {.can_handle = &hcan2, .can_id = 0x1FF, .tx_id = 2, .rx_id = 0x206}
		}
	};

	for (int i = 0; i < 2; i++) {
		gimbal_motors[i] = DJI_Motor_Init(&cfg[i]);
	}
}