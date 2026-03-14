#ifndef SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H
#define SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H

#include "chassis.h"
#include "dji_motor.h"

void Chassis_Power_Control_Init(void);
void Chassis_Power_Control(Chassis_output_t *output, Djimotor_device_t *motors[4]);
void Send2SuperCap(void);

#endif // SYSU_INFANTRY_CHASSIS_POWER_CONTROL_H
