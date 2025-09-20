// Minimal Gimbal task scaffold and optional init

#include "Gimbal_task.h"
#include "gimbal.h"

void Gimbal_task_entry(void const *argument)
{
	// 初始化云台电机
	Gimbal_motors_init();
	for(;;){
		// TODO: 根据指令设置目标值（静态缓冲区）
		osDelay(5);
	}
}
