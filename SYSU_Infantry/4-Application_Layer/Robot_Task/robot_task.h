#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//任务句柄外部声明
extern osThreadId chassis_task_handle; //底盘任务
extern osThreadId gimbal_task_handle; //云台任务
extern osThreadId shoot_task_handle; //发射任务
extern osThreadId ins_task_handle; //姿态解算任务
extern osThreadId rc_task_handle; //遥控器/键盘解析任务
extern osThreadId referee_task_handle; //裁判系统通信任务
extern osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改
extern osThreadId watchdog_task_handle;

extern osThreadId bmi088_test_task_handle; //bmi088测试任务
extern osThreadId can_motors_test_task_handle; // can电机测试任务
extern osThreadId rc_test_task_handle; //单独遥控器测试任务

/*队列*/
extern QueueHandle_t Chassis_cmd_queue_handle;


//机器人任务创建
void Robot_task_init(void);

#endif //_ROBOT_TASK_H
