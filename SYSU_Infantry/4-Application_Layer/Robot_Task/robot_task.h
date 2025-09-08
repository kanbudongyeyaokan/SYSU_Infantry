#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//机器人任务创建
void Robot_task_init(void);

#endif //_ROBOT_TASK_H