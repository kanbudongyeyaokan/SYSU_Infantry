#ifndef _DECISION_MAKING_TASK_H
#define _DECISION_MAKING_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


/**
 * @brief 机器人决策任务,500Hz频率运行
 * @param argument 任务参数（FreeRTOS标准参数）
 */
void Decision_making_task();


#endif //_DECISION_MAKING_TASK_H