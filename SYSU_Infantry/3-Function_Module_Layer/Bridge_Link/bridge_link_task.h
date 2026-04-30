/**
 * @file    bridge_link_task.h
 * @brief   Bridge Link FreeRTOS 任务声明
 */

#ifndef BRIDGE_LINK_TASK_H
#define BRIDGE_LINK_TASK_H

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Bridge Link 主任务入口（传入 FreeRTOS 任务创建函数）
 */
void Bridge_Link_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_LINK_TASK_H */
