#ifndef SYSU_INFANTRY_INS_TASK_H
#define SYSU_INFANTRY_INS_TASK_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief          ins_task
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Ins_task(void const *argument);

#endif //SYSU_INFANTRY_INS_TASK_H