/**
 * @file    ui_task.h
 * @brief   UI task header
 */

#ifndef UI_TASK_H
#define UI_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void Ui_task(void const *argument);

#endif // UI_TASK_H
