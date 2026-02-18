#ifndef SYSU_INFANTRY_REFEREE_TASK_H
#define SYSU_INFANTRY_REFEREE_TASK_H

/* 裁判系统 FreeRTOS 任务入口，负责 UART 初始化与数据接收 */
extern void Referee_task(void const *argument);

#endif //SYSU_INFANTRY_REFEREE_TASK_H
