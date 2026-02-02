#ifndef _SHELL_TASK_H
#define _SHELL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

    // 任务入口函数 (符合 CMSIS-RTOS v1 标准)
    void Shell_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif // _SHELL_TASK_H