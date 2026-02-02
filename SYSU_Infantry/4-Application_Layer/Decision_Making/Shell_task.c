/**
* @file    Shell_task.c
 * @brief   Shell 任务入口
 */

#include "Shell_task.h"
#include "shell_port.h"
#include "cmsis_os.h"  // 对应 osDelay 等函数

/**
 * @brief Shell 任务主体
 * @param argument 任务参数 (通常未用)
 */
void Shell_task(void const * argument)
{
    // 1. 硬件/底层初始化
    // 确保此时 robot_task.c 中的 test_uart 已经注册完毕
    User_Shell_Init();

    // 2. 进入 Letter Shell 的自动托管循环
    // shellTask() 内部是一个 while(1) 循环
    // 它会调用 userShellRead() 阻塞等待数据，收到数据后解析命令
    shellTask(&shell);

    // 理论上不会运行到这里，除非 shellTask 返回
    for(;;)
    {
        osDelay(1000);
    }
}