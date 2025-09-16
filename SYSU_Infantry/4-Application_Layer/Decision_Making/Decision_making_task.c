//任务头文件
#include "Decision_making_task.h"

//API调用

/**
 * @brief 机器人决策任务初始化
 */
void Decision_making_init()
{
    // TODO: 添加决策任务初始化代码
}

/**
 * @brief 机器人决策任务,500Hz频率运行
 * @param argument 任务参数（FreeRTOS标准参数）
 */
void Decision_making_task(void const *argument)
{
    // 任务初始化
    Decision_making_init();
    
    for (;;)
    {
        // TODO: 添加决策逻辑代码
        
        // 以500Hz频率运行，延时2ms
        osDelay(2);
    }
}



