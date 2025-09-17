//任务头文件
#include "Decision_Making_task.h"

//API调用
#include "decision_making.h"


/**
 * @brief 机器人决策任务,500Hz频率运行
 *
 */
void Decision_making_task()
{
    //任务初始化
    Decision_making_task_init();

    for (;;)
    {
        Receive_feedback_infomation();
        Robot_set_command();
        Send_command_to_all_task();
        osDelay(2);//控制频率500HZ
    }

}



