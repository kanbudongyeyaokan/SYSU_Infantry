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
        //接收应用层的反馈数据
        Receive_feedback_infomation();
        //决策要发的控制信息
        Robot_set_command();
        // 根据gimbal的反馈值计算云台和底盘正方向的夹角
        Calc_offset_angle();
        //向各个应用层传送控制信息
        Send_command_to_all_task();
        //控制频率500HZ
        osDelay(2);
    }

}



