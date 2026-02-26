// 任务头文件
#include "Decision_making_task.h"

// API调用
#include <stdio.h>

#include "bsp_dwt.h"
#include "decision_making.h"

#include "buzzer_driver.h"
#include "buzzer_music.h"
#include "main.h"
#include "tim.h"

// static Buzzer_device_t test_buzzer={0};

/**
 * @brief   机器人决策任务主体
 * @note    500Hz频率运行 (2ms周期)。
 *          主要流程：
 *          1. 接收反馈 (Receive_feedback_infomation)
 *          2. 决策计算 (Robot_set_command)
 *          3. 解算云台/底盘夹角 (Calc_offset_angle)
 *          4. 分发指令 (Send_command_to_all_task)
 */
void Decision_making_task() {
  // 任务初始化
  Decision_making_task_init();
  /*****测试蜂鸣器******/
  // Buzzer_init(&test_buzzer,&htim4,TIM_CHANNEL_3);
  for (;;) {
    // 接收应用层的反馈数据
    Receive_feedback_infomation();
    // 决策要发的控制信息
    Robot_set_command();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角
    Calc_offset_angle();
    // 向各个应用层传送控制信息
    Send_command_to_all_task();
    // 控制频率200HZ
    osDelay(1);
  }
}
