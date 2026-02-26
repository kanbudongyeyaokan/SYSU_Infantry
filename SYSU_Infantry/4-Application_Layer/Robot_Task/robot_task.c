#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "robot_task.h"
#include "bsp_usart.h"

#include "usart.h"

#include "queue.h"

// 测试任务头文件
#include <stdio.h>

#include "can_motors_test_task.h"
#include "rc_test_task.h"
#include "Decision_making_task.h"  // 添加决策任务头文件
#include "message_test_task.h"     // 添加消息中心测试任务头文件
#include "chassis_motor_integration_test.h"  // 添加底盘电机集成测试
#include "Chassis_task.h"
#include "decision_making.h"
#include "motor_task.h"
#include "watchdog_task.h"
#include "Gimbal_task.h"
#include "Ins_task.h"
#include "shoot_task.h"
#include "Shell_task.h"
#include "buzzer_alarm_task.h"
#include "buzzer_alarm.h"
#include "Referee_task.h"

  /**任务句柄声明**/
osThreadId chassis_task_handle;//底盘任务
osThreadId gimbal_task_handle; //云台任务
osThreadId shoot_task_handle;  //发射任务
osThreadId ins_task_handle;    //姿态解算任务
osThreadId decision_making_task_handle;     //决策任务
osThreadId referee_task_handle;//裁判系统通信任务
osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改
osThreadId watchdog_task_handle; //看门狗任务
osThreadId buzzer_alarm_task_handle;
/*Shell任务句柄*/
osThreadId shell_task_handle;

osThreadId bmi088_test_task_handle; //bmi088测试任务
osThreadId can_motors_test_task_handle; // can电机测试任务
osThreadId rc_test_task_handle; //单独遥控器测试任务
osThreadId chassis_motor_integration_test_handle; // 底盘电机集成测试任务
osThreadId motor_task_handle;

/**创建各个进程控制队列**/
QueueHandle_t Chassis_cmd_queue_handle;

QueueHandle_t Gimbal_cmd_queue_handle;

QueueHandle_t Shoot_cmd_queue_handle;

QueueHandle_t Buzzer_cmd_queue_handle;

QueueHandle_t Gimbal_feedback_queue_handle;

QueueHandle_t Chassis_feedback_queue_handle;\

QueueHandle_t shoot_feedback_queue_handle;

Uart_instance_t* test_uart = NULL;

/**机器人任务创建**/
void Robot_task_init(void)
{
    test_uart = Uart_register(&huart1,NULL);


    // 创建队列 (必须在任务创建之前!)
    // ============================================================
    // 深度为1，启用覆盖写模式，始终保持最新指令
    Chassis_cmd_queue_handle = xQueueCreate(1, sizeof(Chassis_cmd_send_t));

    Gimbal_cmd_queue_handle = xQueueCreate(1, sizeof(Gimbal_cmd_send_t));

    Shoot_cmd_queue_handle = xQueueCreate(1,sizeof(Shoot_cmd_send_t));

    Gimbal_feedback_queue_handle = xQueueCreate(1, sizeof(Gimbal_feedback_info_t));

    Chassis_feedback_queue_handle = xQueueCreate(1, sizeof(Chassis_feedback_info_t));

    shoot_feedback_queue_handle = xQueueCreate(1, sizeof(Shoot_feedback_info_t));

    Buzzer_cmd_queue_handle = xQueueCreate(5,sizeof(uint8_t));
    // 选择要运行的测试任务（取消注释需要的测试）
    
    // === 单元测试 ===
    //  osThreadDef(bmi088_test_task, Bmi088_test_task, osPriorityNormal, 0, 512);
    //  bmi088_test_task_handle = osThreadCreate(osThread(bmi088_test_task), NULL);


    // osThreadDef(can_motors_test_task, Can_motors_test_task, osPriorityNormal, 0, 512);
    // can_motors_test_task_handle = osThreadCreate(osThread(can_motors_test_task), NULL);

    // osThreadDef(rc_test_task, Rc_test_task, osPriorityNormal, 0, 512);
    // rc_test_task_handle = osThreadCreate(osThread(rc_test_task), NULL);

    // // === 3508电机开环测试 ===
    // osThreadDef(m3508_openloop_test_task, M3508_openloop_test_task, osPriorityNormal, 0, 512);
    // m3508_openloop_test_task_handle = osThreadCreate(osThread(m3508_openloop_test_task), NULL);

    // === 消息中心测试 ===
    // osThreadDef(message_test_task, Message_test_task, osPriorityNormal, 0, 1024);
    // message_test_task_handle = osThreadCreate(osThread(message_test_task), NULL);

    osThreadDef(ins_task, Ins_task, osPriorityHigh, 0, 4096);
    ins_task_handle = osThreadCreate(osThread(ins_task), NULL);
    //看门狗任务
  osThreadDef(watchdog_control_task, Watchdog_control_task, osPriorityHigh, 0, 512);
  watchdog_task_handle = osThreadCreate(osThread(watchdog_control_task), NULL);
  //   // 电机控制任务：1000Hz，聚合并通过 CAN 发送目标值
  osThreadDef(motor_control_task, Motor_control_task, osPriorityNormal, 0, 512);
  motor_task_handle = osThreadCreate(osThread(motor_control_task), NULL);
  //
  //   // 添加短暂延时，让任务有时间初始化
  //  // osDelay(100);
  //
  //   //决策任务
  osThreadDef(decision_making_task,Decision_making_task,osPriorityNormal,0,1024);
  decision_making_task_handle = osThreadCreate(osThread(decision_making_task), NULL);

    // === 启动底盘与电机任务（必需） ===
    // 底盘控制任务：500Hz，接收决策层/测试发布的 chassis_cmd，解算并写入电机目标
    osThreadDef(chassis_control_task, Chassis_control_task, osPriorityNormal, 0, 512);
    chassis_task_handle = osThreadCreate(osThread(chassis_control_task), NULL);

    osThreadDef(gimbal_control_task, Gimbal_control_task, osPriorityNormal, 0, 512);
    gimbal_task_handle = osThreadCreate(osThread(gimbal_control_task), NULL);

    // osThreadDef(shoot_control_task, Shoot_control_task, osPriorityNormal, 0, 512);
    // shoot_task_handle = osThreadCreate(osThread(shoot_control_task), NULL);

    osThreadDef(buzzer_alarm_task, Buzzer_alarm_control_task, osPriorityBelowNormal, 0, 512);
    buzzer_alarm_task_handle = osThreadCreate(osThread(buzzer_alarm_task), NULL);
}


