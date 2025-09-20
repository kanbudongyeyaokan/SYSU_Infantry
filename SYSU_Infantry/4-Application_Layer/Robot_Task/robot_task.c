#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "robot_task.h"
#include "bsp_usart.h"

#include "usart.h"


// 测试任务头文件
#include "bmi088_test_task.h"
#include "can_motors_test_task.h"
#include "rc_test_task.h"
#include "Decision_making_task.h"  // 添加决策任务头文件
#include "message_test_task.h"     // 添加消息中心测试任务头文件
#include "chassis_motor_integration_test.h"  // 添加底盘电机集成测试
#include "Chassis_task.h"
#include "motor_task.h"

/**任务句柄声明**/
osThreadId chassis_task_handle;//底盘任务
osThreadId gimbal_task_handle; //云台任务
osThreadId shoot_task_handle;  //发射任务
osThreadId ins_task_handle;    //姿态解算任务
osThreadId decision_making_task_handle;     //决策任务
osThreadId referee_task_handle;//裁判系统通信任务
osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改

osThreadId bmi088_test_task_handle; //bmi088测试任务
osThreadId can_motors_test_task_handle; // can电机测试任务
osThreadId rc_test_task_handle; //单独遥控器测试任务
osThreadId chassis_motor_integration_test_handle; // 底盘电机集成测试任务
osThreadId motor_task_handle;


/**机器人任务创建**/
void Robot_task_init(void)
{


    // 选择要运行的测试任务（取消注释需要的测试）
    
    // === 单元测试 ===
    //  osThreadDef(bmi088_test_task, Bmi088_test_task, osPriorityNormal, 0, 512);
    //  bmi088_test_task_handle = osThreadCreate(osThread(bmi088_test_task), NULL);

    // osThreadDef(can_motors_test_task, Can_motors_test_task, osPriorityNormal, 0, 512);
    // can_motors_test_task_handle = osThreadCreate(osThread(can_motors_test_task), NULL);

    // osThreadDef(rc_test_task, Rc_test_task, osPriorityNormal, 0, 512);
    // rc_test_task_handle = osThreadCreate(osThread(rc_test_task), NULL);

    // === 消息中心测试 ===
    // osThreadDef(message_test_task, Message_test_task, osPriorityNormal, 0, 1024);
    // message_test_task_handle = osThreadCreate(osThread(message_test_task), NULL);

    // === 集成测试 ===
    printf("[INIT][Robot] Creating chassis_motor_integration_test_task...\r\n");
    printf("[INIT][Robot] Free heap before test task: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
    osThreadDef(chassis_motor_integration_test_task, Chassis_motor_integration_test_task, osPriorityNormal, 0, 2048);
    chassis_motor_integration_test_handle = osThreadCreate(osThread(chassis_motor_integration_test_task), NULL);
    if (chassis_motor_integration_test_handle == NULL) {
        printf("[ERROR][Robot] Failed to create chassis_motor_integration_test_task\r\n");
    } else {
        printf("[SUCCESS][Robot] chassis_motor_integration_test_task created successfully\r\n");
    }
    
    // 添加短暂延时，让任务有时间初始化
    osDelay(100);

    // === 启动底盘与电机任务（必需） ===
    // 底盘控制任务：500Hz，接收决策层/测试发布的 chassis_cmd，解算并写入电机目标
    printf("[INIT][Robot] Creating chassis_control_task...\r\n");
    printf("[INIT][Robot] Free heap before chassis task: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
    osThreadDef(chassis_control_task, Chassis_control_task, osPriorityNormal, 0, 2048);
    chassis_task_handle = osThreadCreate(osThread(chassis_control_task), NULL);
    if (chassis_task_handle == NULL) {
        printf("[ERROR][Robot] Failed to create chassis_control_task\r\n");
    } else {
        printf("[SUCCESS][Robot] chassis_control_task created successfully\r\n");
    }
    
    // 添加短暂延时
    osDelay(100);

    // 电机控制任务：1000Hz，聚合并通过 CAN 发送目标值
    printf("[INIT][Robot] Creating motor_control_task...\r\n");
    printf("[INIT][Robot] Free heap before motor task: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
    osThreadDef(motor_control_task, Motor_control_task, osPriorityNormal, 0, 512);
    motor_task_handle = osThreadCreate(osThread(motor_control_task), NULL);
    if (motor_task_handle == NULL) {
        printf("[ERROR][Robot] Failed to create motor_control_task\r\n");
    } else {
        printf("[SUCCESS][Robot] motor_control_task created successfully\r\n");
    }


    // osThreadDef(rc_test_task, Rc_test_task, osPriorityNormal, 0, 512);
    // rc_test_task_handle = osThreadCreate(osThread(rc_test_task), NULL);
}


