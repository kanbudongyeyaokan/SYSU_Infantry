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

/**任务句柄声明**/
osThreadId chassis_task_handle;//底盘任务
osThreadId gimbal_task_handle; //云台任务
osThreadId shoot_task_handle;  //发射任务
osThreadId ins_task_handle;    //姿态解算任务
osThreadId rc_task_handle;     //遥控器/键盘解析任务
osThreadId referee_task_handle;//裁判系统通信任务
osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改

osThreadId bmi088_test_task_handle; //bmi088测试任务
osThreadId can_motors_test_task_handle; // can电机测试任务
osThreadId rc_test_task_handle; //单独遥控器测试任务

/**机器人任务创建**/
void Robot_task_init(void)
{
    // 初始化调试串口
    debug_uart = Uart_register(&huart1, NULL); // 使用UART1作为调试串口
    
    //osThreadDef中的形参分别为任务名，任务函数入口，任务优先级，保留参数，栈大小
    // osThreadDef(ins_task, Ins_task, osPriorityAboveNormal, 0, 1024);
    // ins_task_handle = osThreadCreate(osThread(ins_task), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    //
    // osThreadDef(chassis_task, Chassis_task, osPriorityNormal, 0, 256);
    // chassis_task_handle = osThreadCreate(osThread(chassis_task), NULL);
    //
    // osThreadDef(gimbal_task, Gimbal_task, osPriorityNormal, 0, 128);
    // gimbal_task_handle = osThreadCreate(osThread(gimbal_task), NULL);
    //
    // osThreadDef(shoot_task, Shoot_task, osPriorityNormal, 0, 1024);
    // shoot_task_handle = osThreadCreate(osThread(shoot_task), NULL);
    //
    // osThreadDef(referee_task, Referee_task, osPriorityNormal, 0, 1024);
    // referee_task_handle = osThreadCreate(osThread(referee_task), NULL);
    //
    // osThreadDef(rc_task, Rc_task, osPriorityNormal, 0, 1024);
    // rc_task_handle = osThreadCreate(osThread(rc_task), NULL);
    //
    // osThreadDef(others_task, Others_task, osPriorityNormal, 0, 1024);
    // others_task_handle = osThreadCreate(osThread(others_task), NULL);

    // 创建测试任务
    osThreadDef(bmi088_test_task, Bmi088_test_task, osPriorityNormal, 0, 512);
    bmi088_test_task_handle = osThreadCreate(osThread(bmi088_test_task), NULL);

    // osThreadDef(can_motors_test_task, Can_motors_test_task, osPriorityNormal, 0, 512);
    // can_motors_test_task_handle = osThreadCreate(osThread(can_motors_test_task), NULL);

    // osThreadDef(rc_test_task, Rc_test_task, osPriorityNormal, 0, 512);
    // rc_test_task_handle = osThreadCreate(osThread(rc_test_task), NULL);
}