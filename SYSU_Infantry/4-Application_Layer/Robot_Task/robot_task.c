#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "robot_task.h"
#include "bsp_usart.h"

#include "usart.h"

#include "queue.h"

// 测试任务头文件
#include <stdio.h>


#include "bridge_link_task.h"
#include "bsp_usb.h"
#include "buzzer_alarm_task.h"
#include "buzzer_alarm.h"
#include "error_handler.h"


osThreadId buzzer_alarm_task_handle;


QueueHandle_t Buzzer_cmd_queue_handle;



Uart_instance_t* test_uart = NULL;
/**机器人任务创建**/
void Robot_task_init(void)
{
    // test_uart = Uart_register(&huart1,NULL);
    Buzzer_cmd_queue_handle = xQueueCreate(5, sizeof(uint8_t));
  osThreadDef(buzzer_alarm_task, Buzzer_alarm_control_task, osPriorityNormal, 0, 1024);
  buzzer_alarm_task_handle = osThreadCreate(osThread(buzzer_alarm_task), NULL);

  error_system_init();

  ERROR_INFO("SYS", "Init");

    osThreadDef(bridge, Bridge_Link_Task, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(bridge), NULL);   
}


