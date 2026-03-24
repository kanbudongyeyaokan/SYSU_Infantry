#include "Gimbal_task.h"
#include "gimbal.h"
#include "robot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "vision_comm.h"


// 云台控制频率 500Hz (5ms)
#define GIMBAL_TASK_PERIOD 1

void Gimbal_control_task(void const *argument) {
    // 初始化
    Gimbal_task_init();

    // 本地指令缓存
    Gimbal_cmd_send_t cmd_recv;
    // 默认初始化为无力模式
    cmd_recv.gimbal_mode = GIMBAL_ZERO_FORCE;
    cmd_recv.yaw = 0;
    cmd_recv.pitch = 0;

    // 绝对延时变量
    TickType_t PreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        Vision_Comm_Parse_Task();
        
        xQueueReceive(Gimbal_cmd_queue_handle, &cmd_recv, 0);

        // Uart_printf(test_uart,"yaw:%f, pitch:%f\r\n",cmd_recv.yaw,cmd_recv.pitch);

        // 调用逻辑层
        // 这一步包含了 状态机逻辑 + PID计算 (算发分离)
        Gimbal_handle_command(&cmd_recv);

        // 绝对延时，保证严格的计算频率
        vTaskDelayUntil(&PreviousWakeTime, GIMBAL_TASK_PERIOD);
    }
}