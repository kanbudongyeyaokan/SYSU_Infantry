#include "Shoot_task.h"
#include "shoot.h"
#include "robot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// 发射任务频率 1000Hz (1ms)
#define SHOOT_TASK_PERIOD 1

void Shoot_control_task(void const *argument) {
    // 初始化
    Shoot_task_init();

    // 本地指令缓存
    Shoot_cmd_send_t cmd_recv;
    // 默认初始化：全停
    cmd_recv.shoot_mode = SHOOT_OFF;
    cmd_recv.loader_mode = LOAD_STOP;
    cmd_recv.shoot_rate = 0;

    // 绝对延时变量
    TickType_t PreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        // ============================================================
        // 非阻塞查询队列
        // ============================================================
        xQueueReceive(Shoot_cmd_queue_handle, &cmd_recv, 0);

        // ============================================================
        // 调用逻辑层 (传入指令 -> 状态机 -> 算发分离)
        // ============================================================
        Shoot_handle_command(&cmd_recv);
        // ============================================================
        // 绝对延时 1ms
        // ============================================================
        vTaskDelayUntil(&PreviousWakeTime, SHOOT_TASK_PERIOD);
    }
}