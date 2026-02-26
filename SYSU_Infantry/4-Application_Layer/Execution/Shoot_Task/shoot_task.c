/**
 * @file    shoot_task.c
 * @brief   发射机构控制任务
 * @author  SYSU Infantry Team
 * @date    2026-02-07
 * @version 1.0
 * @note    负责接收发射指令，控制摩擦轮和拨弹盘
 */

#include "shoot_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "robot_task.h"
#include "shoot.h"
#include "task.h"

// 发射任务运行周期 (ms) -> 200Hz
#define SHOOT_TASK_PERIOD 5

/**
 * @brief   发射机构控制任务主体
 * @param   argument FreeRTOS任务参数
 * @note    1000Hz频率运行。
 *          流程：
 *          1. 接收指令队列 (Shoot_cmd_queue_handle)
 *          2. 调用 Shoot_handle_command 更新状态机与PID
 *          3. 使用 vTaskDelayUntil 保持绝对周期
 */
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