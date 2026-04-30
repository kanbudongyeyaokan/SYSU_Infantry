/**
 * @file    bridge_link_task.c
 * @brief   Bridge Link FreeRTOS 任务示例
 *
 * 使用方法：
 *   1. 在 FreeRTOS 任务创建处（如 freertos.c 或 robot_task.c）增加本任务：
 *
 *      #include "bridge_link_task.h"
 *      osThreadDef(bridge, Bridge_Link_Task, osPriorityNormal, 0, 512);
 *      osThreadCreate(osThread(bridge), NULL);
 *
 *   2. 如需处理来自上位机的 CAN 帧，在应用层重写弱回调：
 *
 *      void Bridge_On_CAN_Received(const Bridge_CANPayload_t *frame) {
 *          // 处理 frame->id, frame->dlc, frame->data[]
 *      }
 *
 * 任务行为：
 *   - 初始化 Bridge Link（注册 USB 接收回调）
 *   - 每 10ms 解析一次接收 FIFO
 */

#include "bridge_link_task.h"
#include "bridge_link.h"
#include "cmsis_os.h"

/* 无数据时的最长等待（ms），兼作看门超时 */
#define BRIDGE_WAIT_TIMEOUT_MS  1

void Bridge_Link_Task(void const *argument)
{
    (void)argument;

    /* 等待系统稳定（USB 枚举需要一定时间） */
    osDelay(500);

    Bridge_Link_Init();

    for (;;) {
        /* 1. 解析上位机下行帧 → Bridge_On_CAN_Received → hcan1 发送 */
        Bridge_Link_Parse();

        /* 2. 排空上行环形缓冲 → 将 CAN1 收到的帧发给上位机 */
        Bridge_Uplink_Drain();

        Bridge_Link_WaitRx(BRIDGE_WAIT_TIMEOUT_MS);
    }
}
