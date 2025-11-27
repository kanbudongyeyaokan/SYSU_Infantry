/**
 * @file    rc_test_task.c
 * @brief   GM6020电机 电流环(转矩环) PID 测试任务 - 强力前馈版
 * @author  SYSU电控组
 * @note    针对实际电流幅值仅为目标一半的问题，大幅倍增前馈系数和积分增益
 */

#include "rc_test_task.h"
#include "bsp_dwt.h"
#include "main.h"
#include "algorithm_pid.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* === 配置区域 === */
#define CAN_HANDLE          &hcan1
#define TEST_MOTOR_ID       1

/* GM6020 协议常量 */
#define GM6020_TX_ID_GROUP  0x1FF
#define GM6020_RX_ID_BASE   0x204

extern CAN_HandleTypeDef hcan1;

static int16_t output_voltage = 0;
static float current_torque_current = 0.0f; // 当前实际电流
static float target_torque_current = 0.0f;  // 目标电流

static Pid_instance_t current_pid; // 电流环 PID

/* === 辅助函数 === */

static void CAN_Force_Reset_And_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    HAL_CAN_Stop(CAN_HANDLE);
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(CAN_HANDLE, &sFilterConfig);
    HAL_CAN_Start(CAN_HANDLE);
    HAL_CAN_ActivateNotification(CAN_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void GM6020_Send_Voltage(int16_t voltage)
{
    static uint8_t tx_data[8];
    static uint32_t tx_mailbox;
    static CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = GM6020_TX_ID_GROUP;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    memset(tx_data, 0, 8);
    tx_data[(TEST_MOTOR_ID - 1) * 2]     = (uint8_t)(voltage >> 8);
    tx_data[(TEST_MOTOR_ID - 1) * 2 + 1] = (uint8_t)(voltage);

    if (HAL_CAN_GetTxMailboxesFreeLevel(CAN_HANDLE) == 0) {
        HAL_CAN_AbortTxRequest(CAN_HANDLE, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        return;
    }
    HAL_CAN_AddTxMessage(CAN_HANDLE, &tx_header, tx_data, &tx_mailbox);
}
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == (GM6020_RX_ID_BASE + TEST_MOTOR_ID))
        {
            int16_t raw_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            current_torque_current = (float)raw_current;
        }
    }
}
*/
void Rc_test_task(void const *argument)
{
    osDelay(200);
    CAN_Force_Reset_And_Config();

    printf("\r\n=== GM6020 CURRENT LOOP TEST (Power Boost) ===\r\n");

    Pid_init_t current_conf = {0};

    // [1. 参数大幅增强]
    current_conf.kp = 2.0f;      // 1.2 -> 1.5 (微调)
    current_conf.ki = 1200.0f;   // 800 -> 2000 (大幅增强积分，消除幅度差)
    current_conf.kd = 0.0f;

    current_conf.optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_FEEDFOWARD | PID_OUTPUT_FILTER;

    // [2. 前馈系数翻倍]
    // 实测幅度比 0.47，需要放大 2.1 倍
    // 12.0 * 2.1 ≈ 25.2 -> 取 25.0
    current_conf.feedfoward_coefficient = 12.0f;

    // 滤波系数保持
    current_conf.LPF_coefficient = 0.002f;

    current_conf.max_out = 28000.0f;
    current_conf.max_iout = 10000.0f;

    Pid_init(&current_pid, &current_conf);

    uint32_t tick = 0;
    float time_s = 0.0f;
    uint32_t previous_wake_time = osKernelSysTick();

    for(;;)
    {
        time_s += 0.001f;
        target_torque_current = 1500.0f * sinf(2.0f * 3.14159f * 0.5f * time_s);

        float pid_out = Pid_calculate(&current_pid, current_torque_current, target_torque_current);
        output_voltage = (int16_t)pid_out;

        GM6020_Send_Voltage(output_voltage);

        if (tick % 20 == 0)
        {
            // 打印
            printf("C:%.2f,%.2f,%.2f\n", target_torque_current, current_torque_current, output_voltage / 10.0f);
        }

        tick++;
        osDelayUntil(&previous_wake_time, 5);
    }
}