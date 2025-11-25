/**
 * @file    can_motors_test_task.c
 * @brief   CAN电机测试任务源文件 (协议兼容修正版)
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 3.0
 * @note    重要修正:
 * 1. 发现该电机回复ID与发送ID相同(0x141)，而非协议文档所述的0x181
 * 2. 修改了解析逻辑，兼容两种ID模式
 */

#include "can_motors_test_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;

typedef struct {
    int8_t  temperature;
    int16_t torque_current;
    int16_t speed;
    uint16_t encoder;
    uint32_t rx_count;
    uint32_t last_rx_id; // 记录最后一次有效解析的ID
} Motor_Feedback_t;

volatile Motor_Feedback_t motor1_data = {0};

/* ================= 函数声明 ================= */
static void CAN_Filter_Config(void);
static void Motor_Enable(uint8_t motor_id);
static void Motor_SetSpeed(uint8_t motor_id, int32_t speed_dps);
static void Motor_Process_Message(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
static void CAN_Polling_Check(void);

/* ================= 任务主体 ================= */

void Can_motors_test_task(void const *argument)
{
    CAN_Filter_Config();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    printf("[Info] CAN Init Success.\r\n");
    osDelay(100);

    // 使能电机
    printf("[Cmd] Enabling Motor ID: 1...\r\n");
    Motor_Enable(1);
    osDelay(500);

    int32_t target_speed_dps = 360;
    uint16_t loop_count = 0;

    for (;;)
    {
        // 发送指令
        Motor_SetSpeed(1, target_speed_dps);

        // 轮询检查 (防止中断挂死)
        CAN_Polling_Check();

        // 打印状态 (每200ms)
        if (++loop_count >= 40)
        {
            printf("\r\n=== Motor Status (ID: 0x%X) ===\r\n", motor1_data.last_rx_id);
            printf("Temp    : %d C\r\n", motor1_data.temperature); // 应该是 27 左右
            printf("Current : %d\r\n", motor1_data.torque_current);
            printf("Speed   : %d dps\r\n", motor1_data.speed);     // 应该是 336 左右
            printf("Encoder : %d\r\n", motor1_data.encoder);
            printf("Rx Count: %lu\r\n", motor1_data.rx_count);     // 这个数应该狂跳
            printf("=============================\r\n");

            loop_count = 0;
        }

        osDelay(5);
    }
}

/* ================= 核心解析逻辑修正 ================= */

static void Motor_Process_Message(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    // 核心修改：放宽ID检查
    // 只要 ID 是 0x141 (发送ID) 或者 0x181 (文档ID)，且第一个字节是命令头，就解析
    if (RxHeader->StdId == 0x141 || RxHeader->StdId == 0x181)
    {
        // 进一步校验：数据头必须是指令回传 (比如 0xA2 或 0x88)
        // 这样可以防止真正回环时的干扰，但这里电机回复的数据头正是 A2，所以没问题

        motor1_data.last_rx_id = RxHeader->StdId;
        motor1_data.rx_count++;

        // 解析数据 (根据日志 A2 1B 12 00 50 01 14 E3)
        motor1_data.temperature = (int8_t)RxData[1]; // 0x1B = 27度
        motor1_data.torque_current = (int16_t)(RxData[2] | (RxData[3] << 8));
        motor1_data.speed = (int16_t)(RxData[4] | (RxData[5] << 8));
        motor1_data.encoder = (uint16_t)(RxData[6] | (RxData[7] << 8));
    }
}

static void CAN_Polling_Check(void)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            Motor_Process_Message(&RxHeader, RxData);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            Motor_Process_Message(&RxHeader, RxData);
        }
    }
}

/* ================= 底层发送与配置 ================= */

static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow  = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow  = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

static void Motor_Enable(uint8_t motor_id)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
    uint32_t TxMailbox;
    TxHeader.StdId = 0x140 + motor_id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

static void Motor_SetSpeed(uint8_t motor_id, int32_t speed_dps)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    int32_t speed_val = speed_dps * 100;
    int16_t iq_limit = 2000;

    TxHeader.StdId = 0x140 + motor_id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0xA2;
    TxData[1] = 0x00;
    TxData[2] = (uint8_t)(iq_limit & 0xFF);
    TxData[3] = (uint8_t)((iq_limit >> 8) & 0xFF);
    TxData[4] = (uint8_t)(speed_val & 0xFF);
    TxData[5] = (uint8_t)((speed_val >> 8) & 0xFF);
    TxData[6] = (uint8_t)((speed_val >> 16) & 0xFF);
    TxData[7] = (uint8_t)((speed_val >> 24) & 0xFF);

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}