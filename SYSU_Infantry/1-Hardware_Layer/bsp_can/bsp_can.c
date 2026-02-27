#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "can.h"
#include "stdio.h"

// [新增] 快速查找表 (Look-Up Table)
// 索引是 CAN ID，存储的是对应的设备指针
static Can_controller_t *can1_rx_lut[CAN_FAST_LUT_SIZE] = {NULL};
static Can_controller_t *can2_rx_lut[CAN_FAST_LUT_SIZE] = {NULL};

// 原有的线性列表保留，用于管理内存防止泄露，或者处理超出 LUT 范围的 ID
static Can_controller_t *can_controller[CAN_MAX_COUNT] = {NULL};
static uint8_t can_ix = 0; // 全局CAN实例索引

/**
 * @brief 配置全局 CAN 过滤器
 */
static void Can_filter_config_global(void)
{
    CAN_FilterTypeDef can_filter_conf;

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterIdHigh = 0x0000;
    can_filter_conf.FilterIdLow = 0x0000;
    can_filter_conf.FilterMaskIdHigh = 0x0000;
    can_filter_conf.FilterMaskIdLow = 0x0000;
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_conf.FilterActivation = ENABLE;

    // 配置 CAN1 过滤器 (Bank 0)
    can_filter_conf.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_conf);

    // 配置 CAN2 过滤器 (Bank 14)
    can_filter_conf.FilterBank = 14;
    can_filter_conf.SlaveStartFilterBank = 14;
    //修改，将CAN2换到另外一个FIFO，避免和CAN1冲突
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_conf);
}

/**
 * @brief 启动CAN服务
 */
void Can_init()
{
    Can_filter_config_global();

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* CAN管理者注册 */
Can_controller_t* Can_device_init(Can_init_t *can_config)
{
    if (can_ix >= CAN_MAX_COUNT) return NULL;

    // 1. 分配内存
    Can_controller_t* can_dev = (Can_controller_t*)malloc(sizeof(Can_controller_t));
    if (can_dev == NULL) return NULL;
    memset(can_dev, 0, sizeof(Can_controller_t));

    // 2. 赋值
    can_dev->can_handle = can_config->can_handle;
    can_dev->can_id = can_config->can_id;
    can_dev->tx_id  = can_config->tx_id;
    can_dev->rx_id = can_config->rx_id;
    can_dev->receive_callback = can_config->receive_callback;
    can_dev->context = can_config->context;

    can_dev->tx_config.StdId = can_config->can_id;
    can_dev->tx_config.IDE = CAN_ID_STD;
    can_dev->tx_config.RTR = CAN_RTR_DATA;
    can_dev->tx_config.DLC = 0x08;

    // 3. [关键优化] 注册到快速查找表
    // 如果 ID 在 0~0x2FF 范围内，直接填入指针数组
    if (can_config->rx_id < CAN_FAST_LUT_SIZE)
    {
        if (can_config->can_handle == &hcan1) {
            can1_rx_lut[can_config->rx_id] = can_dev;
        } else if (can_config->can_handle == &hcan2) {
            can2_rx_lut[can_config->rx_id] = can_dev;
        }
    }

    // 4. 同时也放入线性列表 (作为备份管理)
    can_controller[can_ix++] = can_dev;

    return can_dev;
}

/* CAN发送数据 */
uint8_t Can_send_data(Can_controller_t* Can_controller, uint8_t *tx_buff)
{
    if (tx_buff == NULL || Can_controller == NULL) return 0;

    static uint32_t tx_mailbox;

    // 检查邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(Can_controller->can_handle) == 0)
    {
        return 0;
    }

    if (HAL_CAN_AddTxMessage(Can_controller->can_handle, &Can_controller->tx_config,
                             tx_buff, &tx_mailbox) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

// 内部函数：统一处理 FIFO 数据
// 使用 inline 建议编译器优化
static inline void Can_fifo_process(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    static uint8_t can_rx_buff[8];
    Can_controller_t *target_dev = NULL;

    // 循环取出 FIFO 中的所有数据
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff) == HAL_OK)
        {
            // === [优化] 极速查表 ===
            // 不再循环遍历数组，而是直接用 ID 当索引去取指针
            if (rxconf.StdId < CAN_FAST_LUT_SIZE)
            {
                if (hcan == &hcan1) {
                    target_dev = can1_rx_lut[rxconf.StdId];
                } else {
                    target_dev = can2_rx_lut[rxconf.StdId];
                }
            }

            // 如果查到了设备，且注册了回调
            if (target_dev != NULL && target_dev->receive_callback != NULL)
            {
                // 1. 拷贝数据到设备自己的buffer (兼容旧逻辑)
                // 使用 uint32_t 拷贝比 memcpy 快
                uint32_t *dst = (uint32_t *)target_dev->rx_buffer;
                uint32_t *src = (uint32_t *)can_rx_buff;
                dst[0] = src[0];
                dst[1] = src[1];

                // 2. 执行回调
                target_dev->receive_callback(target_dev, target_dev->context);
            }
            // 重置指针，防止污染下一次循环
            target_dev = NULL;
        }
    }
}

/**
 * @brief CAN FIFO0接收回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_fifo_process(hcan, CAN_RX_FIFO0);
}

/**
 * @brief CAN FIFO1接收回调函数
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_fifo_process(hcan, CAN_RX_FIFO1);
}