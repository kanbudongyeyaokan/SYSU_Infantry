#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "can.h"
#include "stdio.h"

/** CAN实例管理 **/
static Can_controller_t *can_controller[CAN_MAX_COUNT] = {NULL};
static uint8_t can_ix = 0; // 全局CAN实例索引

/**
 * @brief 配置全局 CAN 过滤器 (掩码模式，接收所有标准帧)
 * @note  这样就不受硬件过滤器数量限制了
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

    // 配置 CAN2 过滤器 (Bank 14, Slave Start)
    can_filter_conf.FilterBank = 14;
    can_filter_conf.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_conf);
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 */
void Can_init()
{
    // 1. 配置全局过滤器
    Can_filter_config_global();

    // 2. 启动 CAN1
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    // 3. 启动 CAN2
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* CAN管理者注册 */
Can_controller_t* Can_device_init(Can_init_t *can_config)
{
    // 首次调用检查
    if (can_ix >= CAN_MAX_COUNT) return NULL;

    // 分配内存
    Can_controller_t* can_dev = (Can_controller_t*)malloc(sizeof(Can_controller_t));
    if (can_dev == NULL) return NULL;
    memset(can_dev, 0, sizeof(Can_controller_t));

    // 检查重复 (虽然不是必须，但为了安全)
    for (int i = 0; i < can_ix; i++)
    {
        if (can_controller[i]->can_handle == can_config->can_handle &&
            can_controller[i]->rx_id == can_config->rx_id)
        {
            free(can_dev);
            return can_controller[i]; // 返回已存在的实例
        }
    }

    // 初始化参数
    can_dev->can_handle = can_config->can_handle;
    can_dev->can_id = can_config->can_id;
    can_dev->tx_id  = can_config->tx_id;
    can_dev->rx_id = can_config->rx_id;
    can_dev->receive_callback = can_config->receive_callback;
    can_dev->context = can_config->context;

    /* CAN发送配置 */
    can_dev->tx_config.StdId = can_config->can_id;
    can_dev->tx_config.IDE = CAN_ID_STD;
    can_dev->tx_config.RTR = CAN_RTR_DATA;
    can_dev->tx_config.DLC = 0x08;

    // 加入管理列表
    can_controller[can_ix++] = can_dev;

    // 注意：这里不再调用 Can_filter_add，因为我们在 Can_init 里配置了全局过滤器

    return can_dev;
}

/* CAN发送数据 */
uint8_t Can_send_data(Can_controller_t* Can_controller, uint8_t *tx_buff)
{
    if (tx_buff == NULL || Can_controller == NULL) return 0;

    static uint32_t tx_mailbox;

    // 检查邮箱是否满
    if (HAL_CAN_GetTxMailboxesFreeLevel(Can_controller->can_handle) == 0)
    {
        // 邮箱满直接返回失败，不阻塞，保证实时性
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
static void Can_fifo_process(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    static uint8_t can_rx_buff[8];

    // 循环取出 FIFO 中的所有数据，防止积压
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff) == HAL_OK)
        {
            // 遍历所有注册设备进行分发
            for (size_t i = 0; i < can_ix; ++i)
            {
                if (can_controller[i]->can_handle == hcan &&
                    can_controller[i]->rx_id == rxconf.StdId)
                {
                    if (can_controller[i]->receive_callback != NULL)
                    {
                        memcpy(can_controller[i]->rx_buffer, can_rx_buff, 8);
                        can_controller[i]->receive_callback(can_controller[i], can_controller[i]->context);
                    }
                    break; // 找到后立即退出循环
                }
            }
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