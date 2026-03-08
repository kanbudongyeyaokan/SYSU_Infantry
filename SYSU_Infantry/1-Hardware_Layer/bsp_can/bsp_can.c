#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "can.h"
#include "stdio.h"
#include "error_handler.h"

// [新增] 快速查找表 (Look-Up Table)
// 索引是 CAN ID，存储的是对应的设备指针
static Can_controller_t *can1_rx_lut[CAN_FAST_LUT_SIZE] = {NULL};
static Can_controller_t *can2_rx_lut[CAN_FAST_LUT_SIZE] = {NULL};

// 原有的线性列表保留，用于管理内存防止泄露，或者处理超出 LUT 范围的 ID
static Can_controller_t *can_controller[CAN_MAX_COUNT] = {NULL};
static uint8_t can_ix = 0; // 全局CAN实例索引

/* 高频报错限频，避免影响控制回路 */
#define CAN_ERROR_REPORT_INTERVAL_MS  100u
static uint32_t can_last_mailbox_full_tick = 0u;
static uint32_t can_last_tx_fail_tick = 0u;
static uint32_t can_last_rx_fail_tick = 0u;

static uint8_t Can_Should_Report(uint32_t *last_tick, uint32_t interval_ms)
{
    uint32_t now = HAL_GetTick();
    if ((now - *last_tick) >= interval_ms)
    {
        *last_tick = now;
        return 1u;
    }
    return 0u;
}

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
    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_conf) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN1 filter config failed hal_err=%lu esr=0x%lx msr=0x%lx",
                       HAL_CAN_GetError(&hcan1),
                       (hcan1.Instance != NULL) ? hcan1.Instance->ESR : 0u,
                       (hcan1.Instance != NULL) ? hcan1.Instance->MSR : 0u);
    }

    // 配置 CAN2 过滤器 (Bank 14)
    can_filter_conf.FilterBank = 14;
    can_filter_conf.SlaveStartFilterBank = 14;
    //修改，将CAN2换到另外一个FIFO，避免和CAN1冲突
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO1;
    if (HAL_CAN_ConfigFilter(&hcan2, &can_filter_conf) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN2 filter config failed hal_err=%lu esr=0x%lx msr=0x%lx",
                       HAL_CAN_GetError(&hcan2),
                       (hcan2.Instance != NULL) ? hcan2.Instance->ESR : 0u,
                       (hcan2.Instance != NULL) ? hcan2.Instance->MSR : 0u);
    }
}

/**
 * @brief 启动CAN服务
 */
void Can_init()
{
    Can_filter_config_global();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN1 start failed hal_err=%lu esr=0x%lx msr=0x%lx",
                       HAL_CAN_GetError(&hcan1),
                       (hcan1.Instance != NULL) ? hcan1.Instance->ESR : 0u,
                       (hcan1.Instance != NULL) ? hcan1.Instance->MSR : 0u);
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN1 FIFO0 IRQ enable failed hal_err=%lu irq_mask=%lu",
                       HAL_CAN_GetError(&hcan1),
                       CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN1 FIFO1 IRQ enable failed hal_err=%lu irq_mask=%lu",
                       HAL_CAN_GetError(&hcan1),
                       CAN_IT_RX_FIFO1_MSG_PENDING);
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN2 start failed hal_err=%lu esr=0x%lx msr=0x%lx",
                       HAL_CAN_GetError(&hcan2),
                       (hcan2.Instance != NULL) ? hcan2.Instance->ESR : 0u,
                       (hcan2.Instance != NULL) ? hcan2.Instance->MSR : 0u);
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN2 FIFO0 IRQ enable failed hal_err=%lu irq_mask=%lu",
                       HAL_CAN_GetError(&hcan2),
                       CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        ERROR_CRITICAL("CAN", "CAN2 FIFO1 IRQ enable failed hal_err=%lu irq_mask=%lu",
                       HAL_CAN_GetError(&hcan2),
                       CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

/* CAN管理者注册 */
Can_controller_t* Can_device_init(Can_init_t *can_config)
{
    if (can_config == NULL)
    {
        ERROR_RAISE("CAN", "Can_device_init config is NULL");
        return NULL;
    }

    if (can_config->can_handle == NULL)
    {
        ERROR_RAISE("CAN", "Can_device_init can_handle NULL can_id=%lu tx_id=%lu rx_id=%lu",
                    can_config->can_id, can_config->tx_id,
                    can_config->rx_id);
        return NULL;
    }

    if (can_ix >= CAN_MAX_COUNT)
    {
        ERROR_RAISE("CAN", "CAN device table full can_ix=%lu max=%lu can_id=%lu rx_id=%lu",
                    can_ix, CAN_MAX_COUNT,
                    can_config->can_id, can_config->rx_id);
        return NULL;
    }

    // 1. 分配内存
    Can_controller_t* can_dev = (Can_controller_t*)malloc(sizeof(Can_controller_t));
    if (can_dev == NULL)
    {
        ERROR_RAISE("CAN", "CAN device malloc failed size=%lu can_ix=%lu can_id=%lu rx_id=%lu",
                    sizeof(Can_controller_t), can_ix,
                    can_config->can_id, can_config->rx_id);
        return NULL;
    }
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
            if (can1_rx_lut[can_config->rx_id] != NULL)
            {
                ERROR_WARN("CAN", "CAN1 LUT overwrite rx_id=%lu old_can_id=%lu new_can_id=%lu",
                           can_config->rx_id,
                           can1_rx_lut[can_config->rx_id]->can_id,
                           can_config->can_id);
            }
            can1_rx_lut[can_config->rx_id] = can_dev;
        } else if (can_config->can_handle == &hcan2) {
            if (can2_rx_lut[can_config->rx_id] != NULL)
            {
                ERROR_WARN("CAN", "CAN2 LUT overwrite rx_id=%lu old_can_id=%lu new_can_id=%lu",
                           can_config->rx_id,
                           can2_rx_lut[can_config->rx_id]->can_id,
                           can_config->can_id);
            }
            can2_rx_lut[can_config->rx_id] = can_dev;
        } else {
            ERROR_WARN("CAN", "CAN unknown controller handle_ptr=0x%lx can_id=%lu tx_id=%lu rx_id=%lu",
                       (uint32_t)(uintptr_t)can_config->can_handle,
                       can_config->can_id, can_config->tx_id, can_config->rx_id);
        }
    }

    // 4. 同时也放入线性列表 (作为备份管理)
    can_controller[can_ix++] = can_dev;

    return can_dev;
}

/* CAN发送数据 */
uint8_t Can_send_data(Can_controller_t* Can_controller, uint8_t *tx_buff)
{
    if (tx_buff == NULL || Can_controller == NULL || Can_controller->can_handle == NULL)
    {
        ERROR_RAISE("CAN", "Can_send_data param invalid");
        return 0;
    }

    static uint32_t tx_mailbox;
    uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(Can_controller->can_handle);

    // 检查邮箱
    if (free_level == 0u)
    {
        if (Can_Should_Report(&can_last_mailbox_full_tick, CAN_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_WARN("CAN", "CAN TX mailbox full can_id=%lu tsr=0x%lx esr=0x%lx free_level=%lu",
                       Can_controller->can_id,
                       (Can_controller->can_handle->Instance != NULL) ? Can_controller->can_handle->Instance->TSR : 0u,
                       (Can_controller->can_handle->Instance != NULL) ? Can_controller->can_handle->Instance->ESR : 0u,
                       free_level);
        }
        return 0;
    }

    if (HAL_CAN_AddTxMessage(Can_controller->can_handle, &Can_controller->tx_config,
                             tx_buff, &tx_mailbox) != HAL_OK)
    {
        if (Can_Should_Report(&can_last_tx_fail_tick, CAN_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_RAISE("CAN", "HAL_CAN_AddTxMessage failed can_id=%lu hal_err=%lu tsr=0x%lx esr=0x%lx",
                        Can_controller->can_id,
                        HAL_CAN_GetError(Can_controller->can_handle),
                        (Can_controller->can_handle->Instance != NULL) ? Can_controller->can_handle->Instance->TSR : 0u,
                        (Can_controller->can_handle->Instance != NULL) ? Can_controller->can_handle->Instance->ESR : 0u);
        }
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
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox) > 0u)
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
        else
        {
            if (Can_Should_Report(&can_last_rx_fail_tick, CAN_ERROR_REPORT_INTERVAL_MS))
            {
                ERROR_RAISE("CAN", "HAL_CAN_GetRxMessage failed fifo=%lu hal_err=%lu rf0r=0x%lx rf1r=0x%lx",
                            fifox,
                            HAL_CAN_GetError(hcan),
                            (hcan->Instance != NULL) ? hcan->Instance->RF0R : 0u,
                            (hcan->Instance != NULL) ? hcan->Instance->RF1R : 0u);
            }
            break;
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
