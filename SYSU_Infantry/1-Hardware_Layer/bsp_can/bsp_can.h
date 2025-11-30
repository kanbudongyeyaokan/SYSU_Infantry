#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "can.h"
#include "stdint.h"

// CAN实例最大数量
#define CAN_MAX_COUNT 16

// 前向声明
typedef struct CanController CanController_t;

// 定义回调函数类型
typedef void (*ReceiveCallback_t)(CanController_t*, void* context);

/** CAN结构体定义 **/
#pragma pack(1)
typedef struct CanController
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef tx_config; // CAN报文发送配置
    uint32_t can_id;               // 发送所需要的CAN总线ID

    // 发送缓冲区由外部提供
    uint32_t tx_id;                // 设备ID
    uint8_t rx_buffer[8];          // 接收缓冲区
    uint32_t rx_id;                // 接收ID

    // 接收的回调函数指针
    void* context;                 // 上下文指针
    ReceiveCallback_t receive_callback; // 回调函数指针

    // [新增] 链表指针，用于哈希桶或链表管理 (优化查找效率)
    // 即使目前使用数组管理，保留此指针方便未来扩展哈希表查找
    struct CanController *next;
} Can_controller_t;
#pragma pack()

/* CAN初始化结构体定义 */
#pragma pack(1)
typedef struct
{
    CAN_HandleTypeDef *can_handle; // can句柄
    uint32_t can_id;               // 发送ID (StdId)
    uint32_t tx_id;                // 设备ID (如1,2,3...)
    uint32_t rx_id;                // 接收ID (如0x205)
    void* context;                 // 上下文指针
    ReceiveCallback_t receive_callback; // 回调函数指针
} Can_init_t;
#pragma pack()

/** CAN设备初始化 **/
Can_controller_t *Can_device_init(Can_init_t *can_config);

/** CAN发送数据 **/
uint8_t Can_send_data(Can_controller_t* Can_dev, uint8_t* tx_buff);

/** CAN初始化 **/
void Can_init();

#endif //_BSP_CAN_H