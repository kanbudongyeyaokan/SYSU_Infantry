#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "can.h"

//CAN实例最大数量
# define  CAN_MAX_COUNT  16

/**CAN结构体定义**/
#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef tx_config; // CAN报文发送配置
    uint32_t tx_mailbox;           // CAN消息填入的邮箱
    uint32_t can_id;               // 发送所需要的CAN总线ID
                                    // 发送缓冲区由外部提供
    uint32_t tx_id;                // 设备ID，由设备闪烁次数或拨码开关决定，其值一般为1-8，可有可无
    uint8_t rx_buffer[8];          // 接收缓冲区
    uint32_t rx_id;                // 接收ID,由电机类型以及can_id进行计算，也可以手动输入
    // 接收的回调函数指针,用于解析接收到的数据
    void (*receive_callback)(struct _*);//等效于参数传入为一个Can_controller_t类型的结构体指针
} Can_controller_t;
#pragma pack()

/*CAN初始化结构体定义*/
#pragma pack(1)
typedef struct can_init
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef tx_config; // CAN报文发送配置
    uint32_t can_id;               //电机ID，由电机的闪烁次数或拨码开关决定
    uint32_t tx_id;
    uint32_t rx_id;                // 接收ID,由电机类型以及can_id进行计算，也可以手动输入
    void (*receive_callback)( Can_controller_t* );
} Can_init_t;
#pragma pack()

/**CAN设备初始化**/
Can_controller_t *Can_device_init(Can_init_t *can_config);

/**CAN发送数据**/
uint8_t Can_send_data(Can_controller_t* Can_dev,uint8_t* tx_buff);



#endif //_BSP_CAN_H