#include  "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include  "can.h"
#include  "bsp_dwt.h"

/**CAN实例管理**/
static Can_controller_t *can_controller[CAN_MAX_COUNT] = {NULL};
static uint8_t can_ix; // 全局CAN实例索引,每次有新的模块注册会自增

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
void Can_init()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/*添加CAN过滤器*/
static void Can_filter_add(Can_controller_t *can_temp_controller)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (can_temp_controller->can_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;   // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow = can_temp_controller->rx_id << 5;                                            // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = can_temp_controller->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后进行过滤器管理
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器
    //配置CAN过滤器
    HAL_CAN_ConfigFilter(can_temp_controller->can_handle, &can_filter_conf);
}
/*CAN管理者注册*/
Can_controller_t* Can_device_init(Can_init_t *can_config)
{
    //初始化新的CAN实例
    Can_controller_t* can_dev = (Can_controller_t*)malloc(sizeof(Can_controller_t));
    memset(can_dev, 0, sizeof(Can_controller_t));
    if (can_dev == NULL) return NULL;
    //首次调用,检查设备数量
    if (can_ix > CAN_MAX_COUNT) {
        free(can_dev);
        return NULL;
    }
    //检查是否重复定义CAN
    for (int i=0;i<can_ix;i++)
    {
        if (can_controller[i]->can_handle == can_config->can_handle && can_controller[i]->rx_id == can_config->rx_id) {
            free(can_dev);
            return NULL;
        }
    }
    //若正常，则进行初始化
    can_dev->can_handle = can_config->can_handle;//CAN句柄
    can_dev->can_id = can_config->can_id;//CAN 总线ID
    can_dev->tx_id  = can_config->tx_id; //CAN设备ID
    can_dev->rx_id = can_config->rx_id;  //CAN 接收ID
    can_dev->receive_callback = can_config->receive_callback;//接收函数
    /*CAN发送配置*/
    can_dev->tx_config.StdId = can_config->can_id;//发送ID，比如大疆电机有0X1FF,0X200,0X2FF等等
    can_dev->tx_config.IDE = CAN_ID_STD;//使用CAN标准帧（11位）
    can_dev->tx_config.RTR = CAN_RTR_DATA;//发送数据帧
    can_dev->tx_config.DLC = 0x08;//配置CAN发送长度为8，默认长度
    /*配置对应的过滤器组*/
    Can_filter_add(can_dev);
    can_controller[can_ix++] = can_dev;

    return can_dev;
}
/*CAN发送数据*/
uint8_t Can_send_data(Can_controller_t* Can_controller,uint8_t *tx_buff)
{
    //安全检查
    if (tx_buff == NULL) {
        return 0;
    }
    //获取开始时间
    float time_start = DWT_GetTimeline_ms();
    //检查邮箱是否空闲
   // if (HAL_CAN_GetTxMailboxesFreeLevel(Can_controller->can_handle) == 0)

        //超时保护，1ms如果还不发送就退出
        /*
        if (DWT_GetTimeline_ms() - time_start > 1)
            return 0;
        */
   // HAL_CAN_AddTxMessage(Can_controller->can_handle,&Can_controller->tx_config,tx_buff,&Can_controller->tx_mailbox);
        if (HAL_CAN_AddTxMessage(Can_controller->can_handle,&Can_controller->tx_config,
            tx_buff,&Can_controller->tx_mailbox) != HAL_OK ) {
            return 0;
        }
       // {
       //     return 0;
       // }


}

//接收处理函数，会被两个FIFO接收回调进行调用
static void Can_fifo_callback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8]; //新定义一个数组用于存储CAN接收到的信息
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < can_ix; ++i)
        { // 两者相等说明这是要找的实例
            if (hcan == can_controller[i]->can_handle && rxconf.StdId == can_controller[i]->rx_id)
            {
                if (can_controller[i]->receive_callback != NULL) // 回调函数不为空就调用
                {
                    memcpy(can_controller[i]->rx_buffer, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_controller[i]->receive_callback(can_controller[i],can_controller[i]->context);     // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

/**
 * @brief CAN FIFO0接收回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_fifo_callback(hcan, CAN_RX_FIFO0); // 调用用户自定义函数来处理消息
}

/**
 * @brief CAN FIFO1接收回调函数
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_fifo_callback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
