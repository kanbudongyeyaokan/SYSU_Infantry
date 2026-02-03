#include "referee.h"
#include <string.h>
#include "crc_referee.h"

// 全局变量
static Referee_Data_t referee_data;
static Uart_instance_t *referee_uart = NULL;

/**
 * @brief 裁判系统数据包解包函数
 * @note  配合 bsp_usart 的 DMA 接收使用
 */
static void Referee_Unpack(uint8_t *data, uint16_t len)
{
    int i = 0;
    while (i < len)
    {
        // 寻找帧头 SOF
        if (data[i] == REF_SOF)
        {
            // 确保有足够的长度读完 Header
            if ((len - i) < REF_HEADER_LEN) break;

            // 校验头部 CRC8
            if (Verify_CRC8_Check_Sum(&data[i], REF_HEADER_LEN))
            {
                frame_header_t *pHeader = (frame_header_t *)&data[i];
                uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + pHeader->data_length + REF_CRC16_LEN;

                // 确保有足够长度读完整包
                if ((len - i) < total_len) break;

                // 3. 校验整包 CRC16
                if (Verify_CRC16_Check_Sum(&data[i], total_len))
                {
                    // 校验通过，开始解析数据
                    uint16_t cmd_id = (data[i + 6] << 8) | data[i + 5];
                    uint8_t *pData = &data[i + 7];

                    switch (cmd_id)
                    {
                        case GAME_STATUS_ID:
                            memcpy(&referee_data.game_status, pData, sizeof(ext_game_status_t));
                            break;
                        case ROBOT_STATUS_ID:
                            memcpy(&referee_data.robot_status, pData, sizeof(ext_game_robot_status_t));
                            referee_data.is_online = 1; // 收到此包说明User口通信正常
                            break;
                        case ROBOT_HURT_ID:
                            memcpy(&referee_data.robot_hurt, pData, sizeof(ext_robot_hurt_t));
                            break;
                        case SHOOT_DATA_ID:
                            memcpy(&referee_data.shoot_data, pData, sizeof(ext_shoot_data_t));
                            break;
                        case RFID_STATUS_ID:
                            memcpy(&referee_data.rfid_status, pData, sizeof(ext_rfid_status_t));
                            break;
                        default:
                            break;
                    }
                    // 跳过当前包
                    i += total_len;
                    continue;
                }
            }
        }
        i++; // 未找到头或校验失败，后移一位
    }
}

/**
 * @brief 串口接收回调
 */
static void Referee_Rx_Callback(void)
{
    if (referee_uart == NULL) return;
    // 直接解析 bsp_usart 缓冲区
    Referee_Unpack(referee_uart->rx_buffer, referee_uart->rx_data_len);
}

/**
 * @brief 获取数据指针
 */
Referee_Data_t* Referee_Get_Data(UART_HandleTypeDef *huart)
{
    referee_uart = Uart_register(huart, Referee_Rx_Callback);

    return &referee_data;
}

/**
 * @brief UI 绘制测试函数 (满足图传验收要求)
 * @note  发送一条黄色直线
 */
void Referee_Send_UI_Test(void)
{
    if (referee_uart == NULL) return;

    // 缓冲区 (最大128字节)
    uint8_t tx_buf[128];

    // 指针映射
    frame_header_t *pHeader = (frame_header_t *)tx_buf;
    uint16_t *pCmdID = (uint16_t *)&tx_buf[5];
    ext_student_interactive_header_data_t *pInterHeader = (ext_student_interactive_header_data_t *)&tx_buf[7];
    graphic_data_struct_t *pGraphic = (graphic_data_struct_t *)&tx_buf[7 + sizeof(ext_student_interactive_header_data_t)];

    // 1. 准备数据内容
    // 交互头
    pInterHeader->data_cmd_id = 0x0101; // 绘制一个图形
    uint8_t robot_id = referee_data.robot_status.robot_id;
    pInterHeader->sender_id = robot_id;
    // 客户端ID计算规则: 机器人ID | 0x0100
    pInterHeader->receiver_id = (robot_id == 0) ? (1 | 0x0100) : (robot_id | 0x0100);

    // 图形内容：画一条线
    pGraphic->graphic_name[0] = 'T';
    pGraphic->graphic_name[1] = 'S';
    pGraphic->graphic_name[2] = 'T';
    pGraphic->operate_tpye = 1; // 增加
    pGraphic->graphic_tpye = 0; // 直线
    pGraphic->layer = 0;
    pGraphic->color = 1;        // 黄色
    pGraphic->start_x = 500;
    pGraphic->start_y = 500;
    pGraphic->end_x = 900;
    pGraphic->end_y = 900;
    pGraphic->width = 5;

    // 2. 计算长度
    uint16_t data_len = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t);

    // 3. 填充帧头
    pHeader->SOF = REF_SOF;
    pHeader->data_length = data_len;
    pHeader->seq = 0; // 序列号可自增

    // 4. 填充头部CRC8
    Append_CRC8_Check_Sum(tx_buf, REF_HEADER_LEN - 1); // 这里的长度参数是不包含CRC字节的长度，即4

    // 5. 填充CmdID
    *pCmdID = INTERACTIVE_ID; // 0x0301

    // 6. 填充整包CRC16
    uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + data_len + REF_CRC16_LEN;
    Append_CRC16_Check_Sum(tx_buf, total_len - 2); // 这里的长度参数是不包含CRC两字节的长度

    // 7. 发送
    Uart_sendData(referee_uart, tx_buf, total_len);
}