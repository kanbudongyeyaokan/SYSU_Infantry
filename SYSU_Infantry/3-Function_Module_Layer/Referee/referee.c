#include "referee.h"
#include <string.h>
#include "crc_referee.h"

// 全局变量
static Referee_Data_t referee_data;
static Uart_instance_t *referee_uart = NULL;

static void Referee_Unpack(uint8_t *data, uint16_t len)
{
    int i = 0;
    while (i < len)
    {
        if (data[i] == REF_SOF)
        {
            if ((len - i) < REF_HEADER_LEN) break;

            if (Verify_CRC8_Check_Sum(&data[i], REF_HEADER_LEN))
            {
                frame_header_t *pHeader = (frame_header_t *)&data[i];
                uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + pHeader->data_length + REF_CRC16_LEN;

                if ((len - i) < total_len) break;

                if (Verify_CRC16_Check_Sum(&data[i], total_len))
                {
                    uint16_t cmd_id = (data[i + 6] << 8) | data[i + 5];
                    uint8_t *pData = &data[i + 7];

                    switch (cmd_id)
                    {
                        case GAME_STATUS_CMD_ID:
                            memcpy(&referee_data.game_status, pData, sizeof(ext_game_status_t));
                            break;
                        case GAME_RESULT_CMD_ID:
                            memcpy(&referee_data.game_result, pData, sizeof(ext_game_result_t));
                            break;
                        case ROBOT_HP_CMD_ID:
                            memcpy(&referee_data.game_robot_HP, pData, sizeof(ext_game_robot_HP_t));
                            break;
                        case EVENT_DATA_CMD_ID:
                            memcpy(&referee_data.event_data, pData, sizeof(ext_event_data_t));
                            break;
                        case ROBOT_STATUS_CMD_ID:
                            memcpy(&referee_data.robot_status, pData, sizeof(ext_game_robot_status_t));
                            referee_data.is_online = 1;
                            break;
                        case POWER_HEAT_DATA_CMD_ID: // 0x0202
                            memcpy(&referee_data.power_heat_data, pData, sizeof(ext_power_heat_data_t));
                            break;
                        case ROBOT_POS_CMD_ID:
                            memcpy(&referee_data.robot_pos, pData, sizeof(ext_robot_pos_t));
                            break;
                        case BUFF_MUSCLE_CMD_ID:
                            memcpy(&referee_data.buff, pData, sizeof(ext_buff_t));
                            break;
                        case ROBOT_HURT_CMD_ID:
                            memcpy(&referee_data.robot_hurt, pData, sizeof(ext_robot_hurt_t));
                            break;
                        case SHOOT_DATA_CMD_ID:
                            memcpy(&referee_data.shoot_data, pData, sizeof(ext_shoot_data_t));
                            break;
                        case PROJECTILE_ALLOWANCE_CMD_ID:
                            memcpy(&referee_data.projectile_allowance, pData, sizeof(ext_projectile_allowance_t));
                            break;
                        case RFID_STATUS_CMD_ID:
                            memcpy(&referee_data.rfid_status, pData, sizeof(ext_rfid_status_t));
                            break;
                        default:
                            break;
                    }
                    i += total_len;
                    continue;
                }
            }
        }
        i++;
    }
}

static void Referee_Rx_Callback(void)
{
    if (referee_uart == NULL) return;
    Referee_Unpack(referee_uart->rx_buffer, referee_uart->rx_data_len);
}

// 拆分后的获取指针函数
Referee_Data_t* Referee_Get_Data(UART_HandleTypeDef *huart)
{
    referee_uart = Uart_register(huart, Referee_Rx_Callback);
    return &referee_data;
}

// UI 发送测试
void Referee_Send_UI_Test(void)
{
    if (referee_uart == NULL) return;
    uint8_t tx_buf[128];
    frame_header_t *pHeader = (frame_header_t *)tx_buf;
    uint16_t *pCmdID = (uint16_t *)&tx_buf[5];
    ext_student_interactive_header_data_t *pInterHeader = (ext_student_interactive_header_data_t *)&tx_buf[7];
    graphic_data_struct_t *pGraphic = (graphic_data_struct_t *)&tx_buf[7 + sizeof(ext_student_interactive_header_data_t)];

    pInterHeader->data_cmd_id = 0x0101;
    uint8_t robot_id = referee_data.robot_status.robot_id;
    pInterHeader->sender_id = robot_id;
    pInterHeader->receiver_id = (robot_id == 0) ? (1 | 0x0100) : (robot_id | 0x0100);

    pGraphic->graphic_name[0] = 'T';
    pGraphic->graphic_name[1] = 'S';
    pGraphic->graphic_name[2] = 'T';
    pGraphic->operate_tpye = 1;
    pGraphic->graphic_tpye = 0;
    pGraphic->layer = 0;
    pGraphic->color = 1;
    pGraphic->start_x = 500;
    pGraphic->start_y = 500;
    pGraphic->end_x = 900;
    pGraphic->end_y = 900;
    pGraphic->width = 5;

    uint16_t data_len = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t);
    pHeader->SOF = REF_SOF;
    pHeader->data_length = data_len;
    pHeader->seq = 0;

    Append_CRC8_Check_Sum(tx_buf, REF_HEADER_LEN - 1);
    *pCmdID = INTERACTIVE_DATA_CMD_ID;
    uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + data_len + REF_CRC16_LEN;
    Append_CRC16_Check_Sum(tx_buf, total_len - 2);
    Uart_sendData(referee_uart, tx_buf, total_len);
}

uint16_t ChassisPower_GetMaxLimit(void)
{
    return referee_data.robot_status.chassis_power_limit;
}

uint16_t ChassisPower_GetBuffer(void)
{
    return referee_data.power_heat_data.buffer_energy;
}