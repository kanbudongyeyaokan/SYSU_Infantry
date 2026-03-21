#include "referee_ui.h"
#include <string.h>

// 引入外部必需的变量和函数（确保你在 referee.c 中有对应定义）
extern Uart_instance_t *referee_uart;
extern uint8_t Get_Robot_ID(void); 

// 获取发送目标 Client ID
static uint16_t UI_Get_Client_ID(void)
{
    uint8_t robot_id = Get_Robot_ID(); 
    // 离线测试时强制映射为 3号红方步兵的屏幕，在线时自动匹配
    return (robot_id == 0) ? (3 | 0x0100) : (robot_id | 0x0100);
}

// =========================================================
// 核心发送引擎：绝对物理偏移打包，无视编译器对齐Bug
// =========================================================
static void UI_Send_Raw(uint16_t sub_cmd_id, uint8_t *payload, uint16_t payload_size)
{
    if (referee_uart == NULL) return;

    uint8_t tx_buf[128]; 
    memset(tx_buf, 0, 128);

    // 1. 帧头 (5字节)
    frame_header_t *pHeader = (frame_header_t *)tx_buf;
    pHeader->SOF = REF_SOF;
    pHeader->data_length = 6 + payload_size; // 交互数据头(6字节) + 图形数据长度
    
    static uint8_t seq_num = 0;
    pHeader->seq = seq_num++; 
    Append_CRC8_Check_Sum(tx_buf, REF_HEADER_LEN - 1);

    // 2. CmdID (2字节)
    uint16_t *pCmdID = (uint16_t *)&tx_buf[5];
    *pCmdID = INTERACTIVE_DATA_CMD_ID; // 0x0301

    // 3. 交互数据头 (6字节：通过绝对偏移单字节赋值，100%安全)
    uint8_t robot_id = Get_Robot_ID();
    uint16_t receiver_id = UI_Get_Client_ID();

    tx_buf[7]  = sub_cmd_id & 0xFF;         // data_cmd_id 低八位
    tx_buf[8]  = (sub_cmd_id >> 8) & 0xFF;  // data_cmd_id 高八位
    tx_buf[9]  = robot_id;                  // sender_id 低八位
    tx_buf[10] = 0x00;                      // sender_id 高八位
    tx_buf[11] = receiver_id & 0xFF;        // receiver_id 低八位
    tx_buf[12] = (receiver_id >> 8) & 0xFF; // receiver_id 高八位

    // 4. 数据段拷贝 (从第13个字节开始)
    memcpy(&tx_buf[13], payload, payload_size);

    // 5. 结尾 CRC16
    uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + 6 + payload_size + REF_CRC16_LEN;
    Append_CRC16_Check_Sum(tx_buf, total_len - 2);

    // 串口发送
    Uart_sendData(referee_uart, tx_buf, total_len);
}

// =========================================================
// 图形参数装填 API
// =========================================================

void UI_Pack_Line(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    memset(pic, 0, sizeof(graphic_data_struct_t));
    memcpy(pic->graphic_name, name, 3);
    pic->operate_tpye = op;
    pic->graphic_tpye = UI_GRAPHIC_LINE; 
    pic->layer = layer;
    pic->color = color;
    pic->width = width;
    pic->start_x = start_x;
    pic->start_y = start_y;
    pic->end_x = end_x; 
    pic->end_y = end_y; 
}

void UI_Pack_Rectangle(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    memset(pic, 0, sizeof(graphic_data_struct_t));
    memcpy(pic->graphic_name, name, 3);
    pic->operate_tpye = op;
    pic->graphic_tpye = UI_GRAPHIC_RECTANGLE; 
    pic->layer = layer;
    pic->color = color;
    pic->width = width;
    pic->start_x = start_x;
    pic->start_y = start_y;
    pic->end_x = end_x; 
    pic->end_y = end_y; 
}

void UI_Pack_Circle(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t center_x, uint32_t center_y, uint32_t radius_val)
{
    memset(pic, 0, sizeof(graphic_data_struct_t));
    memcpy(pic->graphic_name, name, 3);
    pic->operate_tpye = op;
    pic->graphic_tpye = UI_GRAPHIC_CIRCLE;
    pic->layer = layer;
    pic->color = color;
    pic->width = width;
    pic->start_x = center_x;
    pic->start_y = center_y;
    pic->radius = radius_val; 
}

void UI_Pack_Float(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t start_x, uint32_t start_y, float value)
{
    memset(pic, 0, sizeof(graphic_data_struct_t));
    memcpy(pic->graphic_name, name, 3);
    pic->operate_tpye = op;
    pic->graphic_tpye = UI_GRAPHIC_FLOAT;
    pic->layer = layer;
    pic->color = color;
    pic->start_x = start_x;
    pic->start_y = start_y;
    pic->start_angle = font_size; // 手册：浮点数字体大小由 details_a(此处宏定义为start_angle) 决定
    
    // 浮点数拆包转换
    int32_t val_int = (int32_t)(value * 1000.0f);
    pic->radius = (val_int >> 0)  & 0x3FF;
    pic->end_x  = (val_int >> 10) & 0x7FF;
    pic->end_y  = (val_int >> 21) & 0x7FF;
}

void UI_Pack_Int(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t start_x, uint32_t start_y, int32_t value)
{
    memset(pic, 0, sizeof(graphic_data_struct_t));
    memcpy(pic->graphic_name, name, 3);
    pic->operate_tpye = op;
    pic->graphic_tpye = UI_GRAPHIC_INT;
    pic->layer = layer;
    pic->color = color;
    pic->start_x = start_x;
    pic->start_y = start_y;
    pic->start_angle = font_size; 
    
    // 整型拆包转换
    pic->radius = (value >> 0)  & 0x3FF;
    pic->end_x  = (value >> 10) & 0x7FF;
    pic->end_y  = (value >> 21) & 0x7FF;
}

void UI_Pack_String(ui_string_t *str_struct, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t str_len, uint32_t start_x, uint32_t start_y, const char* string)
{
    memset(str_struct, 0, sizeof(ui_string_t));
    memcpy(str_struct->figure.graphic_name, name, 3);
    str_struct->figure.operate_tpye = op;
    str_struct->figure.graphic_tpye = UI_GRAPHIC_CHAR;
    str_struct->figure.layer = layer;
    str_struct->figure.color = color;
    str_struct->figure.start_x = start_x;
    str_struct->figure.start_y = start_y;
    str_struct->figure.start_angle = font_size;
    str_struct->figure.end_angle = str_len; 

    // 拷贝至多30个字符
    strncpy((char *)str_struct->string, string, 30);
}

// =========================================================
// 发送指令 API
// =========================================================

void UI_Delete_Layer(uint8_t layer)
{
    uint8_t del_data[2];
    del_data[0] = 1; // 1: 删除单图层
    del_data[1] = layer;
    UI_Send_Raw(UI_DATA_DEL_ID, del_data, 2); 
}

void UI_Delete_All(void)
{
    uint8_t del_data[2];
    del_data[0] = 2; // 2: 删除所有
    del_data[1] = 0;
    UI_Send_Raw(UI_DATA_DEL_ID, del_data, 2); 
}

void UI_Send_Single_Figure(graphic_data_struct_t *pic)
{
    UI_Send_Raw(UI_DATA_DRAW_1_ID, (uint8_t *)pic, 15);
}

void UI_Send_Multi_Figures(uint8_t count, graphic_data_struct_t *pics)
{
    uint16_t sub_cmd_id;
    if (count == 2)      sub_cmd_id = UI_DATA_DRAW_2_ID;  // 0x0102
    else if (count <= 5) sub_cmd_id = UI_DATA_DRAW_5_ID;  // 0x0103
    else if (count <= 7) sub_cmd_id = UI_DATA_DRAW_7_ID;  // 0x0104
    else return;

    UI_Send_Raw(sub_cmd_id, (uint8_t *)pics, count * 15);
}

void UI_Send_String(ui_string_t *str_struct)
{
    UI_Send_Raw(UI_DATA_DRAW_CHAR_ID, (uint8_t *)str_struct, 45); 
}