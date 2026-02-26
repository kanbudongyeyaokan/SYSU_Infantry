#ifndef _VIDEO_LINK_H
#define _VIDEO_LINK_H

#include "usart.h"
#include "bsp_usart.h"

// 图传链路波特率: 921600
#define VIDEO_LINK_BAUDRATE 921600

// 命令码定义
#define CMD_ROBOT_TO_CONTROLLER  0x0309  // 机器人→自定义控制器
#define CMD_SET_VIDEO_CHANNEL    0x0F01  // 设置图传信道

// 图传数据帧结构
typedef __packed struct {
    uint8_t  sof;           // 起始字节 0xA5
    uint16_t data_length;   // 数据段长度
    uint8_t  seq;           // 包序号
    uint8_t  crc8;          // 帧头CRC8
    uint16_t cmd_id;        // 命令码
    uint8_t  data[30];      // 数据段(最大30字节)
    uint16_t crc16;         // 整包CRC16
} video_link_frame_t;

//图传接收端遥控原始帧结构
typedef __packed struct
{
    uint16_t sof;           //起始字节 0xA9 0x53
    uint8_t data[17];       //数据段（17字节）
    uint16_t crc16;         //整包CRC16 (CCITT-FALSE)
}video_rc_data_t;           // 共21字节

#ifndef CURRENT
#define CURRENT 0
#define LAST    1
#endif

// VT03/VT13 解析后的遥控数据
typedef struct {
    struct {
        int16_t Lrocker_x;   // 左摇杆水平 Ch3
        int16_t Lrocker_y;   // 左摇杆竖直 Ch2
        int16_t Rrocker_x;   // 右摇杆水平 Ch0
        int16_t Rrocker_y;   // 右摇杆竖直 Ch1
        int16_t dial;        // 拨轮
        uint8_t mode_switch; // 模式拨杆 C=0,N=1,S=2
        uint8_t pause;       // 暂停键
        uint8_t btn_left;    // 自定义左键
        uint8_t btn_right;   // 自定义右键
        uint8_t trigger;     // 扳机键
    } rc;
    struct {
        int16_t x, y, z;
        uint8_t press_l, press_r, press_m;
    } mouse;
    uint16_t keyboard;       // 键盘位图，同RC_ctrl_t
} Video_RC_ctrl_t;

// // 初始化图传链路，传入对应串口句柄(需预先配置为921600波特率)
// void Video_Link_Init(UART_HandleTypeDef *huart);

// 发送自定义数据到选手端(最大30字节)
void Video_Link_SendCustomData(uint8_t *data, uint16_t length);

// // 设置图传信道(1-6)
// void Video_Link_SetChannel(uint8_t channel);

// 初始化图传遥控接收，返回解析数据指针(索引0=当前,1=上一帧)
Video_RC_ctrl_t *Video_RC_Data_Get(UART_HandleTypeDef *huart);

#endif // _BSP_VIDEO_LINK_H
