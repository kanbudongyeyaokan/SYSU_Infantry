#ifndef _SBUS_H_
#define _SBUS_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "usart.h"

/***************SBUS遥控接收系统规格参数***************/
/********************SBUS通信协议参数******************
 * SBUS协议            工作频率           2.4GHz ISM  *
 *                     通信距离(开阔室外)   1km       *
 *----------------------------------------------------*
 * SBUS协议参数:                                      *
 * 波特率             100Kbps                         *
 * 单元数据长度         8                             *
 * 奇偶校验位         偶校验                          *
 * 结束位               2                             *
 * 流控                 无                            *
 *----------------------------------------------------*
 * 数据帧格式(25字节):                                *
 * 帧头(1字节)      0x0F                              *
 * 通道数据(22字节) 16个11位通道(打包存储)            *
 * 标志位(1字节)    包含开关S1/S2状态                 *
 * 帧尾(1字节)      0x00                              *
 *****************************************************/

/*SBUS数据处理宏*/
#define SBUS_CH_VALUE_OFFSET ((uint16_t)1024) // SBUS中心值偏移量
#define SBUS_FRAME_SIZE 25                    // SBUS帧总长度

/* ----------------------- SBUS开关值 (数字开关) ----------------------------- */
#define SBUS_SWITCH_DOWN ((uint8_t)1) // 数字开关向下时的值
#define SBUS_SWITCH_UP ((uint8_t)2)   // 数字开关向上时的值
#define SBUS_SWITCH_MID ((uint8_t)3)  // 数字开关中间时的值

/* ----------------------- SBUS拨杆阈值 (模拟通道) --------------------------- */
// 三档拨杆阈值 (Ch8等三档开关, 实际值: 下=-660, 中=0, 上=+660)
#define SBUS_3POS_THRESHOLD_DOWN  (-300)  // 小于此值判定为下档
#define SBUS_3POS_THRESHOLD_UP    (300)   // 大于此值判定为上档

// 两档拨杆阈值 (Ch5等两档开关, 实际值: 下=-660, 上=+660)
#define SBUS_2POS_THRESHOLD       (0)     // 小于0判定为下档, 大于等于0判定为上档

// 用于SBUS数据读取，数据是一个大小为2的结构体数组，分为当前数据和上一次数据
#define CURRENT 0
#define LAST 1

// SBUS原始数据结构(25字节)
#pragma pack(push, 1)
typedef struct
{
    uint8_t Start;    // 帧头 0x0F
    uint8_t Data[22]; // 通道数据(16通道×11位打包)
    uint8_t Flag;     // 标志位(含开关状态)
    uint8_t End;      // 帧尾 0x00
} SBUS_raw_t;
#pragma pack(pop)

// SBUS解析后的数据结构
typedef struct
{
    // 16个通道数据(已转换为-660~+660范围，与DBUS兼容)
    struct
    {
        int16_t Ch1;  // 通道1 (右摇杆X)
        int16_t Ch2;  // 通道2 (左摇杆Y)
        int16_t Ch3;  // 通道3 (右摇杆Y)
        int16_t Ch4;  // 通道4 (左摇杆X)
        int16_t Ch5;  // 通道5 (SF 2)
        int16_t Ch6;  // 通道6 (SE 3)
        int16_t Ch7;  // 通道7
        int16_t Ch8;  // 通道8 (SB 3)
        int16_t Ch9;  // 通道9 (SC 3)
        int16_t Ch10; // 通道10 (SD 3)
        int16_t Ch11; // 通道11 (SG 3)
        int16_t Ch12; // 通道12 (SH 2) 有弹簧
        int16_t Ch13; // 通道13 (LD 左旋钮)
        int16_t Ch14; // 通道14 (RD 右旋钮)
        int16_t Ch15; // 通道15 (左波轮)
        int16_t Ch16; // 通道16 (右波轮)
    } rc;

    // 开关数据
    uint8_t S1; // 开关1 (1=下, 2=上, 3=中)
    uint8_t S2; // 开关2 (1=下, 2=上, 3=中) 

    uint8_t Frame_flag; // 帧标志位
} SBUS_ctrl_t;

/**
 * @brief 获取SBUS遥控器数据
 * @param sbus_uart_handle SBUS串口句柄(需配置为100kbps, 8位数据, 偶校验, 2停止位)
 * @return SBUS_ctrl_t* 返回SBUS数据结构指针(包含当前数据和上一次数据)
 * @note 返回的是大小为2的数组，索引0为当前数据(CURRENT)，索引1为上一次数据(LAST)
 */
SBUS_ctrl_t *SBUS_Data_Get(UART_HandleTypeDef *sbus_uart_handle);

#endif // _SBUS_H_
