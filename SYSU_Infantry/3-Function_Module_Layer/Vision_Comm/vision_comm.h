#ifndef __VISION_COMM_H
#define __VISION_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==========================================
// 协议宏定义
// ==========================================
#define VISION_SOF           0xA5
#define CMD_ID_INFANTRY      0x0101   // 步兵视觉指令
#define CMD_ID_SENTRY        0x0102   // 哨兵视觉指令(备用)
#define VISION_RX_FIFO_SIZE  1024     // 视觉接收缓冲区大小(建议设大一点防溢出)

// ==========================================
// 数据结构定义 (严格1字节对齐)
// ==========================================
#pragma pack(push, 1)

// 帧头结构体 (5字节)
typedef struct {
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} Vision_Frame_Header_t;

// 步兵视觉接收数据段结构体 (9字节)
typedef struct {
    uint8_t  vision_flags;    // 视觉状态标志位 (bit0: is_detected, bit1: is_tracking, bit2: is_fire)
    float    yaw_angle;       // 目标绝对 Yaw 角度 (deg)
    float    pitch_angle;     // 目标绝对 Pitch 角度 (deg)
} Infantry_Vision_Rx_Data_t;

#pragma pack(pop)

// ==========================================
// 外部接口函数声明
// ==========================================

/**
 * @brief  视觉通信模块初始化
 * @note   需在 FreeRTOS 调度器启动前、USB初始化后调用
 */
void Vision_Comm_Init(void);

/**
 * @brief  视觉数据解析处理函数
 * @note   ★强烈建议放在 Gimbal_Task 的 while(1) 循环最开头高频调用★
 */
void Vision_Comm_Parse_Task(void);

/**
 * @brief  获取最新的一帧有效视觉数据
 * @return 指向最新数据的指针
 */
const Infantry_Vision_Rx_Data_t* Get_Vision_Data(void);

/**
 * @brief  检查视觉是否离线
 * @return true: 正常在线, false: 已掉线
 */
bool Is_Vision_Online(void);

#endif // __VISION_COMM_H