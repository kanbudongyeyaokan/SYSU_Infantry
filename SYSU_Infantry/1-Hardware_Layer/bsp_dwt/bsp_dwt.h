/**
* @file    bsp_dwt.h
 * @brief   STM32 DWT高精度计时模块 (Robust Version)
 * @note    必须确保 DWT_Init 在所有使用 DWT 的功能之前被调用！
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/**
 * @brief 初始化DWT (必须最先调用)
 * @param CPU_Freq_mHz CPU主频 (如 168 或 180)
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔 (秒)
 * @param cnt_last 上一次的计数器值指针
 * @return float 时间间隔
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取当前系统时间 (秒)
 */
float DWT_GetTimeline_s(void);

/**
 * @brief DWT微秒级延时 (阻塞式)
 * @param delay_us 延时微秒数
 */
void DWT_Delay(float Delay);
void DWT_Delay_ms(uint32_t delay_ms);
void DWT_Delay_us(uint32_t delay_us);

/**
 * @brief 手动更新时间轴 (防止长时间不调用导致的溢出)
 */
void DWT_SysTimeUpdate(void);

#endif /* _BSP_DWT_H */