/**
 * @file    power_meter.h
 * @brief   底盘功率计数据解析模块
 * @author  SYSU电控组
 * @date    2025-03-22
 * @version 1.0
 * 
 * @note    解析 CAN ID 0x212 的功率计报文数据
 *          - 帧类型：标准帧
 *          - DLC：8 字节
 *          - 发送频率：1000Hz
 *          - 数据格式：小端序
 *          - 缩放系数：原始值 = 实际值 × 100
 */

#ifndef POWER_METER_H
#define POWER_METER_H

#include <stdint.h>
#include "can.h"

/* 功率计 CAN 接收 ID */
#define POWER_METER_RX_ID   0x212u

/**
 * @brief 功率计数据结构体
 * @note  存储解算后的真实物理量
 */
typedef struct {
    float real_voltage;     /* 真实电压 (单位: V) */
    float real_current;     /* 真实电流 (单位: A) */
    float real_power;       /* 真实功率 (单位: W) */
} PowerMeter_t;

/**
 * @brief 初始化功率计模块
 * @param hcan CAN句柄指针 (&hcan1 或 &hcan2)
 * @note  注册 CAN 接收回调，应在系统初始化时调用
 */
void PowerMeter_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief 解析功率计 CAN 报文数据
 * @param pm 功率计数据结构体指针
 * @param can_rx_data CAN 接收数据缓冲区 (8字节)
 * @note  使用位移操作拼接数据，避免指针强转的内存对齐风险
 *        数据格式：小端序，DATA[0-1]电压，DATA[2-3]电流
 */
void PowerMeter_Parse(PowerMeter_t *pm, const uint8_t *can_rx_data);

/**
 * @brief 获取功率计数据结构体
 * @return PowerMeter_t* 功率计数据指针
 * @note  返回内部静态实例的指针，可直接访问各物理量
 */
PowerMeter_t* PowerMeter_GetData(void);

/**
 * @brief 获取当前电压
 * @return float 电压值 (V)
 */
float PowerMeter_GetVoltage(void);

/**
 * @brief 获取当前电流
 * @return float 电流值 (A)
 */
float PowerMeter_GetCurrent(void);

/**
 * @brief 获取当前功率
 * @return float 功率值 (W)
 */
float PowerMeter_GetPower(void);

#endif /* POWER_METER_H */