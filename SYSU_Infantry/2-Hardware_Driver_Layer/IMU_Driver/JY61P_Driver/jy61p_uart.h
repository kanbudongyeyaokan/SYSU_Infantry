/**
 * @file    jy61p_uart.h
 * @brief   JY61P 串口协议解析助手
 */

#ifndef JY61P_UART_H
#define JY61P_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_usart.h"
#include "main.h"

typedef struct {
    float x;
    float y;
    float z;
} JY61P_Vector3f_t;

typedef struct {
    JY61P_Vector3f_t acc_mps2;   /* 加速度，单位 m/s^2 */
    JY61P_Vector3f_t gyro_dps;   /* 角速度，单位 deg/s */
    JY61P_Vector3f_t euler_deg;  /* 欧拉角，单位 deg */

    float temp_deg_c;            /* 温度，单位摄氏度 */
    uint16_t version;            /* 版本号（0x53 帧最后两个字节） */
    uint16_t voltage_raw;        /* 0x52 帧中的附加电压字段原始值 */

    uint32_t update_count;       /* 角度帧（0x53）更新次数 */
    uint32_t frame_count;        /* 有效帧计数 */
    uint32_t crc_error_count;    /* 校验错误计数 */
} JY61P_UART_Data_t;

/* 初始化串口解析器并注册回调，成功返回数据指针。 */
JY61P_UART_Data_t *JY61P_UART_Init(UART_HandleTypeDef *uart_handle);
/* 获取当前缓存的解析结果。 */
JY61P_UART_Data_t *JY61P_UART_GetData(void);

#endif /* JY61P_UART_H */
