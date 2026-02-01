// #ifndef _HWT606_UART_H
// #define _HWT606_UART_H
//
// #include "main.h"
// #include "usart.h"
// #include "bsp_usart.h" // 引入你的串口库
//
// // HWT606 协议常量
// #define HWT_HEADER      0x55
// #define HWT_TYPE_ANGLE  0x53
// #define HWT_DATA_LEN    11    // 一个完整数据包长度
//
// #pragma pack(push, 1)
//
// // 原始数据包结构 (方便理解协议，实际解析时使用数组操作更灵活)
// typedef struct {
//     uint8_t header;
//     uint8_t type;
//     uint8_t roll_L;
//     uint8_t roll_H;
//     uint8_t pitch_L;
//     uint8_t pitch_H;
//     uint8_t yaw_L;
//     uint8_t yaw_H;
//     uint8_t ver_L;
//     uint8_t ver_H;
//     uint8_t sum;
// } HWT606_Raw_Packet_t;
//
// // 解析后的数据结构
// typedef struct {
//     float roll;  // 横滚角 (单位: 度)
//     float pitch; // 俯仰角 (单位: 度)
//     float yaw;   // 偏航角 (单位: 度)
//
//     // 如果需要，可以添加加速度或角速度数据，目前仅针对0x53角度
//     uint16_t version; // 版本号
//     uint32_t update_count; //用于调试，看数据是否在刷新
// } HWT606_Data_t;
//
// #pragma pack(pop)
//
// /**
//  * @brief 初始化并注册HWT606串口
//  * @param hwt_uart_handle 指向STM32 HAL库的UART句柄
//  * @return HWT606_Data_t* 返回数据结构体指针
//  */
// HWT606_Data_t* HWT606_Init(UART_HandleTypeDef *hwt_uart_handle);
//
// /**
//  * @brief 获取当前IMU数据指针
//  */
// HWT606_Data_t* HWT606_GetData(void);
//
// #endif // _HWT606_UART_H