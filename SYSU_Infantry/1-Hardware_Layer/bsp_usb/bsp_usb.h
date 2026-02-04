#ifndef __BSP_USB_H
#define __BSP_USB_H

#include "main.h"
#include "cmsis_os.h" // 必须包含，用于互斥锁
#include <stdint.h>

// ==========================================
// 用户配置区
// ==========================================
// 发送缓冲区大小 (字节)
// 建议：至少 1024，如果是传输图像/大量波形，建议 4096
#define USB_FIFO_SIZE  2048

// ==========================================
// 数据结构定义
// ==========================================

// 接收回调函数类型定义
typedef void (*usb_rx_callback)(uint8_t* buf, uint32_t len);

typedef struct {
    // --- 发送部分 ---
    uint8_t  tx_fifo[USB_FIFO_SIZE]; // 环形缓冲区
    uint16_t head;                   // 写指针 (Head)
    uint16_t tail;                   // 读指针 (Tail)
    uint16_t last_sent_len;          // 记录上一次请求发送的长度 (用于中断更新 Tail)
    volatile uint8_t is_busy;        // 硬件忙标志位 (1:正在发送, 0:空闲)

    // --- 线程安全 ---
    osMutexId mutex;                 // 互斥锁，保护 FIFO 写操作

    // --- 接收部分 ---
    usb_rx_callback rx_cb;           // 接收回调函数指针
} Usb_Instance_t;

// ==========================================
// 函数声明
// ==========================================

/**
 * @brief 初始化 USB 虚拟串口库
 * @param cb 接收到数据时的回调函数
 */
void Usb_Init(usb_rx_callback cb);

/**
 * @brief 发送二进制数据 (线程安全、非阻塞)
 * @param data 数据指针
 * @param len  长度
 */
void Usb_Send(uint8_t* data, uint16_t len);

/**
 * @brief 格式化打印 (类似 printf)
 * @note  请注意调用此函数的任务栈大小，内部占用了约 256 字节栈空间
 */
void Usb_Printf(const char* fmt, ...);

// --- 钩子函数 (仅供 usbd_cdc_if.c 调用，用户勿用) ---
void Usb_Rx_Hook(uint8_t* Buf, uint32_t Len);
void Usb_TxCplt_Hook(void);

#endif // __BSP_USB_H