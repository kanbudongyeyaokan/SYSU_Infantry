#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "error_handler.h"

// 单例实例 (STM32 通常只有一个 USB Device)
static Usb_Instance_t usb_inst;

// 内部函数声明
static void Usb_Try_Transmit(void);

// ============================================================
// 初始化
// ============================================================
void Usb_Init(usb_rx_callback cb) {
    // 清零结构体
    memset(&usb_inst, 0, sizeof(Usb_Instance_t));
    
    // 注册接收回调
    usb_inst.rx_cb = cb;

    ERROR_INFO("USB","Usb Init complete");
    // mutex 已移除：Usb_Send 改用 taskENTER_CRITICAL 统一保护 FIFO，
    // 可同时防止任务抢占和 TxCplt ISR 并发访问 head/tail。
    // USB 枚举由硬件中断异步完成，此处仅完成 USB 实例与互斥锁等软件栈初始化，尚未完成与主机的枚举过程。
    // 在主机完成枚举之前，直接调用 CDC_Transmit_FS 可能返回失败（如 USBD_BUSY/USBD_FAIL），本模块会丢弃本次发送长度记录，
    // 但不会自动重试；只有在后续再次触发 Usb_Try_Transmit（例如通过 Usb_Send 发送数据，或在显式“枚举完成”回调中调用）
    // 时才会重新尝试发送队列中的数据。
}

// ============================================================
// 尝试提交发送请求
// ============================================================
static void Usb_Try_Transmit(void) {
    // 如果硬件正在忙，直接退出，等待 TxCplt 中断回调再次触发
    if (usb_inst.is_busy) return;
    
    // 如果缓冲区空，直接退出
    if (usb_inst.head == usb_inst.tail) return;

    // 计算本次可以发送的连续数据长度
    uint16_t send_len = 0;                           
    if (usb_inst.head > usb_inst.tail) {
        // 情况 A: [ ... T ...... H ... ]  未回绕，直接发中间这一段
        send_len = usb_inst.head - usb_inst.tail;
    } else {
        // 情况 B: [ ... H ...... T ... ]  回绕了，先发 Tail 到 End 这一段
        send_len = USB_FIFO_SIZE - usb_inst.tail;
    }

    // 记录本次请求发送的长度 (用于中断里更新 Tail)
    usb_inst.last_sent_len = send_len;

    // 调用 ST 底层驱动发送
    // 注意：CDC_Transmit_FS 返回 USBD_OK (0) 表示成功接收请求
    uint8_t result = CDC_Transmit_FS(&usb_inst.tx_fifo[usb_inst.tail], send_len);

    if (result == USBD_OK) {
        // 标记硬件忙
        usb_inst.is_busy = 1;
    } else {
        // 发送请求失败 (可能是 USB 未连接或底层忙)，清除长度记录，下次重试
        usb_inst.last_sent_len = 0;
    }
}

// ============================================================
// 用户接口：发送数据
// ============================================================
void Usb_Send(uint8_t* data, uint16_t len) {
    if (len == 0 || data == NULL) return;

    // 临界区同时防止：其他任务抢占 + TxCplt ISR 并发修改 tail/is_busy
    taskENTER_CRITICAL();

    // 计算 FIFO 剩余空间
    uint16_t space;
    if (usb_inst.head >= usb_inst.tail) {
        space = USB_FIFO_SIZE - (usb_inst.head - usb_inst.tail);
    } else {
        space = usb_inst.tail - usb_inst.head;
    }

    // 空间充足时写入；保留 1 字节防止 head==tail 被误判为空
    if (space > len) {
        for (uint16_t i = 0; i < len; i++) {
            usb_inst.tx_fifo[usb_inst.head] = data[i];
            usb_inst.head++;
            if (usb_inst.head >= USB_FIFO_SIZE) {
                usb_inst.head = 0;
            }
        }
    }
    // 空间不足时不写入，但仍触发发送以排空 FIFO，使 TX 链在枚举后能自动恢复。

    if (usb_inst.is_busy == 0) {
        Usb_Try_Transmit();
    }

    taskEXIT_CRITICAL();
}

// ============================================================
// 用户接口：格式化打印
// ============================================================
void Usb_Printf(const char* fmt, ...) {
    // 警告：这里的 buf 分配在任务栈上
    // 请确保调用此函数的 FreeRTOS 任务 Stack Size > 512 Bytes
    char buf[128];
    
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        Usb_Send((uint8_t*)buf, (uint16_t)len);
    }
}

// ============================================================
// 5. 钩子函数 ,放在usb_cdc_if.c里面调用
// ============================================================

// 接收钩子
void Usb_Rx_Hook(uint8_t* Buf, uint32_t Len) {
    if (usb_inst.rx_cb) {
        usb_inst.rx_cb(Buf, Len);
    }
}

// 发送完成钩子 (中断上下文)
void Usb_TxCplt_Hook(void) {
    // 硬件已经把数据发走了，现在更新 Tail 指针
    usb_inst.tail += usb_inst.last_sent_len;
    // 处理回绕
    if (usb_inst.tail >= USB_FIFO_SIZE) {
        usb_inst.tail = 0; // 或者 tail -= FIFO_SIZE
    }
    // 清除忙标志
    usb_inst.is_busy = 0;
    usb_inst.last_sent_len = 0;

    // 缓冲区可能还有剩余数据 (比如之前跨越了回绕点)，继续发送
    Usb_Try_Transmit();
}