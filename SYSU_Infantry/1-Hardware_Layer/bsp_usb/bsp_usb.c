#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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

    // 创建互斥锁 (CMSIS-RTOS API)
    osMutexDef(usb_lock);
    usb_inst.mutex = osMutexCreate(osMutex(usb_lock));

    osDelay(1500); // 等待 USB 初始化完成
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

    // --- 加锁 (保护 FIFO) ---
    if (osMutexWait(usb_inst.mutex, 10) != osOK) return;

    // 计算 FIFO 剩余空间
    uint16_t space;
    if (usb_inst.head >= usb_inst.tail) {
        space = USB_FIFO_SIZE - (usb_inst.head - usb_inst.tail);
    } else {
        space = usb_inst.tail - usb_inst.head;
    }

    // 如果空间不足 (留 1 字节防止完全重合)，放弃发送或根据需求在此等待
    if (space <= len + 1) {
        osMutexRelease(usb_inst.mutex);
        return; 
    }

    // 将数据写入 FIFO (处理回绕)
    for (uint16_t i = 0; i < len; i++) {
        usb_inst.tx_fifo[usb_inst.head] = data[i];
        usb_inst.head++;
        if (usb_inst.head >= USB_FIFO_SIZE) {
            usb_inst.head = 0;
        }
    }

    // --- 解锁 ---
    osMutexRelease(usb_inst.mutex);

    // --- 尝试触发发送 ---
    // 进入临界区：防止 "判断 busy 为 0" 后，瞬间被中断打断并设为 1，导致冲突
    taskENTER_CRITICAL();
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