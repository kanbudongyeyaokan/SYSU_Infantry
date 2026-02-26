#ifndef BSP_WDG_H
#define BSP_WDG_H

#include "stdint.h"
#include "stdlib.h"

#define WATCHDOG_MX_NUM 64

/* 回调函数指针类型 */
typedef void (*wdg_callback_func)(void *);

/* watchdog结构体定义 */
typedef struct
{
    uint16_t reload_count;     // 重载值
    volatile uint16_t temp_count; //加 volatile，防止编译器过度优化

    uint8_t  is_offline;       // 离线标志位

    wdg_callback_func offline_callback; // 离线回调
    wdg_callback_func online_callback;  // 上线回调

    void *owner_id;            // 被监控对象
    char name[16];
} Watchdog_device_t;

/* watchdog初始化配置 */
typedef struct
{
    uint16_t reload_count;
    wdg_callback_func callback;         // 默认离线回调
    wdg_callback_func online_callback;  // 上线回调
    void *owner_id;

    char name[16];
} Watchdog_init_t;

/**
 * @brief 注册一个watchdog实例
 */
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config);

/**
 * @brief 喂狗：在收到数据时调用 (ISR安全)
 */
void Watchdog_feed(Watchdog_device_t *instance);

/**
 * @brief 全局控制函数，需放入RTOS任务中循环调用
 */
void Watchdog_control_all(void);

/**
 * @brief 检查设备是否在线 (补全声明)
 */
uint8_t Watchdog_is_online(Watchdog_device_t *instance);

#endif // BSP_WDG_H