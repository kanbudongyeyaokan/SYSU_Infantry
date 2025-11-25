/**
* @file    bsp_wdg.h
 * @brief   软件看门狗模块
 * @author  SYSU电控组
 * @date    2025-09-24
 * @version 2.0
 */

#ifndef BSP_WDG_H
#define BSP_WDG_H

#include "stdint.h"
#include "stdlib.h"

#define WATCHDOG_MX_NUM 64

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);

/* watchdog结构体定义 */
typedef struct
{
    uint16_t reload_count;     // 重载值 (超时时间 = reload_count * 任务周期)
    uint16_t temp_count;       // 当前倒计时

    uint8_t  is_offline;       // 离线标志位: 0-在线, 1-离线 (新增，用于防止回调重复触发)

    offline_callback callback; // 异常处理函数
    void *owner_id;            // 被监控对象的指针(如电机结构体)
} Watchdog_device_t;

/* watchdog初始化配置 */
typedef struct
{
    uint16_t reload_count;
    offline_callback callback;
    void *owner_id;
} Watchdog_init_t;

/**
 * @brief 注册一个watchdog实例
 * @param config 初始化配置
 * @return Watchdog_device_t* 返回实例指针
 */
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config);

/**
 * @brief 喂狗：在收到数据时调用
 * @param instance watchdog实例指针
 */
void Watchdog_feed(Watchdog_device_t *instance);

/**
 * @brief 全局控制函数，需放入RTOS任务中循环调用
 */
void Watchdog_control_all(void);

#endif // BSP_WDG_H