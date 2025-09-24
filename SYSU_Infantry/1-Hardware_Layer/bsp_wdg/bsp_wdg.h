#ifndef MONITOR_H
#define MONITOR_H

#include "stdint.h"
#include "string.h"

#define WATCHDOG_MX_NUM 64

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);

/* watchdog结构体定义 */
typedef struct 
{
    uint16_t reload_count;     // 重载值
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    uint16_t temp_count; // 当前值,减为零说明模块离线或异常
    void *owner_id;      // watchdog实例的地址,初始化的时候填入
} Watchdog_device_t;

/* watchdog初始化配置 */
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    void *owner_id;            // id取拥有watchdog的实例的地址,如DJIMotorInstance*,cast成void*类型
} Watchdog_init_t;

/**
 * @brief 注册一个watchdog实例
 *
 * @param config 初始化配置
 * @return Watchdog_device_t* 返回实例指针
 */
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config);

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance watchdog实例指针
 */
void Watchdog_feed(Watchdog_device_t *instance);

/**
 * @brief 确认模块是否离线
 *
 * @param instance watchdog实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t Watchdog_is_online(Watchdog_device_t *instance);

/**
 * @brief 放入rtos中,会给每个watchdog实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void Watchdog_task();

#endif // !MONITOR_H