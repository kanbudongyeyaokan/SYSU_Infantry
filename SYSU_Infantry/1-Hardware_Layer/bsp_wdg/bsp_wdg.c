/**
 * @file    bsp_wdg.c
 * @brief   软件看门狗驱动实现
 * @author  SYSU电控组
 * @date    2026-02-10
 * @version 1.0
 * @note    提供软件看门狗的注册、喂狗、监控和状态查询功能
 */

#include "bsp_wdg.h"
#include <string.h>

// 注册表，存储所有已注册的看门狗实例
static Watchdog_device_t* wdg_register[WATCHDOG_MX_NUM] = { NULL };
// 当前已注册的数量
static uint8_t idx = 0;

/**
 * @brief   注册一个新的看门狗实例
 * @param   config: 看门狗初始化配置结构体指针
 * @return  Watchdog_device_t*: 返回分配的看门狗实例指针
 */
Watchdog_device_t* Watchdog_register(Watchdog_init_t* config)
{
    if (idx >= WATCHDOG_MX_NUM) return NULL;

    Watchdog_device_t *instance = (Watchdog_device_t *)malloc(sizeof(Watchdog_device_t));
    if (instance == NULL) return NULL;

    memset(instance, 0, sizeof(Watchdog_device_t));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;
    instance->offline_callback = config->callback;
    instance->online_callback = config->online_callback;
    strcpy(instance->name, config->name);
    // 初始化状态
    instance->temp_count = instance->reload_count;
    instance->is_offline = 0;

    wdg_register[idx++] = instance;
    return instance;
}

/**
 * @brief   喂狗函数，重置计数器
 * @param   instance: 看门狗实例指针
 */
void Watchdog_feed(Watchdog_device_t* instance)
{
    if (instance == NULL) return;

    // 重载计数器 (原子写)
    instance->temp_count = instance->reload_count;

    // 处理上线逻辑
    // 如果之前是离线状态，现在喂狗了，说明设备复活了
    if (instance->is_offline == 1)
    {
        instance->is_offline = 0; // 标记为在线

        // 触发上线回调
        if (instance->online_callback) {
            instance->online_callback(instance->owner_id);
        }
    }
}

/**
 * @brief   看门狗总控函数，处理所有看门狗的计数递减和超时检测
 * @note    需在定时器或任务中周期性调用
 */
void Watchdog_control_all(void)
{
    Watchdog_device_t *current_dog;

    for (size_t i = 0; i < idx; ++i)
    {
        current_dog = wdg_register[i];
        if (current_dog == NULL) continue;

        if (current_dog->temp_count > 0)
        {
            current_dog->temp_count--;
        }
        else
        {
            // 计数归零，说明超时
            if (current_dog->is_offline == 0)
            {
                current_dog->is_offline = 1; // 标记为已离线

            }
            // 无论首次还是持续离线，都触发回调，实现持续报警
            if (current_dog->offline_callback)
            {
                current_dog->offline_callback(current_dog->owner_id);
            }
            // 重置计数器，下个周期继续检测并重复报警
            current_dog->temp_count = current_dog->reload_count;
        }
    }
}

/**
 * @brief   查询设备在线状态
 * @param   instance: 看门狗实例指针
 * @return  uint8_t: 1 表示在线，0 表示离线
 */
uint8_t Watchdog_is_online(Watchdog_device_t* instance)
{
    if (instance == NULL) return 0;
    return (instance->is_offline == 0) ? 1 : 0;
}