/**
* @file    bsp_wdg.c
 * @brief   软件看门狗实现
 */

#include "bsp_wdg.h"
#include <string.h>

static Watchdog_device_t *wdg_register[WATCHDOG_MX_NUM] = {NULL};
static uint8_t idx = 0;

Watchdog_device_t *Watchdog_register(Watchdog_init_t *config)
{
    if (idx >= WATCHDOG_MX_NUM) return NULL; // 防止越界

    Watchdog_device_t *instance = (Watchdog_device_t *)malloc(sizeof(Watchdog_device_t));
    if (instance == NULL) return NULL; // 防止内存分配失败

    memset(instance, 0, sizeof(Watchdog_device_t));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;
    instance->callback = config->callback;

    // 初始化时默认为离线还是在线？通常设为最大值，等待第一次喂狗
    instance->temp_count = instance->reload_count;
    instance->is_offline = 0; // 默认在线，或者可以设为1等待第一次数据

    wdg_register[idx++] = instance;
    return instance;
}

void Watchdog_feed(Watchdog_device_t *instance)
{
    if (instance == NULL) return;

    // 重载计数器
    instance->temp_count = instance->reload_count;

    // 恢复在线状态
    // 这里不需要立即调用“上线回调”，通常只需要处理“离线异常”
    instance->is_offline = 0;
}

void Watchdog_control_all(void)
{
    Watchdog_device_t *current_dog;

    for (size_t i = 0; i < idx; ++i)
    {
        current_dog = wdg_register[i];
        if (current_dog == NULL) continue;

        if (current_dog->temp_count > 0)
        {
            // 计数递减
            current_dog->temp_count--;
        }
        else
        {
            // 计数归零，说明超时

            // 核心修复：只有当状态是“在线”转为“离线”的那一次，才执行回调
            if (current_dog->is_offline == 0)
            {
                current_dog->is_offline = 1; // 标记为已离线

                if (current_dog->callback)
                {
                    current_dog->callback(current_dog->owner_id);
                }
            }
        }
    }
}