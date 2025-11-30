/**
 * @file    bsp_wdg.c
 * @brief   软件看门狗实现 (补充完整接口)
 */

#include "bsp_wdg.h"
#include <string.h>

static Watchdog_device_t *wdg_register[WATCHDOG_MX_NUM] = {NULL};
static uint8_t idx = 0;

Watchdog_device_t *Watchdog_register(Watchdog_init_t *config)
{
    if (idx >= WATCHDOG_MX_NUM) return NULL;

    Watchdog_device_t *instance = (Watchdog_device_t *)malloc(sizeof(Watchdog_device_t));
    if (instance == NULL) return NULL;

    memset(instance, 0, sizeof(Watchdog_device_t));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;
    instance->callback = config->callback;

    // 初始化状态
    instance->temp_count = instance->reload_count;
    instance->is_offline = 0; // 默认在线

    wdg_register[idx++] = instance;
    return instance;
}

void Watchdog_feed(Watchdog_device_t *instance)
{
    if (instance == NULL) return;

    // 1. 重载计数器
    instance->temp_count = instance->reload_count;

    // 2. 标记为在线
    // 这里非常重要：如果之前是离线的，现在收到了数据，说明“重连”了
    // 可以在这里加一个“上线回调”，但目前我们只需要把状态置0
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
            // 只有当状态从“在线(0)”转为“离线(1)”的那一次，才执行回调
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

/**
 * @brief 检查设备是否在线
 * @return 1:在线, 0:离线
 */
uint8_t Watchdog_is_online(Watchdog_device_t *instance)
{
    if (instance == NULL) return 0;

    // 如果 is_offline == 0，说明在线，返回 1
    // 如果 is_offline == 1，说明离线，返回 0
    return (instance->is_offline == 0) ? 1 : 0;
}