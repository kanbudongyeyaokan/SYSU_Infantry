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
    instance->offline_callback = config->callback;
    instance->online_callback = config->online_callback;
    // 初始化状态
    instance->temp_count = instance->reload_count;
    instance->is_offline = 0;

    wdg_register[idx++] = instance;
    return instance;
}


void Watchdog_feed(Watchdog_device_t *instance)
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

                if (current_dog->offline_callback)
                {
                    current_dog->offline_callback(current_dog->owner_id);
                }
            }
        }
    }
}

uint8_t Watchdog_is_online(Watchdog_device_t *instance)
{
    if (instance == NULL) return 0;
    return (instance->is_offline == 0) ? 1 : 0;
}