#include "bsp_wdg.h"
#include "bsp_dwt.h" 
#include "stdlib.h"

/************************看门狗注册者管理*****************************/
static Watchdog_device_t *wdg_register[WATCHDOG_MX_NUM] = {NULL};
static uint8_t idx; // 用于记录当前的看门狗数量,配合回调使用


/**
 * @brief 注册一个watchdog实例
 *
 * @param config 初始化配置
 * @return Watchdog_device_t* 返回实例指针
 */
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config)
{
    Watchdog_device_t *instance = (Watchdog_device_t *)malloc(sizeof(Watchdog_device_t));
    memset(instance, 0, sizeof(Watchdog_device_t));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->temp_count = config->reload_count;
    wdg_register[idx++] = instance;
    return instance;
}

/**
 * @brief 放入rtos中,会给每个watchdog实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void Watchdog_control_all()
{
    Watchdog_device_t *current_dog; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {

        current_dog = wdg_register[i];
        if (current_dog->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            current_dog->temp_count--;
        else if (current_dog->callback) // 等于零说明超时了,调用回调函数(如果有的话)
        {
            current_dog->callback(current_dog->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
        }
    }
}

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance watchdog实例指针
 */
void Feed_watchdog(Watchdog_device_t *instance)
{
    instance->temp_count = instance->reload_count;
}

/**
 * @brief 确认模块是否离线
 *
 * @param instance watchdog实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t Watchdog_is_online(Watchdog_device_t *instance)
{
    return instance->temp_count > 0;
}



