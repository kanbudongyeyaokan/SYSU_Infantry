#include "bsp_wdg.h"
#include "bsp_dwt.h" 
#include "stdlib.h"


// 用于保存所有的 instance
static Instance *_instances[_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的 instance数量,配合回调使用

Instance *Register(_Init_Config_s *config)
{
    Instance *instance = (Instance *)malloc(sizeof(Instance));
    memset(instance, 0, sizeof(Instance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数

    instance->temp_count = config->reload_count;
    _instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void Reload(Instance *instance)
{
    instance->temp_count = instance->reload_count;
}

uint8_t IsOnline(Instance *instance)
{
    return instance->temp_count > 0;
}

void Task()
{
    Instance *dins; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {

        dins = _instances[i];
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            dins->temp_count--;
        else if (dins->callback) // 等于零说明超时了,调用回调函数(如果有的话)
        {
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}
// (需要id的原因是什么?) 下面是copilot的回答!
// 需要id的原因是因为有些module可能有多个实例,而我们需要知道具体是哪个实例offline
// 如果只有一个实例,则可以不用id,直接调用callback即可
// 比如: 有一个module叫做"电机",它有两个实例,分别是"电机1"和"电机2",那么我们调用电机的离线处理函数时就需要知道是哪个电机offline
