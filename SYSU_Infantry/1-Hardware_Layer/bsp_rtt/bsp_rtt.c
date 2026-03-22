#include "bsp_rtt.h"
#include "SEGGER_RTT.h"
#include <stdarg.h>
#include <stdio.h>

#define RTT_MAX_CUSTOM_CHANNELS 4  // 最大支持自动配置的非0通道数量
#define RTT_BUFFER_SIZE 1024       // 每个通道的缓冲区大小

// 静态内存池：给非0通道准备的内存，外界无需关心
static uint8_t rtt_buffers[RTT_MAX_CUSTOM_CHANNELS][RTT_BUFFER_SIZE];
static uint8_t rtt_init_flags[RTT_MAX_CUSTOM_CHANNELS] = {0};

void Rtt_Printf(uint8_t channel, const char *fmt, ...)
{
    // 自动“懒加载”初始化 (仅限非0通道，通道0由原厂库默认管理)
    if (channel > 0 && channel < RTT_MAX_CUSTOM_CHANNELS) {
        if (rtt_init_flags[channel] == 0) {
            // 给通道起个默认名字，比如 "CH1", "CH2"
            static char ch_names[RTT_MAX_CUSTOM_CHANNELS][8];
            snprintf(ch_names[channel], sizeof(ch_names[channel]), "CH%d", channel);
            
            // 静默配置通道，使用 NO_BLOCK_SKIP 防止高频打印卡死单片机
            SEGGER_RTT_ConfigUpBuffer(channel, 
                                      ch_names[channel], 
                                      rtt_buffers[channel], 
                                      RTT_BUFFER_SIZE, 
                                      SEGGER_RTT_MODE_NO_BLOCK_SKIP);
            rtt_init_flags[channel] = 1;
        }
    }

    // 格式化处理 (支持浮点数)
    char temp_buf[256]; 
    va_list args;
    va_start(args, fmt);
    // 使用标准库 vsnprintf 转化，确保浮点数能被正常解析出来
    int len = vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);
    va_end(args);

    // 直接通过底层的原生接口扔出去
    if (len > 0) {
        SEGGER_RTT_Write(channel, temp_buf, len);
    }
}