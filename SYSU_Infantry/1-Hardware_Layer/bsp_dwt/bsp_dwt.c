/**
 * @file    bsp_dwt.c
 * @brief   STM32 DWT高精度计时实现
 */

#include "bsp_dwt.h"

static DWT_Time_t SysTime = {0};
static uint32_t CPU_FREQ_Hz = 0;
static uint32_t CPU_FREQ_Hz_ms = 0;
static uint32_t CPU_FREQ_Hz_us = 0;
static uint32_t CYCCNT_RountCount = 0;
static uint32_t CYCCNT_LAST = 0;
static uint64_t CYCCNT64 = 0;
static uint8_t  dt_initialized = 0; // 初始化标志位

/**
 * @brief 初始化DWT
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 1. 使能DWT外设 (DEMCR: Debug Exception and Monitor Control Register) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 2. 清零计数器 */
    DWT->CYCCNT = 0;

    /* 3. 启动计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 4. 设置频率参数 */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    CYCCNT_RountCount = 0;
    CYCCNT_LAST = 0;
    dt_initialized = 1; // 标记初始化完成
}

/**
 * @brief 内部函数：更新64位计数器
 * @note  加入临界区保护，防止中断打断导致计数错乱
 */
static void DWT_CNT_Update(void)
{
    if (!dt_initialized) return;

    // 进入临界区 (关中断)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    volatile uint32_t cnt_now = DWT->CYCCNT;

    // 检测是否发生溢出 (当前值小于上次值，说明绕了一圈)
    if (cnt_now < CYCCNT_LAST)
    {
        CYCCNT_RountCount++;
    }
    CYCCNT_LAST = cnt_now;

    // 更新64位总计数值
    CYCCNT64 = ((uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX) + (uint64_t)cnt_now;

    // 退出临界区 (恢复中断状态)
    if (!primask) __enable_irq();
}

/**
 * @brief 获取两次调用之间的时间间隔 (秒)
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    // 如果未初始化，返回一个安全的非零值，防止PID除零崩溃
    if (!dt_initialized) return 0.001f;

    volatile uint32_t cnt_now = DWT->CYCCNT;

    // 32位无符号减法会自动处理溢出回绕，无需额外判断
    // 前提：两次调用间隔不能超过 2^32 / CPU_FREQ 秒 (约25秒)
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));

    *cnt_last = cnt_now;

    DWT_CNT_Update(); // 顺便更新一下总时间轴

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    if (!dt_initialized || CPU_FREQ_Hz == 0) return;

    DWT_CNT_Update();

    // 更新结构体时间 (除法运算较慢，仅在需要时计算)
    uint64_t temp_cyc = CYCCNT64;

    SysTime.s = temp_cyc / CPU_FREQ_Hz;
    uint64_t remain = temp_cyc % CPU_FREQ_Hz;

    SysTime.ms = remain / CPU_FREQ_Hz_ms;
    remain = remain % CPU_FREQ_Hz_ms;

    SysTime.us = remain / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
}

/* --- 优化后的延时函数 (去除浮点运算，防止卡死) --- */

void DWT_Delay(float Delay)
{
    if (!dt_initialized) return;
    uint32_t tickstart = DWT->CYCCNT;
    // 预先计算需要的 tick 数，避免在循环中做浮点乘法
    uint32_t wait = (uint32_t)(Delay * (float)CPU_FREQ_Hz);

    while ((DWT->CYCCNT - tickstart) < wait);
}

void DWT_Delay_ms(uint32_t delay_ms)
{
    if (!dt_initialized) return;
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait = delay_ms * CPU_FREQ_Hz_ms;

    while ((DWT->CYCCNT - tickstart) < wait);
}

void DWT_Delay_us(uint32_t delay_us)
{
    if (!dt_initialized) return;
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait = delay_us * CPU_FREQ_Hz_us;

    while ((DWT->CYCCNT - tickstart) < wait);
}