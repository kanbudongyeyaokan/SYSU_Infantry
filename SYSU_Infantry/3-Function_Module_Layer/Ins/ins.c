#include "ins.h"
#include "bsp_dwt.h" 
#include <stddef.h> 
#include "error_handler.h"

static const Ins_driver_interface_t *current_driver = NULL;

static Ins_data_t g_ins_data;

// 上一次更新的时间戳
static float last_time_s = 0.0f;
static uint32_t ins_wait_timeout_count = 0u;

#define INS_ERROR_REPORT_INTERVAL_MS  200u
#define INS_TIMEOUT_CRITICAL_THRESHOLD 200u
static uint32_t ins_last_no_driver_tick = 0u;
static uint32_t ins_last_dt_abnormal_tick = 0u;
static uint32_t ins_last_wait_timeout_tick = 0u;
static uint32_t ins_last_wait_timeout_critical_tick = 0u;

static uint8_t Ins_Should_Report(uint32_t *last_tick, uint32_t interval_ms)
{
    uint32_t now = HAL_GetTick();
    if ((now - *last_tick) >= interval_ms)
    {
        *last_tick = now;
        return 1u;
    }
    return 0u;
}

// ================= API 实现 =================

// 初始化函数
void Ins_init(const Ins_driver_interface_t *driver_impl)
{
    if (driver_impl == NULL)
    {
        current_driver = NULL;
        g_ins_data.state = INS_STATE_ERROR;
        ERROR_CRITICAL("INS", "Ins_init driver is NULL");
        return;
    }
    current_driver = driver_impl;
    g_ins_data.state = INS_STATE_INIT;

    if (current_driver->init != NULL) {
        if (current_driver->init()) {
            g_ins_data.state = INS_STATE_READY; // 初始化成功
            ERROR_INFO("INS", "INS init success");
        } else {
            g_ins_data.state = INS_STATE_ERROR; // 初始化失败
            ERROR_CRITICAL("INS", "INS driver init failed");
        }
    } else {
        g_ins_data.state = INS_STATE_ERROR;
        ERROR_RAISE("INS", "INS driver init function is NULL");
    }

    // 记录当前时间，为下一次计算 dt 做准备
    last_time_s = DWT_GetTimeline_s();
}

// 核心更新函数 
void Ins_update(void)
{
    // 如果没有注册驱动，或者处于错误状态，就不跑了
    if (current_driver == NULL)
    {
        if (Ins_Should_Report(&ins_last_no_driver_tick, INS_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_CRITICAL("INS", "Ins_update called without driver");
        }
        g_ins_data.state = INS_STATE_ERROR;
        return;
    }

    // --- 计算时间间隔 dt ---
    float now_s = DWT_GetTimeline_s();
    float dt = now_s - last_time_s;

    // 保护：防止时间倒流或过小导致除以零错误
    if (dt <= 0.0001f || dt > 0.1f)
    {
        if (Ins_Should_Report(&ins_last_dt_abnormal_tick, INS_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_WARN("INS", "INS dt abnormal dt_us=%lu now_ms=%lu last_ms=%lu",
                       (uint32_t)(dt * 1000000.0f),
                       (uint32_t)(now_s * 1000.0f),
                       (uint32_t)(last_time_s * 1000.0f));
        }
        if (dt <= 0.0001f) dt = 0.001f;
    }

    last_time_s = now_s;
    g_ins_data.dt_s = dt;

    // 启动读取 (Kick) 
    // 对应 BMI088 就是启动 DMA 传输
    if (current_driver->start_read) {
        current_driver->start_read();
    } else {
        g_ins_data.state = INS_STATE_ERROR;
        ERROR_RAISE("INS", "INS start_read is NULL");
        return;
    }

    if (current_driver->wait_data == NULL)
    {
        g_ins_data.state = INS_STATE_ERROR;
        ERROR_RAISE("INS", "INS wait_data is NULL");
        return;
    }

    if (current_driver->wait_data())
    {
        if (ins_wait_timeout_count > 0u)
        {
            ERROR_INFO("INS", "INS wait recovered timeout_cnt=%lu dt_us=%lu",
                       ins_wait_timeout_count,
                       (uint32_t)(dt * 1000000.0f));
            ins_wait_timeout_count = 0u;
        }

        // --- 处理数据 (Process) ---
        if (current_driver->process_data) {
            current_driver->process_data(&g_ins_data, dt);
        } else {
            g_ins_data.state = INS_STATE_ERROR;
            ERROR_RAISE("INS", "INS process_data is NULL");
            return;
        }

        // 更新状态为就绪
        g_ins_data.state = INS_STATE_READY;
    }
    else {
        // 如果 wait_data 返回 false，说明超时了
        g_ins_data.state = INS_STATE_ERROR;
        ins_wait_timeout_count++;

        if (Ins_Should_Report(&ins_last_wait_timeout_tick, INS_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_RAISE("INS", "INS wait timeout timeout_cnt=%lu dt_us=%lu",
                        ins_wait_timeout_count,
                        (uint32_t)(dt * 1000000.0f));
        }

        if (ins_wait_timeout_count >= INS_TIMEOUT_CRITICAL_THRESHOLD &&
            Ins_Should_Report(&ins_last_wait_timeout_critical_tick, INS_ERROR_REPORT_INTERVAL_MS))
        {
            ERROR_CRITICAL("INS", "INS consecutive timeout timeout_cnt=%lu crit_threshold=%lu",
                           ins_wait_timeout_count,
                           INS_TIMEOUT_CRITICAL_THRESHOLD);
        }
    }
}

// 获取数据的接口
const Ins_data_t* Ins_get_data(void)
{
    return &g_ins_data;
}
