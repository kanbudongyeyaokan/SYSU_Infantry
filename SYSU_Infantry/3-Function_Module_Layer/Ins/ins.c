#include "ins.h"
#include "bsp_dwt.h" 
#include <stddef.h> 

static const Ins_driver_interface_t *current_driver = NULL;

static Ins_data_t g_ins_data;

// 上一次更新的时间戳
static float last_time_s = 0.0f;

// ================= API 实现 =================

// 初始化函数
void Ins_init(const Ins_driver_interface_t *driver_impl)
{
    if (driver_impl == NULL) return;
    current_driver = driver_impl;
    g_ins_data.state = INS_STATE_INIT;

    if (current_driver->init != NULL) {
        if (current_driver->init()) {
            g_ins_data.state = INS_STATE_READY; // 初始化成功
        } else {
            g_ins_data.state = INS_STATE_ERROR; // 初始化失败
        }
    }

    // 记录当前时间，为下一次计算 dt 做准备
    last_time_s = DWT_GetTimeline_s();
}

// 核心更新函数 
void Ins_update(void)
{
    // 如果没有注册驱动，或者处于错误状态，就不跑了
    if (current_driver == NULL) return;

    // --- 计算时间间隔 dt ---
    float now_s = DWT_GetTimeline_s();
    float dt = now_s - last_time_s;

    // 保护：防止时间倒流或过小导致除以零错误
    if (dt <= 0.0001f) dt = 0.001f;

    last_time_s = now_s;
    g_ins_data.dt_s = dt;

    // 启动读取 (Kick) 
    // 对应 BMI088 就是启动 DMA 传输
    if (current_driver->start_read) {
        current_driver->start_read();
    }

    if (current_driver->wait_data && current_driver->wait_data())
    {
        // --- 处理数据 (Process) ---
        if (current_driver->process_data) {
            current_driver->process_data(&g_ins_data, dt);
        }

        // 更新状态为就绪
        g_ins_data.state = INS_STATE_READY;
    }
    else {
        // 如果 wait_data 返回 false，说明超时了
        g_ins_data.state = INS_STATE_ERROR;
    }
}

// 获取数据的接口
const Ins_data_t* Ins_get_data(void)
{
    return &g_ins_data;
}