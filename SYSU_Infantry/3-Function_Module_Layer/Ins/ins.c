#include "ins.h"
#include "bsp_dwt.h" // 我们需要高精度时间戳来计算 dt
#include <stddef.h>  // 为了使用 NULL

// 1. 私有变量 (static)
// 语法：static 表示这个变量只在这个 .c 文件内可见，外部访问不到。
// 含义：保存当前的“合同”是谁签的。
static const Ins_driver_interface_t *current_driver = NULL;

// 全局唯一的 INS 数据实例
static Ins_data_t g_ins_data;

// 上一次更新的时间戳 (用于算 dt)
static float last_time_s = 0.0f;

// ================= API 实现 =================

// 初始化函数
void Ins_init(const Ins_driver_interface_t *driver_impl)
{
    // 防御性编程：如果传进来的指针是空的，直接返回，防止死机
    if (driver_impl == NULL) return;

    // 保存驱动接口指针
    current_driver = driver_impl;

    // 先标记状态为初始化
    g_ins_data.state = INS_STATE_INIT;

    // 调用驱动的初始化函数 (如果驱动提供了的话)
    // 语法：if (current_driver->init != NULL)
    // 含义：检查驱动里有没有写 init 函数？写了就调用，没写(NULL)就跳过。
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

// 核心更新函数 (Task 层循环调用的就是它)
void Ins_update(void)
{
    // 如果没有注册驱动，或者处于错误状态，就不跑了
    if (current_driver == NULL) return;

    // --- 步骤 1: 计算时间间隔 dt ---
    float now_s = DWT_GetTimeline_s();
    float dt = now_s - last_time_s;

    // 保护：防止时间倒流或过小导致除以零错误
    if (dt <= 0.0001f) dt = 0.001f;

    last_time_s = now_s;
    g_ins_data.dt_s = dt;

    // --- 步骤 2: 启动读取 (Kick) ---
    // 对应 BMI088 就是启动 DMA 传输
    if (current_driver->start_read) {
        current_driver->start_read();
    }

    // --- 步骤 3: 等待数据 (Wait) ---
    // 对应 BMI088 就是等待 RTOS 信号量。
    // 此时 Task 会挂起，CPU 去处理别的任务。
    if (current_driver->wait_data && current_driver->wait_data())
    {
        // 醒来后，说明数据到了

        // --- 步骤 4: 处理数据 (Process) ---
        // 调用驱动的解析函数。
        // 如果是 BMI088，EKF 就在这里面默默地跑完了。
        // 结果会被填入 g_ins_data 中。
        if (current_driver->process_data) {
            current_driver->process_data(&g_ins_data, dt);
        }

        // 更新状态为就绪
        g_ins_data.state = INS_STATE_READY;
    }
    else {
        // 如果 wait_data 返回 false，说明超时了（比如线断了）
        g_ins_data.state = INS_STATE_ERROR;
    }
}

// 获取数据的接口
// 语法：const Ins_Data_t* ...
// 含义：返回一个“只读”指针。外部只能看数据，不能改数据。
const Ins_data_t* Ins_get_data(void)
{
    return &g_ins_data;
}