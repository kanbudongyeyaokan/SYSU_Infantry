#ifndef SYSU_INFANTRY_INS_H
#define SYSU_INFANTRY_INS_H

#include <stdint.h>
#include <stdbool.h>

// ================= 标准物理量定义 =================

// 用法：统一三维向量的格式。
typedef struct {
    float x;
    float y;
    float z;
} Ins_vector3_t;

// 统一欧拉角格式
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Ins_euler_t;

// 定义状态枚举
typedef enum {
    INS_STATE_INIT = 0,     // 初始化中
    INS_STATE_READY,        // 数据正常，可以使用
    INS_STATE_ERROR,        // 传感器故障或通信超时
    INS_STATE_CALIBRATING, // 自动校准中 (如陀螺零偏校准)
} Ins_state_e;

/**
 * @brief 核心 INS 数据包 (这是给上层控制代码看的“最终结果”)
 */
typedef struct {
    Ins_vector3_t acc_body;    // 机体系加速度 (单位必须统一: m/s^2)
    Ins_vector3_t gyro_body;   // 机体系角速度 (单位必须统一: deg/s 或 rad/s，建议 deg/s)
    Ins_euler_t   euler;       // 欧拉角 (单位: 度)

    float total_yaw;           // 累计 Yaw 角度 (用于过零处理，比如转了 720度)
    int32_t round_count;       // 圈数记录
    float temp;                // 温度 (用于温控监测)
    float dt_s;                // 两次更新的时间间隔 (秒)，对于积分和微分控制很重要

    Ins_state_e state;         // 当前 INS 模块的状态
} Ins_data_t;

// ================= 硬件驱动抽象接口 ================

//使用一个函数指针表来管理接口
typedef struct {
    // INS层调用它来初始化底层硬件。
    bool (*init)(void);

    // 启动读取 (Kick)
    // 对应 BMI088 的 start_dma，或 HWT606 的请求数据
    void (*start_read)(void);

    // 等待数据 (Wait)
    // 如果返回 false，说明超时了
    bool (*wait_data)(void);

    // 数据处理 (Process)
    // 含义：底层驱动要把自己的原始数据，算好填入 out_data 里。
    void (*process_data)(Ins_data_t *out_data, float dt_s);

} Ins_driver_interface_t;


// ================= INS 层 API =================

/**
 * @brief 注册底层驱动
 * @param driver_impl 这是一个指针，指向具体的驱动实现
 */
void Ins_init(const Ins_driver_interface_t *driver_impl);

/**
 * @brief 更新 INS 数据
 * @note  Task 层只需要死循环调用这个函数，里面的 Kick->Wait->Process 流程自动完成
 */
void Ins_update(void);

/**
 * @brief 获取 INS 数据指针 (只读)
 * @note  Task 层或者控制层通过这个函数拿到最终的数据
 */
const Ins_data_t* Ins_get_data(void);

#endif // SYSU_INFANTRY_INS_H