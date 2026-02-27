/**
 * @file    hwt606_iic.c
 * @brief   HWT606 驱动实现 (18字节连续读取 + DMA + 轴向修正 + 三轴自动零漂校准)
 */

#include "hwt606_iic.h"
#include <string.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"
 // ================= 配置 =================
 // 从角速度(0x37)开始，一次读到角度(0x3F)
 // 顺序: Gyro(6) + Mag(6/跳过) + Angle(6)
#define REG_READ_START      0x37 
#define READ_LEN            18   
#define CALIB_SAMPLES       100     // 采样100次取平均 (约0.5s)


#define NORMALIZE_ANGLE(theta) \
    if (theta > 180.0f) theta -= 360.0f; \
    else if (theta < -180.0f) theta += 360.0f;

static Watchdog_device_t *imu_wdg;

// ================= 私有对象结构体 =================
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;

    // 原始数据缓存 (经过解析后的 int16)
    struct
    {
        int16_t gyro[3];  // X, Y, Z
        int16_t angle[3]; // Roll, Pitch, Yaw
    } raw;

    bool is_ready;

    // 标志位：volatile 防止编译器优化中断变量
    volatile bool read_success;

    // --- 校准相关变量 ---
    bool is_calibrated;         // 是否完成校准
    uint16_t calib_cnt;         // 当前采样次数

    float sum_roll, sum_pitch, sum_yaw;

    // 计算出的零漂值
    float offset_roll, offset_pitch, offset_yaw;
    //多圈 Yaw 计算所需的历史状态记录 
    float last_yaw;
} HWT606_Driver_t;

static HWT606_Driver_t hwt606_dev;

// DMA 专用全局缓冲区 
static uint8_t hwt606_dma_buf[READ_LEN];

// ================= 内部辅助函数 =================

static void HWT606_Reset_I2C(void)
{
    HAL_I2C_DeInit(hwt606_dev.hi2c);
    HAL_I2C_Init(hwt606_dev.hi2c);
}

// ================= 接口实现 =================

static void IMU_Offline_Callback(void *arg)
{
    Watchdog_buzzer_alarm("imu");
}


static bool HWT606_Init(void)
{
    if (hwt606_dev.hi2c == NULL) return false;

    //延时等待初始化成功
    HAL_Delay(200);
    // --- 初始化校准变量 ---
    hwt606_dev.is_calibrated = false;
    hwt606_dev.calib_cnt = 0;
    hwt606_dev.sum_roll = 0.0f;
    hwt606_dev.sum_pitch = 0.0f;
    hwt606_dev.sum_yaw = 0.0f;
    hwt606_dev.offset_roll = 0.0f;
    hwt606_dev.offset_pitch = 0.0f;
    hwt606_dev.offset_yaw = 0.0f;
    hwt606_dev.last_yaw = 0.0f;
    // 复位总线防死锁
    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY)
    {
        HWT606_Reset_I2C();
    }

    //加入重试机制
    uint8_t retry_count = 0;
    while (HAL_I2C_IsDeviceReady(hwt606_dev.hi2c, hwt606_dev.dev_addr, 3, 100) != HAL_OK)
    {
        retry_count++;
        if (retry_count > 5) {
            // 重试了 5 次 (约 250ms) 还是不行，说明真坏了或线掉了
            return false;
        }
        HAL_Delay(50); // 每次失败等 50ms 再试
    }
    // 检查设备在线 
    // if (HAL_I2C_IsDeviceReady(hwt606_dev.hi2c, hwt606_dev.dev_addr, 3, 100) != HAL_OK)
    // {
    //     return false;
    // }

    Watchdog_init_t wdg_config = {
    .owner_id = NULL,
    .reload_count = 30,
    .callback = IMU_Offline_Callback,
    .name = "imu"
    };
    imu_wdg = Watchdog_register(&wdg_config);


    hwt606_dev.is_ready = true;
    return true;
}

// 启动 DMA 读取
static void HWT606_Start_Read(void)
{
    if (!hwt606_dev.is_ready) return;

    // 清除成功标志
    hwt606_dev.read_success = false;

    // 检查 I2C 状态，如果正忙则跳过本次读取
    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY)
    {
        return;
    }

    // 启动 DMA 传输 -> 数据直接存入全局数组 hwt606_dma_buf
    if (HAL_I2C_Mem_Read_DMA(hwt606_dev.hi2c, hwt606_dev.dev_addr, REG_READ_START,
        I2C_MEMADD_SIZE_8BIT, hwt606_dma_buf, READ_LEN) != HAL_OK)
    {
        // 启动失败
    }
}

// 接收完成回调 (被 bsp_iic.c 调用)
void HWT606_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hwt606_dev.hi2c != NULL && hi2c == hwt606_dev.hi2c)
    {
        hwt606_dev.read_success = true;
    }
}

// 等待数据 (检查标志位)
static bool HWT606_Wait_Data(void)
{
    uint32_t start_tick = HAL_GetTick();
    while (hwt606_dev.read_success == false)
    {
        // 超时退出 (1ms)
        if (HAL_GetTick() - start_tick > 1)
        {
            return false;
        }
    }
    Watchdog_feed(imu_wdg);

    return true;
}

// 解析数据 + 校准
static void HWT606_Process(Ins_data_t *out_data, float dt_s)
{
    if (!hwt606_dev.read_success) return;

    // 引用全局 DMA 缓冲区
    uint8_t *buf = hwt606_dma_buf;

    // --- 解析原始数据 (保持你原有的轴向映射) ---

    // 解析角速度 (前6字节)
    hwt606_dev.raw.gyro[1] = (int16_t) (buf[0] | (buf[1] << 8)); // 原 X -> 给 Y
    hwt606_dev.raw.gyro[0] = (int16_t) (buf[2] | (buf[3] << 8)); // 原 Y -> 给 X
    hwt606_dev.raw.gyro[2] = (int16_t) (buf[4] | (buf[5] << 8)); // Z 不变

    // 解析角度 (后6字节)
    hwt606_dev.raw.angle[1] = (int16_t) (buf[12] | (buf[13] << 8)); // 原 Roll -> 给 Pitch
    hwt606_dev.raw.angle[0] = (int16_t) (buf[14] | (buf[15] << 8)); // 原 Pitch -> 给 Roll
    hwt606_dev.raw.angle[2] = (int16_t) (buf[16] | (buf[17] << 8)); // Yaw 不变

    // 转换系数
    const float K_ANGLE = 180.0f / 32768.0f;  // 角度 (deg)
    const float K_GYRO = 2000.0f / 32768.0f; // 角速度 (deg/s)

    // 计算当前的原始物理角度 
    // angle[0] 对应 out_data->roll
    // angle[1] 对应 out_data->pitch
    // angle[2] 对应 out_data->yaw
    float curr_roll = hwt606_dev.raw.angle[0] * K_ANGLE;
    float curr_pitch = hwt606_dev.raw.angle[1] * K_ANGLE;
    float curr_yaw = hwt606_dev.raw.angle[2] * K_ANGLE;

    // --- 自动零漂校准---
    if (!hwt606_dev.is_calibrated)
    {
        hwt606_dev.sum_roll += curr_roll;
        hwt606_dev.sum_pitch += curr_pitch;
        hwt606_dev.sum_yaw += curr_yaw;
        hwt606_dev.calib_cnt++;

        // 采样达到 100 次后锁定 Offset
        if (hwt606_dev.calib_cnt >= CALIB_SAMPLES)
        {
            hwt606_dev.offset_roll = hwt606_dev.sum_roll / CALIB_SAMPLES;
            hwt606_dev.offset_pitch = hwt606_dev.sum_pitch / CALIB_SAMPLES;
            hwt606_dev.offset_yaw = hwt606_dev.sum_yaw / CALIB_SAMPLES;
            hwt606_dev.is_calibrated = true;
        }

        // 校准期间，状态设为 0 (未就绪)，数据输出 0
        out_data->state = 0;
        out_data->euler.roll = 0.0f;
        out_data->euler.pitch = 0.0f;
        out_data->euler.yaw = 0.0f;
        return;
    }

    // --- 正常输出---
    
    float final_roll  = curr_roll - hwt606_dev.offset_roll;
    // float final_pitch = curr_pitch - hwt606_dev.offset_pitch;
    float final_pitch = curr_pitch; // Pitch 轴不做零漂校准，保持原始输出
    float final_yaw   = curr_yaw - hwt606_dev.offset_yaw;

    NORMALIZE_ANGLE(final_roll);
    NORMALIZE_ANGLE(final_pitch);
    NORMALIZE_ANGLE(final_yaw);

// 如果是校准完成后的“第一帧”正常数据，先初始化 last_yaw，防止起步误判跳变
    if (hwt606_dev.calib_cnt == CALIB_SAMPLES) {
        hwt606_dev.last_yaw = final_yaw;
        hwt606_dev.calib_cnt++;    // 计数器加1，以后就不会再进这个初始化分支了
        out_data->round_count = 0; // 圈数清零
    }

    // 计算当前帧与上一帧的差值
    float yaw_diff = final_yaw - hwt606_dev.last_yaw;

    // 过零检测 (假设正转是角度增加)
    if (yaw_diff < -180.0f) {
        // 从 179 度跳变到 -179 度，差值为负大数，说明正向转过了一圈
        out_data->round_count++;
    } 
    else if (yaw_diff > 180.0f) {
        // 从 -179 度跳变到 179 度，差值为正大数，说明反向转过了一圈
        out_data->round_count--;
    }

    // 记录本次的 Yaw 供下一帧对比
    hwt606_dev.last_yaw = final_yaw;

    // 填充角度
    out_data->euler.roll = final_roll;
    out_data->euler.pitch = final_pitch;
    out_data->euler.yaw = final_yaw;

    // 填充角速度
    out_data->gyro_body.x = hwt606_dev.raw.gyro[0] * K_GYRO;
    out_data->gyro_body.y = hwt606_dev.raw.gyro[1] * K_GYRO;
    out_data->gyro_body.z = hwt606_dev.raw.gyro[2] * K_GYRO;

    // 清零加速度 
    out_data->acc_body.x = 0;
    out_data->acc_body.y = 0;
    out_data->acc_body.z = 0;

    out_data->total_yaw = (out_data->round_count * 360.0f) + final_yaw;
    out_data->state = INS_STATE_READY;
}

// ================= Getter =================
static const Ins_driver_interface_t hwt606_drv = {
    .init = HWT606_Init,
    .start_read = HWT606_Start_Read,
    .wait_data = HWT606_Wait_Data,
    .process_data = HWT606_Process
};

const Ins_driver_interface_t *HWT606_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle)
{
    hwt606_dev.hi2c = i2c_handle;
    hwt606_dev.dev_addr = HWT606_IIC_ADDR;
    hwt606_dev.is_ready = false;
    return &hwt606_drv;
}