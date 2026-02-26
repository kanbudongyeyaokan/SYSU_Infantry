/**
 * @file    hwt101_iic.c
 * @brief   HWT101 驱动
 */

#include "hwt101_iic.h"
#include <string.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"
 // ================= 配置 =================
#define HWT101_REG_GYRO_Z   0x39 
#define HWT101_REG_YAW      0x3F 
#define CALIB_SAMPLES       100     

static Watchdog_device_t *imu_wdg;

// ================= 结构体 =================
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;

    struct { int16_t gyro_z; int16_t yaw; } raw;

    bool is_ready;
    bool read_success;

    // --- 校准变量 ---
    bool is_calibrated;
    uint16_t calib_cnt;
    float calib_sum;
    float yaw_offset;
} HWT101_Driver_t;

static HWT101_Driver_t hwt_dev;

// ================= 接口实现 =================

static void IMU_Offline_Callback(void *arg)
{
    Watchdog_buzzer_alarm("imu");
}

static bool HWT101_Init(void)
{
    if (hwt_dev.hi2c == NULL) return false;

    // 初始化变量
    hwt_dev.is_calibrated = false;
    hwt_dev.calib_cnt = 0;
    hwt_dev.calib_sum = 0.0f;
    hwt_dev.yaw_offset = 0.0f;

    hwt_dev.is_ready = true;

    Watchdog_init_t wdg_config = {
    .owner_id = NULL,
    .reload_count = 30,
    .callback = IMU_Offline_Callback,
    .name = "imu"
    };
    imu_wdg = Watchdog_register(&wdg_config);

    return true;
}

static void HWT101_Start_Read(void)
{
    if (!hwt_dev.is_ready) return;

    // 硬件 I2C 读取，不加任何重试逻辑
    // 如果失败，read_success 直接为 false，这一帧丢弃即可
    bool ok = true;

    if (HAL_I2C_Mem_Read(hwt_dev.hi2c, hwt_dev.dev_addr, HWT101_REG_YAW,
        I2C_MEMADD_SIZE_8BIT, (uint8_t *) &hwt_dev.raw.yaw, 2, 2) != HAL_OK) ok = false;

    for (volatile int i = 0; i < 50; i++);

    if (HAL_I2C_Mem_Read(hwt_dev.hi2c, hwt_dev.dev_addr, HWT101_REG_GYRO_Z,
        I2C_MEMADD_SIZE_8BIT, (uint8_t *) &hwt_dev.raw.gyro_z, 2, 2) != HAL_OK) ok = false;

    hwt_dev.read_success = ok;
}

static bool HWT101_Wait_Data(void)
{
    if (hwt_dev.read_success)
    {
        Watchdog_feed(imu_wdg);
    }

    return hwt_dev.read_success;
}

static void HWT101_Process(Ins_data_t *out_data, float dt_s)
{
    if (!hwt_dev.read_success) return;

    const float K_ANGLE = 180.0f / 32768.0f;
    const float K_GYRO = 2000.0f / 32768.0f;

    float curr_yaw = hwt_dev.raw.yaw * K_ANGLE;

    // --- 非阻塞式校准逻辑 ---
    if (!hwt_dev.is_calibrated)
    {
        hwt_dev.calib_sum += curr_yaw;
        hwt_dev.calib_cnt++;

        // 采样达到 100 次 (约0.5s) 后锁定 Offset
        if (hwt_dev.calib_cnt >= CALIB_SAMPLES)
        {
            hwt_dev.yaw_offset = hwt_dev.calib_sum / CALIB_SAMPLES;
            hwt_dev.is_calibrated = true;
        }

        // 校准中，状态设为不可用，或者输出 0
        out_data->state = INS_STATE_CALIBRATING;
        out_data->euler.yaw = 0.0f;
        return;
    }

    // --- 正常输出 ---
    float final_yaw = curr_yaw - hwt_dev.yaw_offset;

    // 归一化 (-180 ~ 180)
    if (final_yaw > 180.0f)  final_yaw -= 360.0f;
    if (final_yaw < -180.0f) final_yaw += 360.0f;

    out_data->euler.yaw = final_yaw;
    out_data->total_yaw = out_data->euler.yaw;
    out_data->gyro_body.z = hwt_dev.raw.gyro_z * K_GYRO;

    // 其他轴清零
    out_data->euler.roll = 0; out_data->euler.pitch = 0;
    out_data->gyro_body.x = 0; out_data->gyro_body.y = 0;
    out_data->acc_body.x = 0; out_data->acc_body.y = 0; out_data->acc_body.z = 0;

    out_data->state = INS_STATE_READY;
}

// ================= Getter =================
static const Ins_driver_interface_t hwt101_drv = {
    .init = HWT101_Init,
    .start_read = HWT101_Start_Read,
    .wait_data = HWT101_Wait_Data,
    .process_data = HWT101_Process
};

const Ins_driver_interface_t *HWT101_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle)
{
    hwt_dev.hi2c = i2c_handle;
    hwt_dev.dev_addr = 0xA0;
    hwt_dev.is_ready = false;
    return &hwt101_drv;
}