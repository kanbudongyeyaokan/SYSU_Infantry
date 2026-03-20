/**
 * @file    hwt606_iic.c
 * @brief   HWT606 驱动实现 (内置Mahony 互补滤波，极致精简版)
 */

#include "hwt606_iic.h"
#include <string.h>
#include <math.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"
#include "error_handler.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "arm_math.h"

// ================= 配置 =================
// 维特智能寄存器 (Word寻址)：0x34起为 Ax, Ay, Az, Wx, Wy, Wz
#define REG_READ_START      0x34 
// 【优化】只读加速度和角速度，共 6 个 short 数据，合计 12 字节！通信耗时减半！
#define READ_LEN            12   

#define DEG2SEC             (3.14159265f / 180.0f)
#define RAD2DEG             (180.0f / 3.14159265f)

#define HWT_ACCEL_16G_SEN   (16.0f / 32768.0f)
#define HWT_GYRO_2000_SEN   (2000.0f / 32768.0f)

static Watchdog_device_t *imu_wdg;

// ================= 私有对象结构体 =================
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;

    bool is_ready;
    volatile bool read_success;

    // 连续 Yaw 计算记录
    float last_yaw;
    int32_t round_count;
    bool is_first_frame;
} HWT606_Driver_t;

static HWT606_Driver_t hwt606_dev;
static uint8_t hwt606_dma_buf[READ_LEN];

// Mahony 滤波全局状态
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

// ================= 内部辅助函数 =================
static void HWT606_Reset_I2C(void)
{
    HAL_I2C_DeInit(hwt606_dev.hi2c);
    HAL_I2C_Init(hwt606_dev.hi2c);
}

static void IMU_Offline_Callback(void *arg)
{
    ERROR_CRITICAL("HWT606", "IMU offline");
    Watchdog_buzzer_alarm("imu");
}

static bool HWT606_Init(void)
{
    if (hwt606_dev.hi2c == NULL) return false;

    HAL_Delay(200);

    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY)
    {
        HWT606_Reset_I2C();
    }

    uint8_t retry_count = 0;
    while (HAL_I2C_IsDeviceReady(hwt606_dev.hi2c, hwt606_dev.dev_addr, 3, 100) != HAL_OK)
    {
        retry_count++;
        if (retry_count > 5) return false;
        HAL_Delay(50); 
    }

    Watchdog_init_t wdg_config = {
        .owner_id = NULL, .reload_count = 30,
        .callback = IMU_Offline_Callback, .name = "HWT606"
    };
    imu_wdg = Watchdog_register(&wdg_config);

    hwt606_dev.is_first_frame = true;
    hwt606_dev.round_count = 0;
    hwt606_dev.is_ready = true;
    return true;
}

static void HWT606_Start_Read(void)
{
    if (!hwt606_dev.is_ready) return;
    hwt606_dev.read_success = false;
    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY) return;

    HAL_I2C_Mem_Read_DMA(hwt606_dev.hi2c, hwt606_dev.dev_addr, REG_READ_START,
        I2C_MEMADD_SIZE_8BIT, hwt606_dma_buf, READ_LEN);
}

void HWT606_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hwt606_dev.hi2c != NULL && hi2c == hwt606_dev.hi2c) {
        hwt606_dev.read_success = true;
    }
}

static bool HWT606_Wait_Data(void)
{
    uint32_t start_tick = HAL_GetTick();
    while (hwt606_dev.read_success == false) {
        if (HAL_GetTick() - start_tick > 10) return false;
        taskYIELD(); // 必须保留，防止轮询卡死系统
    }
    Watchdog_feed(imu_wdg);
    return true;
}

// 核心解算：解析数据 + 自适应 Mahony 滤波
static void HWT606_Process(Ins_data_t *out_data, float dt_s)
{
    if (!hwt606_dev.read_success) return;

    uint8_t *buf = hwt606_dma_buf;

    // 严格按照物理原轴读取
    int16_t acc_int[3], gyro_int[3];
    acc_int[0] = (int16_t)(buf[0] | (buf[1] << 8));
    acc_int[1] = (int16_t)(buf[2] | (buf[3] << 8));
    acc_int[2] = (int16_t)(buf[4] | (buf[5] << 8));

    gyro_int[0] = (int16_t)(buf[6] | (buf[7] << 8));
    gyro_int[1] = (int16_t)(buf[8] | (buf[9] << 8));
    gyro_int[2] = (int16_t)(buf[10] | (buf[11] << 8));

    // 单位换算
    float ax = acc_int[0] * HWT_ACCEL_16G_SEN * 9.80665f;
    float ay = acc_int[1] * HWT_ACCEL_16G_SEN * 9.80665f;
    float az = acc_int[2] * HWT_ACCEL_16G_SEN * 9.80665f;

    float gx = gyro_int[0] * HWT_GYRO_2000_SEN * DEG2SEC;
    float gy = gyro_int[1] * HWT_GYRO_2000_SEN * DEG2SEC;
    float gz = gyro_int[2] * HWT_GYRO_2000_SEN * DEG2SEC;

    // ==========================================================
    // 【核心修复】备份纯净数据！这是治好 YAW 轴急停摆动的神药
    // ==========================================================
    float pure_gx = gx;
    float pure_gy = gy;
    float pure_gz = gz;

    // 零漂处理
    static float gyro_z_bias = 0.0f; 
    float acc_norm;
    
    // 【DSP 优化 1】使用 FPU 硬件指令开方，代替 sqrtf
    arm_sqrt_f32(ax*ax + ay*ay + az*az, &acc_norm);

    // 静止检测
    if (acc_norm > 9.6f && acc_norm < 10.0f && 
        fabsf(gx) < 0.02f && fabsf(gy) < 0.02f && fabsf(gz) < 0.02f) {
        gyro_z_bias += 0.001f * (gz - gyro_z_bias); 
    }

    // 扣除学习到的零偏（纯净数据和将要用于融合的数据都要扣除）
    pure_gz -= gyro_z_bias;
    gz -= gyro_z_bias;

    // 动态抗干扰 Mahony 核心逻辑 
    float Kp = 0.4f;  // 【优化】降低解算的 Kp，减少加速度计高频噪声干扰，提升姿态平滑度
    float Ki = 0.005f;

    // 防点头
    if (acc_norm < 8.8f || acc_norm > 10.8f) {
        Kp = 0.0f; 
        Ki = 0.0f; 
    }

    if (acc_norm > 0.1f) {
        ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        float ex = (ay*vz - az*vy);
        float ey = (az*vx - ax*vz);
        float ez = (ax*vy - ay*vx);

        exInt += ex * Ki * dt_s;
        eyInt += ey * Ki * dt_s;
        ezInt += ez * Ki * dt_s;

        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    // 四元数更新 (一阶龙格库塔积分)
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last*gx - q2_last*gy - q3_last*gz) * (0.5f * dt_s);
    q1 += ( q0_last*gx + q2_last*gz - q3_last*gy) * (0.5f * dt_s);
    q2 += ( q0_last*gy - q1_last*gz + q3_last*gx) * (0.5f * dt_s);
    q3 += ( q0_last*gz + q1_last*gy - q2_last*gx) * (0.5f * dt_s);

    // 【DSP 优化 2】四元数归一化硬件加速
    float q_norm;
    arm_sqrt_f32(q0*q0 + q1*q1 + q2*q2 + q3*q3, &q_norm);
    q0 /= q_norm; q1 /= q_norm; q2 /= q_norm; q3 /= q_norm;

    // 计算标准欧拉角
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    float pitch = asinf(-2.0f*(q1*q3 - q0*q2)) * RAD2DEG;
    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;

    out_data->euler.roll  = pitch; 
    out_data->euler.pitch = roll; 
    out_data->euler.yaw   = yaw;

    out_data->acc_body.x  = ax * acc_norm; 
    out_data->acc_body.y  = ay * acc_norm;
    out_data->acc_body.z  = az * acc_norm;
    
    // ==========================================================
    // 【终极闭环】必须将纯净的数据喂给云台 PID 速度环！
    // ==========================================================
    out_data->gyro_body.x = pure_gx * RAD2DEG; 
    out_data->gyro_body.y = pure_gy * RAD2DEG;
    out_data->gyro_body.z = pure_gz * RAD2DEG;

    // 5. 连续偏航角(Yaw)多圈处理
    if (hwt606_dev.is_first_frame) {
        hwt606_dev.last_yaw = yaw;
        hwt606_dev.is_first_frame = false;
    }
    float yaw_diff = yaw - hwt606_dev.last_yaw;
    if (yaw_diff < -180.0f) hwt606_dev.round_count++;
    else if (yaw_diff > 180.0f) hwt606_dev.round_count--;
    hwt606_dev.last_yaw = yaw;

    out_data->round_count = hwt606_dev.round_count;
    out_data->total_yaw   = (hwt606_dev.round_count * 360.0f) + yaw;
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