/**
 * @file    hwt606_iic.c
 * @brief   HWT606 驱动实现 (内置Mahony 互补滤波)
 */

#include "hwt606_iic.h"
#include <string.h>
#include <math.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"
#include "error_handler.h"

// ================= 配置 =================
// 维特智能寄存器 (Word寻址)：0x34起为 Ax, Ay, Az, Wx, Wy, Wz
#define REG_READ_START      0x34 
#define READ_LEN            24   

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
    }
    Watchdog_feed(imu_wdg);
    return true;
}

// 核心解算：解析数据 + 自适应 Mahony 滤波
static void HWT606_Process(Ins_data_t *out_data, float dt_s)
{
    if (!hwt606_dev.read_success) return;

    uint8_t *buf = hwt606_dma_buf;

    // 严格按照物理原轴读取，绝不在原始数据处互换 X/Y，保证右手坐标系！
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

    //零漂处理
    static float gyro_z_bias = 0.0f; // 静态 Z 轴零偏记录
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);

    // 静止检测：加速度模长接近 1g，且三轴角速度极小
    if (acc_norm > 9.6f && acc_norm < 10.0f && 
        fabsf(gx) < 0.02f && fabsf(gy) < 0.02f && fabsf(gz) < 0.02f) {
        
        // 低通滤波在线学习 Z 轴当前的温漂误差 ，学习率为0.001，可以进行修改
        gyro_z_bias += 0.001f * (gz - gyro_z_bias); 
    }

    // 扣除学习到的零偏
    // gz -= gyro_z_bias;

    // 施加死区 (Deadband)：彻底滤除静止时的残余白噪声
    // 0.0015 rad/s 约等于 0.08 deg/s，如果角速度比这个还小，直接视为云台绝对静止
    if (fabsf(gz) < 0.0015f) {
        gz = 0.0f; 
    }

    // 动态抗干扰 Mahony 核心逻辑 
    // float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float Kp = 1.0f;  // 互补滤波比例增益
    float Ki = 0.005f;// 互补滤波积分增益 (用于消除陀螺仪静态零偏)

    // 防点头：当加速度偏离 1g (9.8) 超过 ±10% 时，说明在剧烈运动
    if (acc_norm < 8.8f || acc_norm > 10.8f) {
        Kp = 0.0f; // 彻底屏蔽加速度计，防止方向带偏
        Ki = 0.0f; // 停止积分更新
    }

    if (acc_norm > 0.1f) {
        ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

        // 从四元数推导出的机体系重力分量
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 叉乘求误差
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

    // 四元数归一化
    float q_norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= q_norm; q1 /= q_norm; q2 /= q_norm; q3 /= q_norm;

    // 计算标准欧拉角 (ZYX顺序)
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    float pitch = asinf(-2.0f*(q1*q3 - q0*q2)) * RAD2DEG;
    // float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;

    // ================= 新增：提取维特硬件 YAW 角度 =================
    // buf[18~19]是X角, buf[20~21]是Y角, buf[22~23]是Z角(YAW)
    int16_t hw_yaw_int = (int16_t)(buf[22] | (buf[23] << 8));
    float hw_yaw = hw_yaw_int * (180.0f / 32768.0f);

    out_data->euler.roll  = pitch;  // 若需对调，改为 pitch
    out_data->euler.pitch = roll; // 若需对调，改为 roll
    // out_data->euler.yaw   = yaw;
    out_data->euler.yaw   = hw_yaw; // 直接使用维特硬件输出的 YAW 角度，单位是度

    out_data->acc_body.x  = ax * acc_norm; // 恢复原始未归一化的m/s^2
    out_data->acc_body.y  = ay * acc_norm;
    out_data->acc_body.z  = az * acc_norm;
    out_data->gyro_body.x = gx * RAD2DEG;
    out_data->gyro_body.y = gy * RAD2DEG;
    out_data->gyro_body.z = gz * RAD2DEG;
    //ERROR_INFO("HWT606", "gyro_body_z: %.2f, gz: %.2f", out_data->gyro_body.y, gy * RAD2DEG);

    // 5. 连续偏航角(Yaw)多圈处理
    if (hwt606_dev.is_first_frame) {
        hwt606_dev.last_yaw = hw_yaw;
        hwt606_dev.is_first_frame = false;
    }
    float yaw_diff = hw_yaw - hwt606_dev.last_yaw;
    if (yaw_diff < -180.0f) hwt606_dev.round_count++;
    else if (yaw_diff > 180.0f) hwt606_dev.round_count--;
    hwt606_dev.last_yaw = hw_yaw;

    out_data->round_count = hwt606_dev.round_count;
    out_data->total_yaw   = (hwt606_dev.round_count * 360.0f) + hw_yaw;
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