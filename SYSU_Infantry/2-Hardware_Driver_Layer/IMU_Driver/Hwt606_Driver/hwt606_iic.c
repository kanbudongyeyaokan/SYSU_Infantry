/**
 * @file    hwt606_iic.c
 * @brief   HWT606 驱动实现 (极致精简版 + 三轴上电静止校准 + 动态温漂追踪 + 6轴Yaw防漂移)
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
static SemaphoreHandle_t hwt606_dma_sem = NULL;

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

// ================= 静态校准全局变量 =================
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint16_t cali_count = 0;
#define CALI_FRAMES 1000 // 假设运行在1000Hz，上电前1秒钟强制静止收集零偏

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

    if (hwt606_dma_sem == NULL) {
        hwt606_dma_sem = xSemaphoreCreateBinary();
    }

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
    
    // 初始化时重置校准状态与四元数，防止热重启时带入旧误差
    cali_count = 0;
    gyro_bias[0] = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.0f;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;

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

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hwt606_dma_sem != NULL) {
        xSemaphoreGiveFromISR(hwt606_dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
    }
}

static bool HWT606_Wait_Data(void)
{
   if (hwt606_dma_sem == NULL) return false;

    if (xSemaphoreTake(hwt606_dma_sem, pdMS_TO_TICKS(10)) == pdTRUE) {
        Watchdog_feed(imu_wdg);
        return true;
    } else {
        hwt606_dev.read_success = false;
        return false;
    }
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
    // 【静态校准层】：上电锁定绝对零偏
    // ==========================================================
    if (cali_count < CALI_FRAMES) {
        gyro_bias[0] += gx;
        gyro_bias[1] += gy;
        gyro_bias[2] += gz;
        cali_count++;
        
        if (cali_count == CALI_FRAMES) {
            gyro_bias[0] /= CALI_FRAMES;
            gyro_bias[1] /= CALI_FRAMES;
            gyro_bias[2] /= CALI_FRAMES;
        }
        
        // 校准期间挂起状态，阻止云台运动，防止失控
        out_data->state = INS_STATE_INIT;
        return; 
    }

    // 扣除开机时的绝对零偏
    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

    // ==========================================================
    // 【动态温漂追踪层 (ZUPT)】：专治 6轴 Yaw 温漂
    // ==========================================================
    static float dynamic_gz_bias = 0.0f;
    float acc_norm;
    arm_sqrt_f32(ax*ax + ay*ay + az*az, &acc_norm);

    // 严苛的静止检测条件：
    // 1. 加速度模长接近 1G (没受到撞击或加减速)
    // 2. X和Y轴陀螺仪极小 (云台没有在抬头或翻滚)
    // 3. Z轴有极其缓慢的漂移 (不超过 0.05 rad/s，即温漂范围内)
    if (acc_norm > 9.6f && acc_norm < 10.0f && 
        fabsf(gx) < 0.01f && fabsf(gy) < 0.01f && fabsf(gz) < 0.05f) {
        
        // 极低通滤波，偷偷把微小的温漂吃掉
        dynamic_gz_bias += 0.0005f * (gz - dynamic_gz_bias); 
    }
    
    // 再次扣除动态温漂 (专门给 Z 轴开小灶)
    gz -= dynamic_gz_bias;

    // ==========================================================
    // 【纯净反馈层】：用于喂给速度环 PID
    // ==========================================================
    float pure_gx = gx;
    float pure_gy = gy;
    float pure_gz = gz;

    // 动态抗干扰 Mahony 核心逻辑 
    float Kp = 0.4f;  
    float Ki = 0.005f; 

    // 防点头：检测到剧烈加速度干扰时，关闭重力修正
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
        
        // 【核心修复】：6轴 IMU 的重力绝对无法修正 Yaw 轴！
        // 必须强行把 ez 设为 0，防止加速度计的平移横向噪声污染 Yaw 轴陀螺仪！
        float ez = 0.0f; 

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
    float q_norm;
    arm_sqrt_f32(q0*q0 + q1*q1 + q2*q2 + q3*q3, &q_norm);
    q0 /= q_norm; q1 /= q_norm; q2 /= q_norm; q3 /= q_norm;

    // 计算标准欧拉角 (含防 NaN 限幅护盾)
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    
    float sinp = -2.0f * (q1*q3 - q0*q2);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp) * RAD2DEG;
    
    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;

    out_data->euler.roll  = pitch; 
    out_data->euler.pitch = roll; 
    out_data->euler.yaw   = yaw;

    out_data->acc_body.x  = ax * acc_norm; 
    out_data->acc_body.y  = ay * acc_norm;
    out_data->acc_body.z  = az * acc_norm;
    
    // 纯净角速度反馈
    out_data->gyro_body.x = pure_gx * RAD2DEG; 
    out_data->gyro_body.y = pure_gy * RAD2DEG;
    out_data->gyro_body.z = pure_gz * RAD2DEG;

    // 连续偏航角(Yaw)多圈处理
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