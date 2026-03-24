/**
 * @file    bmi088.c
 * @brief   BMI088 驱动 (基于 EKF 最优估计)
 */
#include "bmi088.h"
#include "bmi088_reg_def.h"
#include "algorithm_ekf.h"  // 替换为 EKF 头文件
#include "bsp_dwt.h"
#include "main.h" 
#include "spi.h"  
#include <string.h>
#include <math.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"
#include "bmi088_temp.h"

// #include "arm_math.h"

#define DEG2SEC             (3.14159265f / 180.0f)
#define RAD2DEG             (180.0f / 3.14159265f)

// ================= 硬件引脚定义 =================
#define CS_ACC_GPIO_Port    GPIOA
#define CS_ACC_Pin          GPIO_PIN_4
#define CS_GYRO_GPIO_Port   GPIOB
#define CS_GYRO_Pin         GPIO_PIN_0

// ================= 原始数据变量 =================
static uint8_t acc_rx_buf[8];
static uint8_t gyro_rx_buf[8];
static uint8_t temp_rx_buf[3];  

static Watchdog_device_t *imu_wdg;
extern SPI_HandleTypeDef hspi1;

// ================= 温度与滤波器状态 =================
static float current_temp = 25.0f; 
static bool is_temp_ready = false; // 热力学稳态标志
static bool is_ekf_init = false;   // 滤波器初始化标志

// 实例化 EKF 结构体与统计协方差参数
static Ekf_state_t imu_ekf;
static Ekf_config_t imu_ekf_cfg = {
    .process_noise_q = 10.0f,          // 状态转移过程噪声方差
    .measurement_noise_r = 50000000.0f, // 观测噪声方差
    .gyro_bias_noise = 0.001f,         // 零偏游走噪声方差
    .fading_factor = 0.9996f           // 渐减因子
};

// ================= 初始基线估计变量 =================
static float gyro_bias_startup[3] = {0.0f, 0.0f, 0.0f};
static uint16_t cali_count = 0;
#define CALI_FRAMES 1000 // 收集 1000 个样本计算初始均值

static void IMU_Offline_Callback(void *arg)
{
    Watchdog_buzzer_alarm("imu");
}

// ... [Bmi088_Write_Reg, Bmi088_Read_Reg, Bmi088_Config_HardWare 保持不变] ...

static void Bmi088_Write_Reg(GPIO_TypeDef *port, uint16_t pin, uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t tx_addr = addr & BMI088_SPI_WRITE_CODE;
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100);
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    HAL_Delay(1);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void Bmi088_Read_Reg(GPIO_TypeDef *port, uint16_t pin, uint8_t addr, uint8_t *data, uint8_t len) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t tx_addr = addr | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void Bmi088_Config_HardWare(void) {
    // ---------------- Accel 配置 ----------------
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    HAL_Delay(10);
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    HAL_Delay(10);
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_RANGE_ADDR, ACC_RANGE_6G);
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_CONF_ADDR, 0x8C);
    // ---------------- Gyro 配置 ----------------
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    HAL_Delay(10);
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_BANDWIDTH_ADDR, GYRO_ODR_1000Hz_BANDWIDTH_116Hz);
}

static bool BMI088_Interface_Init(void)
{
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);

    Bmi088_temp_init();
    Bmi088_Config_HardWare();
    HAL_Delay(50);

    uint8_t buf[2] = { 0 };
    Bmi088_Read_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_CHIP_ID_ADDR, buf, 2);
    if (buf[1] != ACC_CHIP_ID_VAL) return false;
    Bmi088_Read_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_CHIP_ID_ADDR, buf, 1);
    if (buf[0] != GYRO_CHIP_ID_VAL) return false;

    // 状态重置
    is_temp_ready = false;
    is_ekf_init = false;
    cali_count = 0;
    memset(gyro_bias_startup, 0, sizeof(gyro_bias_startup));

    Watchdog_init_t wdg_config = {
        .owner_id = NULL, .reload_count = 30,
        .callback = IMU_Offline_Callback, .name = "imu"
    };
    imu_wdg = Watchdog_register(&wdg_config);

    return true;
}

static void BMI088_Interface_Start_Read(void)
{
    Bmi088_Read_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_X_LSB_ADDR, acc_rx_buf, 7);
}

static bool BMI088_Interface_Wait_Data(void)
{
    Bmi088_Read_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_RATE_X_LSB_ADDR, gyro_rx_buf, 6);
    Bmi088_Read_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, TEMP_MSB_ADDR, temp_rx_buf, 3);
    Watchdog_feed(imu_wdg);
    return true;
}

static void BMI088_Interface_Process(Ins_data_t *out_data, float dt_s)
{
    // --- 1. 数据解析 ---
    int16_t acc_int[3], gyro_int[3];
    acc_int[0] = (int16_t) ((acc_rx_buf[2] << 8) | acc_rx_buf[1]);
    acc_int[1] = (int16_t) ((acc_rx_buf[4] << 8) | acc_rx_buf[3]);
    acc_int[2] = (int16_t) ((acc_rx_buf[6] << 8) | acc_rx_buf[5]);

    gyro_int[0] = (int16_t) ((gyro_rx_buf[1] << 8) | gyro_rx_buf[0]);
    gyro_int[1] = (int16_t) ((gyro_rx_buf[3] << 8) | gyro_rx_buf[2]);
    gyro_int[2] = (int16_t) ((gyro_rx_buf[5] << 8) | gyro_rx_buf[4]);

    const float ACC_K = BMI088_ACCEL_3G_SEN * 9.80665f*2.0;
    const float GYRO_K_RAD = (1.0f / 16.384f) * DEG2SEC; 

    float acc_mzs[3] = { acc_int[0] * ACC_K, acc_int[1] * ACC_K, acc_int[2] * ACC_K };
    float gyro_rad[3] = { gyro_int[0] * GYRO_K_RAD, gyro_int[1] * GYRO_K_RAD, gyro_int[2] * GYRO_K_RAD };

    // --- 2. 提取物理温度并控制 ---
    uint16_t temp_uint11 = (temp_rx_buf[1] << 3) | (temp_rx_buf[2] >> 5);
    int16_t temp_int11 = (temp_uint11 > 1023) ? ((int16_t)temp_uint11 - 2048) : (int16_t)temp_uint11;
    current_temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
    Bmi088_temp_control(current_temp);

    // --- 3. 稳态门控 (Temperature Gating) ---
    // 在信号统计特性平稳前，拒绝进行估计
    // if (!is_temp_ready) {
    //     if (current_temp >= 36.5f) {
    //         is_temp_ready = true;
    //     } else {
    //         out_data->temp = current_temp;
    //         out_data->state = INS_STATE_INIT;
    //         return; 
    //     }
    // }

    // --- 4. 滤波器状态初始化 ---
    if (!is_ekf_init) {
        Ekf_init(&imu_ekf, &imu_ekf_cfg);
        is_ekf_init = true;
    }

    // --- 5. 初始基线均值估计 ---
    // 利用样本均值（Sample Mean）为 EKF 的零偏状态向量提供良好的先验
    if (cali_count < CALI_FRAMES) {
        gyro_bias_startup[0] += gyro_rad[0];
        gyro_bias_startup[1] += gyro_rad[1];
        gyro_bias_startup[2] += gyro_rad[2];
        cali_count++;
        
        if (cali_count == CALI_FRAMES) {
            imu_ekf.gyro_bias[0] = gyro_bias_startup[0] / CALI_FRAMES;
            imu_ekf.gyro_bias[1] = gyro_bias_startup[1] / CALI_FRAMES;
            imu_ekf.gyro_bias[2] = gyro_bias_startup[2] / CALI_FRAMES;
        }
        
        out_data->temp = current_temp;
        out_data->state = INS_STATE_INIT;
        return; 
    }

    // --- 6. EKF 最优估计更新 ---
    Ekf_update(&imu_ekf, acc_mzs, gyro_rad, dt_s);

    // --- 7. 数据映射 ---
    out_data->acc_body.x = acc_mzs[0];
    out_data->acc_body.y = acc_mzs[1];
    out_data->acc_body.z = acc_mzs[2];
    
    // 输出无偏估计后的净角速度
    out_data->gyro_body.x = (gyro_rad[0] - imu_ekf.gyro_bias[0]) * RAD2DEG;
    out_data->gyro_body.y = (gyro_rad[1] - imu_ekf.gyro_bias[1]) * RAD2DEG;
    out_data->gyro_body.z = (gyro_rad[2] - imu_ekf.gyro_bias[2]) * RAD2DEG;
    
    out_data->euler.pitch = imu_ekf.euler.pitch; 
    out_data->euler.roll  = imu_ekf.euler.roll;
    out_data->euler.yaw   = imu_ekf.euler.yaw;
    
    out_data->total_yaw = imu_ekf.yaw_total_angle;
    out_data->round_count = imu_ekf.yaw_round_count;
    out_data->temp = current_temp;
    out_data->state = INS_STATE_READY;
}

static const Ins_driver_interface_t bmi088_drv = {
    .init = BMI088_Interface_Init,
    .start_read = BMI088_Interface_Start_Read,
    .wait_data = BMI088_Interface_Wait_Data,
    .process_data = BMI088_Interface_Process
};

const Ins_driver_interface_t *BMI088_Get_Driver(void) { return &bmi088_drv; }
float BMI088_Get_Temp_Raw(void) { return current_temp; }