/**
 * @file    bmi088.c
 * @brief   BMI088 驱动
 * @note    抛弃 bsp_spi 封装，用于彻底解决初始化和读取失败的问题
 */
#include "bmi088.h"
#include "bmi088_reg_def.h"
#include "algorithm_ekf.h"
#include "bsp_dwt.h"
#include "main.h" 
#include "spi.h"  
#include <string.h>
#include <math.h>
#include "buzzer_alarm.h"
#include "bsp_wdg.h"

 // ================= 硬件引脚定义 (C板标准) =================
#define CS_ACC_GPIO_Port    GPIOA
#define CS_ACC_Pin          GPIO_PIN_4
#define CS_GYRO_GPIO_Port   GPIOB
#define CS_GYRO_Pin         GPIO_PIN_0

// ================= 私有变量 =================
static uint8_t acc_rx_buf[8];
static uint8_t gyro_rx_buf[8];
static Ekf_state_t ekf_state;
static Watchdog_device_t *imu_wdg;

// 外部 SPI 句柄
extern SPI_HandleTypeDef hspi1;

static void IMU_Offline_Callback(void *arg)
{
    Watchdog_buzzer_alarm("imu");
}

// ================= 内部底层函数 (直接 HAL 操作) =================

static void Bmi088_Write_Reg(GPIO_TypeDef *port, uint16_t pin, uint8_t addr, uint8_t data)
{
    // 拉低片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    // 发送地址 (bit7 = 0 表示写)
    uint8_t tx_addr = addr & BMI088_SPI_WRITE_CODE;
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100);
    // 发送数据
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    // 写入延时
    HAL_Delay(1);
    // 拉高片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}
static void Bmi088_Read_Reg(GPIO_TypeDef *port, uint16_t pin, uint8_t addr, uint8_t *data, uint8_t len)
{
    // 拉低片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    // 发送地址 (bit7 = 1 表示读)
    uint8_t tx_addr = addr | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100);
    // 读取 (如果是 Accel，第一个字节是 Dummy，需要在上层处理或者这里处理)
    // 为了简单，我们这里只负责透传读取，不做 Dummy 处理，让上层去偏移
    HAL_SPI_Receive(&hspi1, data, len, 100);
    // 拉高片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void Bmi088_Config_HardWare(void)
{
    // ---------------- Accel 配置 ----------------
    // 软复位
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开电源 (老代码顺序: 先开电源，再切模式)
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    HAL_Delay(10);
    // 切换 Active 模式
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    HAL_Delay(10);
    // 量程 3G
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 带宽 (老代码配置: 0x8C)
    Bmi088_Write_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_CONF_ADDR, 0x8C);
    // ---------------- Gyro 配置 ----------------
    // 软复位
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 切换 Normal 模式
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    HAL_Delay(10);
    // 量程 500dps
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 带宽
    Bmi088_Write_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

// ================= 接口实现 =================

static bool BMI088_Interface_Init(void)
{
    // 初始化 GPIO 
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);
    // 配置硬件
    Bmi088_Config_HardWare();
    HAL_Delay(50);
    // ID 校验
    uint8_t buf[2] = { 0 };
    // Accel ID 
    Bmi088_Read_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_CHIP_ID_ADDR, buf, 2);
    if (buf[1] != ACC_CHIP_ID_VAL) return false;
    // Gyro ID 
    Bmi088_Read_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_CHIP_ID_ADDR, buf, 1);
    if (buf[0] != GYRO_CHIP_ID_VAL) return false;
    // EKF 初始化
    Ekf_config_t ekf_conf = {
        .process_noise_q = 10.0f, .measurement_noise_r = 1000000.0f,
        .dt = 0.001f, .fading_factor = 0.9996f, .gyro_bias_noise = 0.001f,
        .enable_bias_correction = true
    };
    Ekf_init(&ekf_state, &ekf_conf);

    Watchdog_init_t wdg_config = {

    .owner_id = NULL,
    .reload_count = 30,
    .callback = IMU_Offline_Callback,
    .name = "imu"
    };
    imu_wdg = Watchdog_register(&wdg_config);

    return true;
}

static void BMI088_Interface_Start_Read(void)
{
    // Accel: 读 7 字节 (Dummy + 6 Data)
    Bmi088_Read_Reg(CS_ACC_GPIO_Port, CS_ACC_Pin, ACC_X_LSB_ADDR, acc_rx_buf, 7);
}

static bool BMI088_Interface_Wait_Data(void)
{
    // Gyro: 读 6 字节 
    Bmi088_Read_Reg(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GYRO_RATE_X_LSB_ADDR, gyro_rx_buf, 6);

    Watchdog_feed(imu_wdg);

    return true;
}

static void BMI088_Interface_Process(Ins_data_t *out_data, float dt_s)
{
    // --- Accel 解析 ---
    int16_t acc_int[3];
    acc_int[0] = (int16_t) ((acc_rx_buf[2] << 8) | acc_rx_buf[1]);
    acc_int[1] = (int16_t) ((acc_rx_buf[4] << 8) | acc_rx_buf[3]);
    acc_int[2] = (int16_t) ((acc_rx_buf[6] << 8) | acc_rx_buf[5]);

    // 单位转换 (3G -> m/s^2)
    const float ACC_K = BMI088_ACCEL_3G_SEN * 9.81f;
    float acc_mzs[3] = { acc_int[0] * ACC_K, acc_int[1] * ACC_K, acc_int[2] * ACC_K };

    // --- Gyro 解析 ---
    int16_t gyro_int[3];
    gyro_int[0] = (int16_t) ((gyro_rx_buf[1] << 8) | gyro_rx_buf[0]);
    gyro_int[1] = (int16_t) ((gyro_rx_buf[3] << 8) | gyro_rx_buf[2]);
    gyro_int[2] = (int16_t) ((gyro_rx_buf[5] << 8) | gyro_rx_buf[4]);

    // 单位转换 (500dps -> rad/s)
    const float GYRO_K_DEG = 1.0f / 65.536f;
    float gyro_rad[3] = {
        gyro_int[0] * GYRO_K_DEG * DEG2SEC,
        gyro_int[1] * GYRO_K_DEG * DEG2SEC,
        gyro_int[2] * GYRO_K_DEG * DEG2SEC
    };

    Ekf_update(&ekf_state, acc_mzs, gyro_rad, dt_s);

    out_data->acc_body.x = acc_mzs[0];
    out_data->acc_body.y = acc_mzs[1];
    out_data->acc_body.z = acc_mzs[2];
    out_data->gyro_body.x = gyro_int[0] * GYRO_K_DEG;
    out_data->gyro_body.y = gyro_int[1] * GYRO_K_DEG;
    out_data->gyro_body.z = gyro_int[2] * GYRO_K_DEG;
    out_data->euler.roll = ekf_state.euler.roll;
    out_data->euler.pitch = ekf_state.euler.pitch;
    out_data->euler.yaw = ekf_state.euler.yaw;
    out_data->total_yaw = ekf_state.yaw_total_angle;
    out_data->round_count = (int32_t) floorf((out_data->total_yaw + 180.0f) / 360.0f);
}

static const Ins_driver_interface_t bmi088_drv = {
    .init = BMI088_Interface_Init,
    .start_read = BMI088_Interface_Start_Read,
    .wait_data = BMI088_Interface_Wait_Data,
    .process_data = BMI088_Interface_Process
};

const Ins_driver_interface_t *BMI088_Get_Driver(void) { return &bmi088_drv; }
float BMI088_Get_Temp_Raw(void) { return 25.0f; }