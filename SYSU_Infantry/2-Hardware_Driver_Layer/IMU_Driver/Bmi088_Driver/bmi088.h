#pragma once
#include "bmi088_reg_def.h"
#include "spi.h"
#include "gpio.h"
#include "cmsis_os.h"
#include <stdbool.h>

// --- 数据结构定义 ---

// 欧拉角结构体 (单位: 度)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_angles_t;

// 原始数据结构体 (单位: m/s^2 和 rad/s)
typedef struct {
    float x;
    float y;
    float z;
} Acc_raw_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Gyro_raw_data_t;

typedef struct {
    Acc_raw_data_t acc_raw_data;
    float temperature;
} Acc_data_t;

typedef struct {
    Gyro_raw_data_t gyro_raw_data;
} Gyro_data_t;

typedef enum {
    NO_ERROR = 0,
    BMI088_INIT_ERROR = 0x01,
} Bmi088_error_e;

// BMI088 设备对象
typedef struct {
    // 硬件配置
    struct {
        SPI_HandleTypeDef* spi_handle;
        GPIO_TypeDef* accel_cs_gpio_port;
        uint16_t accel_cs_gpio_pin;
        GPIO_TypeDef* gyro_cs_gpio_port;
        uint16_t gyro_cs_gpio_pin;
    } config;

    // 数据存储
    struct {
        Acc_data_t acc_data;
        Gyro_data_t gyro_data;
    } data;

    Bmi088_error_e last_error;

    // DMA 缓冲区 (私有)
    uint8_t acc_rx_buf[8];
    uint8_t gyro_rx_buf[8];
    osSemaphoreId spi_sem;
} Bmi088_device_t;

// --- 配置结构体 (用于初始化传参) ---
typedef struct {
    SPI_HandleTypeDef* spi_handle;
    GPIO_TypeDef* accel_cs_gpio_port;
    uint16_t accel_cs_gpio_pin;
    GPIO_TypeDef* gyro_cs_gpio_port;
    uint16_t gyro_cs_gpio_pin;
} Bmi088_config_t;

// --- 函数声明 ---
Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config);
void Bmi088_read_acc_dma(Bmi088_device_t* dev);
void Bmi088_read_gyro_dma(Bmi088_device_t* dev);
void Bmi088_read_temp(Bmi088_device_t* dev);
void Bmi088_read_gyro_blocking(Bmi088_device_t* dev); // 用于初始校准
void Bmi088_DMA_RxCpltCallback(Bmi088_device_t* dev); // 放在 HAL_SPI_RxCpltCallback 中