#pragma once
#include "bmi088_reg_def.h"
#include "spi.h"
#include "gpio.h"
#include "algorithm_ekf.h"
#include "cmsis_os.h"

// 配置结构体
typedef struct {
    SPI_HandleTypeDef* spi_handle;
    GPIO_TypeDef* accel_cs_gpio_port;
    uint16_t accel_cs_gpio_pin;
    GPIO_TypeDef* gyro_cs_gpio_port;
    uint16_t gyro_cs_gpio_pin;
    bool enable_accel_self_test;  // 保留配置项
    bool enable_gyro_self_test;   // 保留配置项
} Bmi088_config_t;

// 定义原始数据结构体 (补充回来的部分)
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

// 数据结构体
typedef struct {
    Acc_raw_data_t acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test; // 运行时状态
} Acc_data_t;

typedef struct {
    Gyro_raw_data_t gyro_raw_data;
    bool enable_self_test; // 运行时状态
} Gyro_data_t;

typedef enum {
    NO_ERROR = 0,
    ACC_CHIP_ID_ERR = 0x01,
    ACC_DATA_ERR = 0x02,
    GYRO_CHIP_ID_ERR = 0x04,
    GYRO_DATA_ERR = 0x08,
} Bmi088_error_e;

typedef enum {
    IMU_STATE_INIT = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_READY,
    IMU_STATE_ERROR
} Imu_state_e;

typedef struct {
    Acc_data_t acc_data;
    Gyro_data_t gyro_data;      /*!< 陀螺仪数据 */
    Ekf_state_t ekf_state;      /*!< EKF计算用状态 */
    Bmi088_error_e bmi088_error;
    Imu_state_e state;          /*!< IMU/EKF是否就绪的状态 */
} Bmi088_data_t;

// 设备对象
typedef struct {
    Bmi088_config_t config;
    Bmi088_data_t data;
    Bmi088_error_e last_error;
    
    // 新增 DMA 缓冲区和同步信号量
    // ACC读取需要Dummy Byte，所以buffer长度需要+1
    // 假设最大连续读取长度不超过8字节
    uint8_t acc_rx_buf[8];  
    uint8_t gyro_rx_buf[8]; 
    osSemaphoreId spi_sem;  // 用于等待 DMA 完成
} Bmi088_device_t;

/**
 * @brief 初始化BMI088设备实例
 */
Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config);

// DMA 读取函数 (内部或任务调用)
void Bmi088_read_acc_dma(Bmi088_device_t* dev);
void Bmi088_read_gyro_dma(Bmi088_device_t* dev);
void Bmi088_read_temp(Bmi088_device_t* dev);

/**
 * @brief 更新EKF状态
 * @param dt 采样间隔(s)
 */
Bmi088_error_e Bmi088_ekf_update(Bmi088_device_t* bmi088, float dt);

// 获取结果接口
Quaternion_t* Bmi088_get_quaternion(Bmi088_device_t* bmi088);
Euler_angles_t* Bmi088_get_euler_angles(Bmi088_device_t* bmi088);

// 兼容旧接口的数据获取函数 (实际上现在数据已经存在结构体里了，直接返回指针)
Acc_raw_data_t* Read_acc_data(Bmi088_device_t* bmi088);
Gyro_raw_data_t* Read_gyro_data(Bmi088_device_t* bmi088);

// DMA中断回调接口，需要在HAL_SPI_RxCpltCallback中调用
void Bmi088_DMA_RxCpltCallback(Bmi088_device_t* dev);