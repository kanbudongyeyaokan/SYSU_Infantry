/**
 * @Author         : SYSU电控组
 * @Date           : 2025-12-1
 * @Note           : C板BMI088驱动库
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088_reg_def.h"
#include "spi.h"
#include "gpio.h"
#include "algorithm_ekf.h"  // 包含EKF模块头文件

/**
 * @brief BMI088配置结构体
 */
typedef struct {
    SPI_HandleTypeDef* spi_handle;        /*!< SPI句柄 */
    GPIO_TypeDef* accel_cs_gpio_port;     /*!< 加速度计片选GPIO端口 */
    uint16_t accel_cs_gpio_pin;           /*!< 加速度计片选GPIO引脚 */
    GPIO_TypeDef* gyro_cs_gpio_port;      /*!< 陀螺仪片选GPIO端口 */
    uint16_t gyro_cs_gpio_pin;            /*!< 陀螺仪片选GPIO引脚 */
    bool enable_accel_self_test;          /*!< 启用加速度计自检 */
    bool enable_gyro_self_test;           /*!< 启用陀螺仪自检 */
    float accel_rotation[3][3];           /*!< 加速度计->机体系旋转矩阵 */
    float gyro_rotation[3][3];            /*!< 陀螺仪->机体系旋转矩阵 */
} Bmi088_config_t;

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

// EKF相关结构体已移至ekf.h中

typedef struct {
    Acc_raw_data_t acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test;
} Acc_data_t;

typedef struct {
    Gyro_raw_data_t gyro_raw_data;
    bool enable_self_test;
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

/**
 * @brief BMI088设备结构体
 */
typedef struct {
    Bmi088_config_t config;        /*!< BMI088配置 */
    Bmi088_data_t data;            /*!< BMI088数据 */
    Bmi088_error_e last_error;     /*!< 上次错误代码 */
} Bmi088_device_t;

/**
 * @brief BMI088设备初始化
 * @param config BMI088配置结构体指针
 * @return BMI088设备结构体指针
 */
Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config);

// 基础函数 (内部使用)
static void Write_data_to_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t data);
static void Write_data_to_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t data);
static void Read_single_data_from_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t *data);
static void Read_single_data_from_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t *data);
static void Read_multi_data_from_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t len, uint8_t *data);
static void Read_multi_data_from_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t len, uint8_t *data);

// 初始化函数 (内部使用)
static Bmi088_error_e Bmi088_init(Bmi088_device_t* bmi088);
static void Bmi088_conf_init(Bmi088_device_t* bmi088);

// 功能函数 (外部使用)
/**
 * @brief 读取加速度计数据
 * @param bmi088 BMI088设备结构体指针
 * @return 加速度计数据指针，指向设备内部存储的数据，不需要释放
 * @note 返回值指向设备内部存储，每次调用都会更新，用户无需释放内存
 */
Acc_raw_data_t* Read_acc_data(Bmi088_device_t* bmi088);

/**
 * @brief 读取陀螺仪数据
 * @param bmi088 BMI088设备结构体指针
 * @return 陀螺仪数据指针，指向设备内部存储的数据，不需要释放
 * @note 返回值指向设备内部存储，每次调用都会更新，用户无需释放内存
 */
Gyro_raw_data_t* Read_gyro_data(Bmi088_device_t* bmi088);

/**
 * @brief 读取加速度计传感器时间
 * @param bmi088 BMI088设备结构体指针
 * @return 传感器时间指针，指向设备内部存储的数据，不需要释放
 * @note 返回值指向设备内部存储，每次调用都会更新，用户无需释放内存
 */
float* Read_acc_sensor_time(Bmi088_device_t* bmi088);

/**
 * @brief 读取加速度计温度数据
 * @param bmi088 BMI088设备结构体指针
 * @return 温度数据指针，指向设备内部存储的数据，不需要释放
 * @note 返回值指向设备内部存储，每次调用都会更新，用户无需释放内存
 */
float* Read_acc_temperature(Bmi088_device_t* bmi088);

/**
 * @brief 获取BMI088当前温度（用于EKF温度补偿）
 * @param bmi088 BMI088设备结构体指针
 * @return 当前温度值（摄氏度），如果设备无效则返回25.0f
 * @note 这是对Read_acc_temperature的简单封装，方便INS任务调用
 */
float Bmi088_get_temperature(Bmi088_device_t* bmi088);

// EKF相关函数

/**********************************************************************************************************
 * @attention
 * 
 * IMU坐标系与机体坐标系转换:
 * 为了适应不同的IMU安装方向，从传感器读取的原始数据需要先转换到机体坐标系(Body Frame)，
 * 然后再送入EKF进行姿态解算。
 * 
 * 机体坐标系 (Body Frame) 定义:
 *   - X轴: 指向机器人前方
 *   - Y轴: 指向机器人左侧
 *   - Z轴: 垂直向上
 * 
 * 坐标变换在 `Bmi088_ekf_update` 函数中实现。
 * 请根据IMU模块在您机器人上的实际安装方向，修改该函数中的坐标映射关系。
 * 
 ***********************************************************************************************************/

/**
 * @brief 初始化EKF参数和状态
 * @param bmi088 BMI088设备结构体指针
 * @param ekf_config EKF配置参数指针
 * @return 错误代码
 */
Bmi088_error_e Bmi088_ekf_init(Bmi088_device_t* bmi088, Ekf_config_t* ekf_config);

/**
 * @brief 使用新的传感器数据更新EKF
 * @param bmi088 BMI088设备结构体指针
 * @return 错误代码
 */
Bmi088_error_e Bmi088_ekf_update(Bmi088_device_t* bmi088);

/**
 * @brief 获取当前姿态四元数
 * @param bmi088 BMI088设备结构体指针
 * @return 四元数指针，指向设备内部存储的数据，不需要释放
 */
Quaternion_t* Bmi088_get_quaternion(Bmi088_device_t* bmi088);

/**
 * @brief 获取当前欧拉角（单位：度）
 * @param bmi088 BMI088设备结构体指针
 * @return 欧拉角指针，指向设备内部存储的数据，不需要释放
 */
Euler_angles_t* Bmi088_get_euler_angles(Bmi088_device_t* bmi088);

// 校验函数 (内部使用)
static Bmi088_error_e Verify_acc_chip_id(Bmi088_device_t* bmi088);
static Bmi088_error_e Verify_gyro_chip_id(Bmi088_device_t* bmi088);
static Bmi088_error_e Verify_acc_self_test(Bmi088_device_t* bmi088);
static Bmi088_error_e Verify_gyro_self_test(Bmi088_device_t* bmi088);
