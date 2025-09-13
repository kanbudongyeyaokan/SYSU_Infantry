/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 22:54
 * @LastEditTime   : 2022-11-28 16:09
 * @Note           :
 * @Copyright(c)   : Minghang Li Copyright
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088_regNdef.h"

#define BMI088_SPI hspi1
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0

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

typedef struct {
    Acc_data_t acc_data;
    Bmi088_error_e bmi088_error;
} Bmi088_data_t;

// 基础函数
void Write_data_to_acc(uint8_t addr, uint8_t data);
void Write_data_to_gyro(uint8_t addr, uint8_t data);
void Read_single_data_from_acc(uint8_t addr, uint8_t *data);
void Read_single_data_from_gyro(uint8_t addr, uint8_t *data);
void Read_multi_data_from_acc(uint8_t addr, uint8_t len, uint8_t *data);
void Read_multi_data_from_gyro(uint8_t addr, uint8_t len, uint8_t *data);

// 初始化函数
Bmi088_error_e Bmi088_init(void);
void Bmi088_conf_init(void);

// 功能函数
void Read_acc_data(Acc_raw_data_t *data);
void Read_gyro_data(Gyro_raw_data_t *data);
void Read_acc_sensor_time(float *time);
void Read_acc_temperature(float *temp);

// 校验函数
Bmi088_error_e Verify_acc_chip_id(void);
Bmi088_error_e Verify_gyro_chip_id(void);
Bmi088_error_e Verify_acc_self_test(void);
Bmi088_error_e Verify_gyro_self_test(void);
