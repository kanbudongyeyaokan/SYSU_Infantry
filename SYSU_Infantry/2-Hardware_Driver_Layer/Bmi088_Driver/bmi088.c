/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 22:54
 * @LastEditTime   : 2025-09-14
 * @Note           : 重构支持多实例
 * @Copyright(c)   : Minghang Li Copyright
 */
#include "bmi088.h"

#include <math.h>
#include <stdlib.h>

#include "bmi088_regNdef.h"
#include "gpio.h"
#include "spi.h"

// 静态BMI088设备实例存储区
static Bmi088_device_t bmi088_instances[1]; // 可以根据需要增加
static uint8_t bmi088_instance_count = 0;

/**
 * @brief BMI088设备初始化
 * @param config BMI088配置结构体指针
 * @return BMI088设备结构体指针，如果初始化失败则返回NULL
 */
Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config)
{
    // 检查是否还有空闲实例
    if (bmi088_instance_count >= sizeof(bmi088_instances)/sizeof(bmi088_instances[0])) {
        return NULL; // 没有更多的实例可用
    }

    // 获取一个空闲实例
    Bmi088_device_t* bmi088 = &bmi088_instances[bmi088_instance_count++];
    
    // 复制配置
    bmi088->config = *config;
    bmi088->last_error = NO_ERROR;

    // 初始化BMI088
    bmi088->last_error = Bmi088_init(bmi088);
    
    return bmi088;
}

static Bmi088_error_e Bmi088_init(Bmi088_device_t* bmi088) {
    Bmi088_error_e error = NO_ERROR;

    Bmi088_conf_init(bmi088);

    error |= Verify_acc_chip_id(bmi088);
    error |= Verify_gyro_chip_id(bmi088);
    
    if (bmi088->config.enable_accel_self_test) {
        error |= Verify_acc_self_test(bmi088);
    }
    
    if (bmi088->config.enable_gyro_self_test) {
        error |= Verify_gyro_self_test(bmi088);
    }
    
    return error;
}

static void Write_data_to_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_SET);
}

static void Write_data_to_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_SET);
}

static void Read_single_data_from_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(bmi088->config.spi_handle, data, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_SPI_Receive(bmi088->config.spi_handle, data, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_SET);
}

static void Read_single_data_from_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(bmi088->config.spi_handle, data, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_SET);
}

static void Read_multi_data_from_acc(Bmi088_device_t* bmi088, uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(bmi088->config.spi_handle, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(bmi088->config.spi_handle, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(bmi088->config.accel_cs_gpio_port, bmi088->config.accel_cs_gpio_pin, GPIO_PIN_SET);
}

static void Read_multi_data_from_gyro(Bmi088_device_t* bmi088, uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(bmi088->config.spi_handle, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_TX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(bmi088->config.spi_handle, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(bmi088->config.spi_handle) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(bmi088->config.gyro_cs_gpio_port, bmi088->config.gyro_cs_gpio_pin, GPIO_PIN_SET);
}

static void Bmi088_conf_init(Bmi088_device_t* bmi088) {
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    Write_data_to_acc(bmi088, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开加速度计电源
    Write_data_to_acc(bmi088, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 加速度计变成正常模式
    Write_data_to_acc(bmi088, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    Write_data_to_gyro(bmi088, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 陀螺仪变成正常模式
    Write_data_to_gyro(bmi088, GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    Write_data_to_acc(bmi088, ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 写入配置，正常带宽，1600hz输出频率
    Write_data_to_acc(bmi088, ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    Write_data_to_gyro(bmi088, GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    Write_data_to_gyro(bmi088, GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

static Bmi088_error_e Verify_acc_chip_id(Bmi088_device_t* bmi088) {
    uint8_t chip_id;
    Read_single_data_from_acc(bmi088, ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL) {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

static Bmi088_error_e Verify_gyro_chip_id(Bmi088_device_t* bmi088) {
    uint8_t chip_id;
    Read_single_data_from_gyro(bmi088, GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL) {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

static Bmi088_error_e Verify_acc_self_test(Bmi088_device_t* bmi088) {
    Acc_raw_data_t pos_data, neg_data;
    Acc_raw_data_t *data_ptr;
    
    Write_data_to_acc(bmi088, ACC_RANGE_ADDR, ACC_RANGE_24G);
    Write_data_to_acc(bmi088, ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    Write_data_to_acc(bmi088, ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    data_ptr = Read_acc_data(bmi088);
    pos_data = *data_ptr;  // 复制数据，避免指针被后续调用覆盖
    
    Write_data_to_acc(bmi088, ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    data_ptr = Read_acc_data(bmi088);
    neg_data = *data_ptr;  // 复制数据，避免指针被后续调用覆盖
    
    Write_data_to_acc(bmi088, ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f)) {
        return ACC_DATA_ERR;
    }
    Write_data_to_acc(bmi088, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    Write_data_to_acc(bmi088, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    Write_data_to_acc(bmi088, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    Write_data_to_acc(bmi088, ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
    Write_data_to_acc(bmi088, ACC_RANGE_ADDR, ACC_RANGE_3G);
    return NO_ERROR;
}

static Bmi088_error_e Verify_gyro_self_test(Bmi088_device_t* bmi088) {
    Write_data_to_gyro(bmi088, GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0) {
        Read_single_data_from_gyro(bmi088, GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    Read_single_data_from_gyro(bmi088, GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0) {
        return NO_ERROR;
    } else {
        return GYRO_DATA_ERR;
    }
}

Acc_raw_data_t* Read_acc_data(Bmi088_device_t* bmi088) {
    static Acc_raw_data_t acc_data;  // 静态变量，确保返回值持久有效
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    
    Read_single_data_from_acc(bmi088, ACC_RANGE_ADDR, &range);
    Read_multi_data_from_acc(bmi088, ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    
    acc_data.x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    acc_data.y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    acc_data.z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
    
    // 同时更新设备数据结构中的数据（可选）
    bmi088->data.acc_data.acc_raw_data = acc_data;
    
    return &acc_data;
}

Gyro_raw_data_t* Read_gyro_data(Bmi088_device_t* bmi088) {
    static Gyro_raw_data_t gyro_data;  // 静态变量，确保返回值持久有效
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    
    Read_single_data_from_gyro(bmi088, GYRO_RANGE_ADDR, &range);
    switch (range) {
        case 0x00:
            unit = 16.384;
            break;
        case 0x01:
            unit = 32.768;
            break;
        case 0x02:
            unit = 65.536;
            break;
        case 0x03:
            unit = 131.072;
            break;
        case 0x04:
            unit = 262.144;
            break;
        default:
            unit = 16.384;
            break;
    }
    Read_multi_data_from_gyro(bmi088, GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    
    gyro_data.roll = (float)gyro[0] / unit * DEG2SEC;
    gyro_data.pitch = (float)gyro[1] / unit * DEG2SEC;
    gyro_data.yaw = (float)gyro[2] / unit * DEG2SEC;
    
    // 同时更新设备数据结构中的数据（可选）
    bmi088->data.acc_data.acc_raw_data = bmi088->data.acc_data.acc_raw_data;
    
    return &gyro_data;
}

float* Read_acc_sensor_time(Bmi088_device_t* bmi088) {
    static float sensor_time;  // 静态变量，确保返回值持久有效
    uint8_t buf[SENSORTIME_LEN];
    
    Read_multi_data_from_acc(bmi088, SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    sensor_time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
    
    // 同时更新设备数据结构中的数据（可选）
    bmi088->data.acc_data.sensor_time = sensor_time;
    
    return &sensor_time;
}

float* Read_acc_temperature(Bmi088_device_t* bmi088) {
    static float temperature;  // 静态变量，确保返回值持久有效
    uint8_t buf[TEMP_LEN];
    
    Read_multi_data_from_acc(bmi088, TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }
    temperature = temp_int11 * TEMP_UNIT + TEMP_BIAS;
    
    // 同时更新设备数据结构中的数据（可选）
    bmi088->data.acc_data.temperature = temperature;
    
    return &temperature;
}

