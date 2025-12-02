#include "bmi088.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "bmi088_reg_def.h"
#include "gpio.h"
#include "spi.h"

// 包含EKF模块头文件
#include "algorithm_ekf.h"
#include "main.h"

// 静态BMI088设备实例存储区
static Bmi088_device_t bmi088_instances[1]; // 可以根据需要增加
static uint8_t bmi088_instance_count = 0;

#define BMI088_INIT_MAX_ATTEMPTS        3U
#define BMI088_READY_TIMEOUT_MS       300U
#define BMI088_READY_STABLE_COUNT      10U

static void Bmi088_set_identity_matrix(float matrix[3][3]) {
    memset(matrix, 0, sizeof(float) * 9);
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

static bool Bmi088_is_matrix_zero(const float matrix[3][3]) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            sum += fabsf(matrix[i][j]);
        }
    }
    return sum < 1e-6f;
}

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
    bmi088->config = *config;
    if (Bmi088_is_matrix_zero(bmi088->config.accel_rotation)) {
        Bmi088_set_identity_matrix(bmi088->config.accel_rotation);
    }
    if (Bmi088_is_matrix_zero(bmi088->config.gyro_rotation)) {
        Bmi088_set_identity_matrix(bmi088->config.gyro_rotation);
    }
    bmi088->last_error = NO_ERROR;

    // 复制配置
    bmi088->config = *config;
    if (Bmi088_is_matrix_zero(bmi088->config.accel_rotation)) {
        Bmi088_set_identity_matrix(bmi088->config.accel_rotation);
    }
    if (Bmi088_is_matrix_zero(bmi088->config.gyro_rotation)) {
        Bmi088_set_identity_matrix(bmi088->config.gyro_rotation);
    }
    bmi088->last_error = NO_ERROR;

    // 初始化BMI088
    bmi088->data.bmi088_error = NO_ERROR;
    bmi088->data.state = IMU_STATE_INIT;

    bmi088->last_error = Bmi088_init(bmi088);
    bmi088->data.bmi088_error = bmi088->last_error;
    if (bmi088->last_error != NO_ERROR) {
        bmi088->data.state = IMU_STATE_ERROR;
    }

    return bmi088;
}

static bool Bmi088_is_sample_valid(const Acc_raw_data_t* acc, const Gyro_raw_data_t* gyro) {
    if (acc == NULL || gyro == NULL) {
        return false;
    }
    if (!isfinite(acc->x) || !isfinite(acc->y) || !isfinite(acc->z) ||
        !isfinite(gyro->roll) || !isfinite(gyro->pitch) || !isfinite(gyro->yaw)) {
        return false;
    }
    if (fabsf(acc->x) > 200.0f || fabsf(acc->y) > 200.0f || fabsf(acc->z) > 200.0f) {
        return false;
    }
    if (fabsf(gyro->roll) > 2000.0f || fabsf(gyro->pitch) > 2000.0f || fabsf(gyro->yaw) > 2000.0f) {
        return false;
    }
    return true;
}

static Bmi088_error_e Bmi088_wait_device_ready(Bmi088_device_t* bmi088, uint32_t timeout_ms) {
    uint32_t start_tick = HAL_GetTick();
    uint32_t stable_count = 0U;

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        Acc_raw_data_t* acc = Read_acc_data(bmi088);
        Gyro_raw_data_t* gyro = Read_gyro_data(bmi088);

        if (Bmi088_is_sample_valid(acc, gyro)) {
            stable_count++;
            if (stable_count >= BMI088_READY_STABLE_COUNT) {
                return NO_ERROR;
            }
        } else {
            stable_count = 0U;
        }

        HAL_Delay(2);
    }

    return ACC_DATA_ERR;
}

static Bmi088_error_e Bmi088_init(Bmi088_device_t* bmi088) {
    Bmi088_error_e error = NO_ERROR;

    bmi088->data.state = IMU_STATE_INIT;

    for (uint8_t attempt = 0U; attempt < BMI088_INIT_MAX_ATTEMPTS; ++attempt) {
        Bmi088_conf_init(bmi088);

        error = Verify_acc_chip_id(bmi088);
        if (error != NO_ERROR) {
            HAL_Delay(10);
            continue;
        }

        error = Verify_gyro_chip_id(bmi088);
        if (error != NO_ERROR) {
            HAL_Delay(10);
            continue;
        }

        if (bmi088->config.enable_accel_self_test) {
            error |= Verify_acc_self_test(bmi088);
        }

        if (bmi088->config.enable_gyro_self_test) {
            error |= Verify_gyro_self_test(bmi088);
        }

        if (error != NO_ERROR) {
            HAL_Delay(20);
            continue;
        }

        error = Bmi088_wait_device_ready(bmi088, BMI088_READY_TIMEOUT_MS);
        if (error == NO_ERROR) {
            bmi088->data.state = IMU_STATE_CALIBRATING;
            break;
        }
    }

    if (error != NO_ERROR) {
        bmi088->data.state = IMU_STATE_ERROR;
    }

    return error;
}

//向加速度计写数据
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

//向陀螺仪写数据
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

//向加速度计读数据
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
//向陀螺仪读数据
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
    bmi088->data.gyro_data.gyro_raw_data = gyro_data;

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

// ===================== EKF算法实现 =====================

/**
 * @brief 初始化EKF参数和状态
 */
Bmi088_error_e Bmi088_ekf_init(Bmi088_device_t* bmi088, Ekf_config_t* ekf_config) {
    if (bmi088 == NULL || ekf_config == NULL) {
        return ACC_DATA_ERR; // 使用现有错误类型
    }

    // 确保配置参数完整，特别是fading_factor
    if (ekf_config->fading_factor <= 0.99f || ekf_config->fading_factor > 1.0f) {
        ekf_config->fading_factor = 0.9996f; // 使用默认优化值
    }

    // 调用EKF模块初始化函数
    Ekf_error_e ekf_err = Ekf_init(&bmi088->data.ekf_state, ekf_config);
    if (ekf_err != EKF_NO_ERROR) {
        return ACC_DATA_ERR; // 使用现有错误类型
    }

    return NO_ERROR;
}

/**
 * @brief 使用新的传感器数据更新EKF
 */
Bmi088_error_e Bmi088_ekf_update(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL ||
        bmi088->data.state == IMU_STATE_INIT ||
        bmi088->data.state == IMU_STATE_ERROR ||
        !bmi088->data.ekf_state.is_initialized) {
        if (bmi088 != NULL) {
            bmi088->data.bmi088_error = ACC_DATA_ERR;
            bmi088->data.state = IMU_STATE_ERROR;
        }
        return ACC_DATA_ERR;
    }

    // [关键修复] 将时间戳类型从 uint32_t 改为 float
    // DWT_GetTimeline_s() 返回的是秒 (float)，例如 1.234567
    // 如果用 uint32_t 接收，会截断为 1，导致毫秒级变化丢失，dt 计算错误
    static float last_update_time = 0.0f;
    float now = DWT_GetTimeline_s(); // 使用 float 接收
    float dt;

    if (last_update_time == 0.0f) {
        dt = 0.001f; // 首次运行使用默认值 1ms
    } else {
        dt = now - last_update_time;
        // 限制dt范围，避免异常值
        if (dt > 0.1f) dt = 0.001f;   // 最大100ms
        if (dt < 0.0001f) dt = 0.001f; // 最小0.1ms
    }
    last_update_time = now;

    // 读取传感器数据
    Acc_raw_data_t* acc_data = Read_acc_data(bmi088);
    Gyro_raw_data_t* gyro_data = Read_gyro_data(bmi088);

    if (acc_data == NULL || gyro_data == NULL) {
        bmi088->data.state = IMU_STATE_ERROR;
        bmi088->data.bmi088_error = ACC_DATA_ERR;
        return ACC_DATA_ERR;
    }

    //----------------- IMU坐标系到机体坐标系转换 -----------------
    float acc[3];
    const float (*acc_rot)[3] = bmi088->config.accel_rotation;
    for (uint8_t i = 0; i < 3; ++i) {
        acc[i] = acc_rot[i][0] * acc_data->x +
                 acc_rot[i][1] * acc_data->y +
                 acc_rot[i][2] * acc_data->z;
    }

    float gyro[3];
    const float (*gyro_rot)[3] = bmi088->config.gyro_rotation;
    for (uint8_t i = 0; i < 3; ++i) {
        gyro[i] = gyro_rot[i][0] * gyro_data->roll +
                  gyro_rot[i][1] * gyro_data->pitch +
                  gyro_rot[i][2] * gyro_data->yaw;
    }

    // 调用EKF模块更新函数
    Ekf_error_e ekf_err = Ekf_update(&bmi088->data.ekf_state, acc, gyro, dt);
    if (ekf_err != EKF_NO_ERROR) {
        bmi088->data.state = IMU_STATE_ERROR;
        bmi088->data.bmi088_error = ACC_DATA_ERR;
        return ACC_DATA_ERR;  // 使用现有错误类型
    }

    bmi088->data.bmi088_error = NO_ERROR;
    if (bmi088->data.state == IMU_STATE_ERROR) {
        bmi088->data.state = IMU_STATE_CALIBRATING;
    }

    return NO_ERROR;
}

/**
 * @brief 获取当前姿态四元数
 */
Quaternion_t* Bmi088_get_quaternion(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL || !bmi088->data.ekf_state.is_initialized) {
        return NULL;
    }
    return &bmi088->data.ekf_state.quaternion;
}

/**
 * @brief 获取当前欧拉角
 */
Euler_angles_t* Bmi088_get_euler_angles(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL || !bmi088->data.ekf_state.is_initialized) {
        return NULL;
    }
    return &bmi088->data.ekf_state.euler;
}

/**
 * @brief 获取BMI088当前温度（用于EKF温度补偿）
 */
float Bmi088_get_temperature(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL) {
        return 25.0f; // 默认返回25°C
    }

    float* temp_ptr = Read_acc_temperature(bmi088);
    return temp_ptr ? *temp_ptr : 25.0f;
}

// ===================== 辅助函数实现 =====================

// EKF相关函数已经移到ekf.c中实现
