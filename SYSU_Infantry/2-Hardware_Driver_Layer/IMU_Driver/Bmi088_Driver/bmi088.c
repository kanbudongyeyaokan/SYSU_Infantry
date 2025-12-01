#include "bmi088.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bmi088_reg_def.h"

// 静态BMI088设备实例存储区
static Bmi088_device_t bmi088_instances[1];
static uint8_t bmi088_instance_count = 0;

// 内部变量记录量程对应的灵敏度，避免每次读取寄存器
static float acc_sensitivity = BMI088_ACCEL_3G_SEN;
static float gyro_sensitivity = 16.384f; // 默认2000dps

// 辅助延时函数
static void Bmi088_Delay_us(float us) {
    DWT_Delay(us * 1e-6f);
}

static void Bmi088_Delay_ms(float ms) {
    DWT_Delay(ms * 1e-3f);
}

// 内部函数声明
static void Bmi088_conf_init(Bmi088_device_t* dev);
static Bmi088_error_e Bmi088_init(Bmi088_device_t* dev);
static Bmi088_error_e Verify_acc_chip_id(Bmi088_device_t* bmi088);
static Bmi088_error_e Verify_gyro_chip_id(Bmi088_device_t* bmi088);

/**
 * @brief BMI088设备初始化
 */
Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config)
{
    if (bmi088_instance_count >= 1) {
        return NULL;
    }

    Bmi088_device_t* bmi088 = &bmi088_instances[bmi088_instance_count++];
    bmi088->config = *config;
    bmi088->last_error = NO_ERROR;

    // 初始化信号量，用于DMA同步
    osSemaphoreDef(bmi_sem);
    bmi088->spi_sem = osSemaphoreCreate(osSemaphore(bmi_sem), 1);
    // 初始获取一次信号量，确保它是空的(Taken)
    osSemaphoreWait(bmi088->spi_sem, 0);

    // 初始化BMI088
    bmi088->last_error = Bmi088_init(bmi088);

    return bmi088;
}

// 阻塞式写入 (初始化用，频率低，无需DMA)
static void Write_data(SPI_HandleTypeDef* hspi, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t txData[2] = {addr & BMI088_SPI_WRITE_CODE, data};
    HAL_SPI_Transmit(hspi, txData, 2, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    Bmi088_Delay_us(200); // 必要的传感器处理时间
}

// 阻塞式读取单字节 (初始化校验ID用)
static void Read_single_data(SPI_HandleTypeDef* hspi, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t* data) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t txData = addr | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(hspi, &txData, 1, 10);
    // 注意：ACC和GYRO在读取时行为可能略有不同，这里做通用处理需要小心
    // BMI088 ACC读取第一个字节是dummy，GYRO不是
    // 为了通用，这里假设调用者知道自己在读什么，如果是ACC，addr需要处理
    HAL_SPI_Receive(hspi, data, 1, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

// DMA读取通用函数
// 注意：ACC读取时 rx_buf[0] 是dummy byte
static void Read_data_dma(Bmi088_device_t* dev, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t* rx_buf, uint8_t len) {
    // 1. 拉低片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    // 2. 准备发送地址 (读命令)
    uint8_t txData = addr | BMI088_SPI_READ_CODE;

    // 3. 发送地址 (阻塞发送即可，非常快)
    HAL_SPI_Transmit(dev->config.spi_handle, &txData, 1, 10);

    // 4. 启动DMA接收
    if(HAL_SPI_Receive_DMA(dev->config.spi_handle, rx_buf, len) == HAL_OK) {
        // 5. 挂起任务，等待DMA中断唤醒 (超时时间 2ms)
        if (osSemaphoreWait(dev->spi_sem, 2) != osOK) {
            // 超时处理: 可能是DMA挂了，重置片选防止死锁
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            HAL_SPI_DMAStop(dev->config.spi_handle);
            return;
        }
    } else {
        // DMA启动失败，降级为阻塞读取以防死机
        HAL_SPI_Receive(dev->config.spi_handle, rx_buf, len, 10);
    }

    // 6. 拉高片选
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

// SPI DMA 完成回调函数 (需要在 stm32f4xx_it.c 或 main.c 的 HAL_SPI_RxCpltCallback 中调用)
void Bmi088_DMA_RxCpltCallback(Bmi088_device_t* dev) {
    if (dev && dev->spi_sem) {
        // 发送信号量唤醒任务
        osSemaphoreRelease(dev->spi_sem);
    }
}

// 读取加速度计 (DMA模式)
void Bmi088_read_acc_dma(Bmi088_device_t* dev) {
    // ACC需要读 Dummy(1) + Data(6) = 7字节
    Read_data_dma(dev, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin,
                  ACC_X_LSB_ADDR, dev->acc_rx_buf, 7);

    uint8_t* pData = &dev->acc_rx_buf[1]; // 跳过dummy byte
    int16_t acc_int[3];
    acc_int[0] = (int16_t)((pData[1] << 8) | pData[0]);
    acc_int[1] = (int16_t)((pData[3] << 8) | pData[2]);
    acc_int[2] = (int16_t)((pData[5] << 8) | pData[4]);

    dev->data.acc_data.acc_raw_data.x = acc_int[0] * acc_sensitivity;
    dev->data.acc_data.acc_raw_data.y = acc_int[1] * acc_sensitivity;
    dev->data.acc_data.acc_raw_data.z = acc_int[2] * acc_sensitivity;
}

// 读取陀螺仪 (DMA模式)
void Bmi088_read_gyro_dma(Bmi088_device_t* dev) {
    // GYRO直接读6字节，无dummy byte
    Read_data_dma(dev, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin,
                  GYRO_RATE_X_LSB_ADDR, dev->gyro_rx_buf, 6);

    uint8_t* pData = dev->gyro_rx_buf;
    int16_t gyro_int[3];
    gyro_int[0] = (int16_t)((pData[1] << 8) | pData[0]);
    gyro_int[1] = (int16_t)((pData[3] << 8) | pData[2]);
    gyro_int[2] = (int16_t)((pData[5] << 8) | pData[4]);

    // 转换为弧度制 (rad/s)，因为EKF内部使用弧度
    dev->data.gyro_data.gyro_raw_data.roll  = (gyro_int[0] / gyro_sensitivity) * DEG2SEC;
    dev->data.gyro_data.gyro_raw_data.pitch = (gyro_int[1] / gyro_sensitivity) * DEG2SEC;
    dev->data.gyro_data.gyro_raw_data.yaw   = (gyro_int[2] / gyro_sensitivity) * DEG2SEC;
}

// 读取温度 (阻塞读取即可，频率低)
void Bmi088_read_temp(Bmi088_device_t* dev) {
    uint8_t buf[2];
    // 加速度计读取需要先读dummy
    HAL_GPIO_WritePin(dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t tx = TEMP_MSB_ADDR | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(dev->config.spi_handle, &tx, 1, 10);
    uint8_t dummy;
    HAL_SPI_Receive(dev->config.spi_handle, &dummy, 1, 10);
    HAL_SPI_Receive(dev->config.spi_handle, buf, 2, 10);
    HAL_GPIO_WritePin(dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, GPIO_PIN_SET);

    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11 = (temp_uint11 > 1023) ? (int16_t)temp_uint11 - 2048 : (int16_t)temp_uint11;
    dev->data.acc_data.temperature = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

// 初始化配置
static void Bmi088_conf_init(Bmi088_device_t* dev) {
    SPI_HandleTypeDef* spi = dev->config.spi_handle;

    // 1. 加速度计配置
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    Bmi088_Delay_ms(50); // 复位等待

    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 设置 ACC 范围 +-3G
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_RANGE_ADDR, ACC_RANGE_3G);
    acc_sensitivity = BMI088_ACCEL_3G_SEN;

    // 写入配置：正常带宽，1600Hz 输出
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_CONF_ADDR, (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 2. 陀螺仪配置
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    Bmi088_Delay_ms(50);
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 设置 GYRO 范围 +-2000 dps
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    gyro_sensitivity = 16.384f;

    // 写入带宽：2000Hz ODR, 532Hz Filter
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

// ID 校验函数
static Bmi088_error_e Verify_acc_chip_id(Bmi088_device_t* dev) {
    uint8_t chip_id;
    // 加速度计读取ID也需要读dummy byte
    HAL_GPIO_WritePin(dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t tx = ACC_CHIP_ID_ADDR | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(dev->config.spi_handle, &tx, 1, 10);
    uint8_t dummy;
    HAL_SPI_Receive(dev->config.spi_handle, &dummy, 1, 10);
    HAL_SPI_Receive(dev->config.spi_handle, &chip_id, 1, 10);
    HAL_GPIO_WritePin(dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, GPIO_PIN_SET);

    return (chip_id == ACC_CHIP_ID_VAL) ? NO_ERROR : ACC_CHIP_ID_ERR;
}

static Bmi088_error_e Verify_gyro_chip_id(Bmi088_device_t* dev) {
    uint8_t chip_id;
    Read_single_data(dev->config.spi_handle, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_CHIP_ID_ADDR, &chip_id);
    return (chip_id == GYRO_CHIP_ID_VAL) ? NO_ERROR : GYRO_CHIP_ID_ERR;
}

static Bmi088_error_e Bmi088_init(Bmi088_device_t* dev) {
    Bmi088_error_e error = NO_ERROR;
    Bmi088_conf_init(dev);
    error |= Verify_acc_chip_id(dev);
    error |= Verify_gyro_chip_id(dev);
    return error;
}

// 修复后的 EKF 更新函数，增加 dt 参数
Bmi088_error_e Bmi088_ekf_update(Bmi088_device_t* bmi088, float dt) {
    if (bmi088 == NULL || !bmi088->data.ekf_state.is_initialized) {
        return ACC_DATA_ERR;
    }

    // 数据已经在 read_dma 函数中更新到结构体了，直接取用
    float acc[3] = {
        bmi088->data.acc_data.acc_raw_data.x,
        bmi088->data.acc_data.acc_raw_data.y,
        bmi088->data.acc_data.acc_raw_data.z
    };
    float gyro[3] = {
        bmi088->data.gyro_data.gyro_raw_data.roll,
        bmi088->data.gyro_data.gyro_raw_data.pitch,
        bmi088->data.gyro_data.gyro_raw_data.yaw
    };

    // 调用核心算法
    Ekf_update(&bmi088->data.ekf_state, acc, gyro, dt);

    return NO_ERROR;
}

// 兼容旧接口 (返回指针)
Acc_raw_data_t* Read_acc_data(Bmi088_device_t* bmi088) {
    return &bmi088->data.acc_data.acc_raw_data;
}

Gyro_raw_data_t* Read_gyro_data(Bmi088_device_t* bmi088) {
    return &bmi088->data.gyro_data.gyro_raw_data;
}

Quaternion_t* Bmi088_get_quaternion(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL || !bmi088->data.ekf_state.is_initialized) return NULL;
    return &bmi088->data.ekf_state.quaternion;
}

Euler_angles_t* Bmi088_get_euler_angles(Bmi088_device_t* bmi088) {
    if (bmi088 == NULL || !bmi088->data.ekf_state.is_initialized) return NULL;
    return &bmi088->data.ekf_state.euler;
}


