#include "bmi088.h"
#include "bsp_dwt.h"
#include <string.h>
#include "bmi088_reg_def.h"

static Bmi088_device_t bmi088_instances[1];
static uint8_t bmi088_instance_count = 0;

// ------------------- 核心参数配置 -------------------
// 加速度计灵敏度: Range +-3G -> m/s^2
static float acc_sensitivity = (3.0f / 32768.0f) * 9.80665f;

// 陀螺仪灵敏度: Range +-2000 dps -> rad/s
static float gyro_sensitivity = (1.0f / 16.384f) * (3.1415926535f / 180.0f);
// --------------------------------------------------

static void Bmi088_Delay_us(float us) { DWT_Delay(us * 1e-6f); }
static void Bmi088_Delay_ms(float ms) { DWT_Delay(ms * 1e-3f); }
static void Write_data(SPI_HandleTypeDef* hspi, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t data);
static void Bmi088_conf_init(Bmi088_device_t* dev);

osSemaphoreDef(bmi_sem);

Bmi088_device_t* Bmi088_device_init(Bmi088_config_t* config)
{
    if (bmi088_instance_count >= 1) return NULL;
    Bmi088_device_t* bmi088 = &bmi088_instances[bmi088_instance_count++];

    // 复制配置
    bmi088->config.spi_handle = config->spi_handle;
    bmi088->config.accel_cs_gpio_port = config->accel_cs_gpio_port;
    bmi088->config.accel_cs_gpio_pin = config->accel_cs_gpio_pin;
    bmi088->config.gyro_cs_gpio_port = config->gyro_cs_gpio_port;
    bmi088->config.gyro_cs_gpio_pin = config->gyro_cs_gpio_pin;

    bmi088->spi_sem = osSemaphoreCreate(osSemaphore(bmi_sem), 0);

    // 硬件配置初始化
    Bmi088_conf_init(bmi088);
    return bmi088;
}

// 通用DMA读取函数
static void Read_data_dma(Bmi088_device_t* dev, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t* rx_buf, uint8_t len) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t txData = addr | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(dev->config.spi_handle, &txData, 1, 10);

    if(HAL_SPI_Receive_DMA(dev->config.spi_handle, rx_buf, len) == HAL_OK) {
        if (osSemaphoreWait(dev->spi_sem, 2) != osOK) {
            HAL_SPI_DMAStop(dev->config.spi_handle);
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            return;
        }
    } else {
        HAL_SPI_Receive(dev->config.spi_handle, rx_buf, len, 10);
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

// ------------------- 关键数据读取与坐标变换 -------------------

void Bmi088_read_acc_dma(Bmi088_device_t* dev) {
    Read_data_dma(dev, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin,
                  ACC_X_LSB_ADDR, dev->acc_rx_buf, 7);

    uint8_t* pData = &dev->acc_rx_buf[1];
    int16_t acc_int[3];
    acc_int[0] = (int16_t)((pData[1] << 8) | pData[0]); // X
    acc_int[1] = (int16_t)((pData[3] << 8) | pData[2]); // Y
    acc_int[2] = (int16_t)((pData[5] << 8) | pData[4]); // Z

    // 【C板坐标系修正】 Chip X->Y, Y->-X, Z->Z
    dev->data.acc_data.acc_raw_data.x =  (float)acc_int[1] * acc_sensitivity;
    dev->data.acc_data.acc_raw_data.y = -(float)acc_int[0] * acc_sensitivity;
    dev->data.acc_data.acc_raw_data.z =  (float)acc_int[2] * acc_sensitivity;
}

void Bmi088_read_gyro_dma(Bmi088_device_t* dev) {
    Read_data_dma(dev, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin,
                  GYRO_RATE_X_LSB_ADDR, dev->gyro_rx_buf, 6);

    uint8_t* pData = dev->gyro_rx_buf;
    int16_t gyro_int[3];
    gyro_int[0] = (int16_t)((pData[1] << 8) | pData[0]);
    gyro_int[1] = (int16_t)((pData[3] << 8) | pData[2]);
    gyro_int[2] = (int16_t)((pData[5] << 8) | pData[4]);

    // 【坐标系修正】
    dev->data.gyro_data.gyro_raw_data.roll  =  (float)gyro_int[1] * gyro_sensitivity;
    dev->data.gyro_data.gyro_raw_data.pitch = -(float)gyro_int[0] * gyro_sensitivity;
    dev->data.gyro_data.gyro_raw_data.yaw   =  (float)gyro_int[2] * gyro_sensitivity;
}

// 阻塞式读取 (用于初始化校准)
void Bmi088_read_gyro_blocking(Bmi088_device_t* dev) {
    HAL_GPIO_WritePin(dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GPIO_PIN_RESET);
    uint8_t tx = GYRO_RATE_X_LSB_ADDR | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(dev->config.spi_handle, &tx, 1, 10);
    HAL_SPI_Receive(dev->config.spi_handle, dev->gyro_rx_buf, 6, 10);
    HAL_GPIO_WritePin(dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GPIO_PIN_SET);

    int16_t gyro_int[3];
    uint8_t* pData = dev->gyro_rx_buf;
    gyro_int[0] = (int16_t)((pData[1] << 8) | pData[0]);
    gyro_int[1] = (int16_t)((pData[3] << 8) | pData[2]);
    gyro_int[2] = (int16_t)((pData[5] << 8) | pData[4]);

    dev->data.gyro_data.gyro_raw_data.roll  =  (float)gyro_int[1] * gyro_sensitivity;
    dev->data.gyro_data.gyro_raw_data.pitch = -(float)gyro_int[0] * gyro_sensitivity;
    dev->data.gyro_data.gyro_raw_data.yaw   =  (float)gyro_int[2] * gyro_sensitivity;
}

void Bmi088_read_temp(Bmi088_device_t* dev) {
    uint8_t buf[2];
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

void Bmi088_DMA_RxCpltCallback(Bmi088_device_t* dev) {
    if (dev && dev->spi_sem) {
        osSemaphoreRelease(dev->spi_sem);
    }
}

static void Write_data(SPI_HandleTypeDef* hspi, GPIO_TypeDef* port, uint16_t pin, uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    uint8_t txData[2] = {addr & BMI088_SPI_WRITE_CODE, data};
    HAL_SPI_Transmit(hspi, txData, 2, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    Bmi088_Delay_us(200);
}

static void Bmi088_conf_init(Bmi088_device_t* dev) {
    SPI_HandleTypeDef* spi = dev->config.spi_handle;

    // 1. 加速度计配置 (Range: +-3G, ODR: 800Hz)
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    Bmi088_Delay_ms(50);
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_RANGE_ADDR, ACC_RANGE_3G);
    Write_data(spi, dev->config.accel_cs_gpio_port, dev->config.accel_cs_gpio_pin, ACC_CONF_ADDR, (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_800_Hz));

    // 2. 陀螺仪配置 (Range: +-2000deg/s, ODR: 2000Hz)
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    Bmi088_Delay_ms(50);
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    Write_data(spi, dev->config.gyro_cs_gpio_port, dev->config.gyro_cs_gpio_pin, GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_230Hz);
}