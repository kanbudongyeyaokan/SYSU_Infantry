#include "ins.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "imu_temp.h" // 引用温控头文件
#include "string.h"
#include "main.h"     // 获取 hspi1 定义

// 全局句柄
static Bmi088_device_t *bmi088_dev;
static attitude_t g_attitude;
static uint32_t last_dwt_cnt = 0;

// 外部引用的 SPI 句柄 (CubeMX 生成)
extern SPI_HandleTypeDef hspi1;

/**
 * @brief INS 模块初始化
 */
void INS_Init(void)
{
    // 1. 确保 DWT 已开启 (在 main.c 中可能已调用，这里防守性调用)
    // DWT_Init(168);

    // 2. 初始化温控 (PID & PWM)
    Imu_Temp_Init();

    // 3. 配置并初始化 BMI088
    Bmi088_config_t bmi_conf = {
        .spi_handle = &hspi1,
        // 请根据实际硬件原理图确认 CS 引脚
        .accel_cs_gpio_port = GPIOA,
        .accel_cs_gpio_pin = GPIO_PIN_4,
        .gyro_cs_gpio_port = GPIOB,
        .gyro_cs_gpio_pin = GPIO_PIN_0,
        // 生产环境可以关闭自检以加快启动
        .enable_accel_self_test = false,
        .enable_gyro_self_test = false
    };

    bmi088_dev = Bmi088_device_init(&bmi_conf);

    // 4. 初始化 EKF
    if (bmi088_dev != NULL) {
        Ekf_config_t ekf_conf = {
            .process_noise_q = 0.001f,
            .measurement_noise_r = 0.1f,
            .gyro_bias_noise = 0.0001f,
            .dt = 0.001f, // 初始 dt
            .enable_bias_correction = true,
            .static_threshold = 0.2f
        };
        Ekf_init(bmi088_dev, &ekf_conf);
    }

    // 5. 预热传感器 (丢弃前 50 组数据)
    for(int i=0; i<50; i++) {
        Bmi088_read_acc_dma(bmi088_dev);
        Bmi088_read_gyro_dma(bmi088_dev);
        HAL_Delay(2); // 初始化阶段可以使用 HAL_Delay
    }

    // 初始化时间戳
    DWT_GetDeltaT(&last_dwt_cnt);
}

/**
 * @brief INS 任务主循环逻辑
 * @note 此函数内部包含阻塞等待(信号量)，调用者不需要额外 delay (取决于具体实现)
 * 如果 Bmi088_read_xxx_dma 内部是阻塞的，则此函数耗时约 DMA传输时间 + 计算时间
 */
void INS_Task(void)
{
    if (bmi088_dev == NULL) return;

    // 1. 获取真实时间间隔 dt
    float dt = DWT_GetDeltaT(&last_dwt_cnt);
    g_attitude.dt = dt;

    // 2. 读取传感器数据 (DMA模式)
    // 注意：这两个函数内部会挂起任务等待 DMA 中断，释放 CPU
    Bmi088_read_acc_dma(bmi088_dev);
    Bmi088_read_gyro_dma(bmi088_dev);

    // 3. 温控逻辑
    Bmi088_read_temp(bmi088_dev);
    Imu_Temp_Control(bmi088_dev->data.acc_data.temperature);

    // 4. EKF 姿态解算
    Bmi088_ekf_update(bmi088_dev, dt);

    // 5. 更新全局数据给应用层使用
    Euler_angles_t* euler = Bmi088_get_euler_angles(bmi088_dev);
    Acc_raw_data_t* acc = &bmi088_dev->data.acc_data.acc_raw_data;
    Gyro_raw_data_t* gyro = &bmi088_dev->data.gyro_data.gyro_raw_data;

    if (acc && gyro && euler) {
        memcpy(&g_attitude.accel_raw, acc, sizeof(Acc_raw_data_t));
        memcpy(&g_attitude.gyro_raw, gyro, sizeof(Gyro_raw_data_t));
        memcpy(&g_attitude.euler_angles, euler, sizeof(Euler_angles_t));
        g_attitude.temperature = bmi088_dev->data.acc_data.temperature;
    }
}

/**
 * @brief 获取姿态数据接口
 */
const attitude_t* INS_Get_Attitude(void)
{
    return &g_attitude;
}

/**
 * @brief SPI 接收完成中断回调
 * @note  此函数由 HAL 库在 DMA 中断中调用
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 判断是否是 BMI088 使用的 SPI 接口
    if (hspi == &hspi1) {
        // 调用 BMI088 驱动层提供的回调，释放信号量唤醒任务
        Bmi088_DMA_RxCpltCallback(bmi088_dev);
    }
}