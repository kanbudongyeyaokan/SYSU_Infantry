#include "ins.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "imu_temp.h"
#include "MahonyAHRS.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

static Bmi088_device_t *bmi088_dev;
static attitude_t g_attitude;
static uint32_t last_dwt_cnt = 0;
static float gyro_offset[3] = {0}; // 零偏校准数据

// 声明阻塞读取函数 (在bmi088.c中定义)
extern void Bmi088_read_gyro_blocking(Bmi088_device_t* dev);

void INS_Init(void)
{
    DWT_Init(168);
    Imu_Temp_Init();

    Bmi088_config_t bmi_conf = {
        .spi_handle = &hspi1,
        .accel_cs_gpio_port = GPIOA,
        .accel_cs_gpio_pin = GPIO_PIN_4,
        .gyro_cs_gpio_port = GPIOB,
        .gyro_cs_gpio_pin = GPIO_PIN_0,
    };
    bmi088_dev = Bmi088_device_init(&bmi_conf);

    // --- 启动校准 ---
    if(bmi088_dev) {
        const int CALIB_COUNT = 1000;
        float gyro_sum[3] = {0};

        // 循环读取，确保机器人静止
        for(int i=0; i<CALIB_COUNT; i++) {
            Bmi088_read_gyro_blocking(bmi088_dev);
            gyro_sum[0] += bmi088_dev->data.gyro_data.gyro_raw_data.roll;
            gyro_sum[1] += bmi088_dev->data.gyro_data.gyro_raw_data.pitch;
            gyro_sum[2] += bmi088_dev->data.gyro_data.gyro_raw_data.yaw;

            // 顺便跑一下温控
            Bmi088_read_temp(bmi088_dev);
            Imu_Temp_Control(bmi088_dev->data.acc_data.temperature);

            HAL_Delay(1);
        }

        gyro_offset[0] = gyro_sum[0] / CALIB_COUNT;
        gyro_offset[1] = gyro_sum[1] / CALIB_COUNT;
        gyro_offset[2] = gyro_sum[2] / CALIB_COUNT;
    }

    DWT_GetDeltaT(&last_dwt_cnt); // 重置时间戳
}

void INS_Task(void)
{
    if (bmi088_dev == NULL) return;

    // 1. 计算dt
    float dt = DWT_GetDeltaT(&last_dwt_cnt);
    // dt保护：调试暂停或任务卡死时防止数据发散
    if(dt > 0.01f) dt = 0.01f;

    // 2. 读取数据
    Bmi088_read_acc_dma(bmi088_dev);
    Bmi088_read_gyro_dma(bmi088_dev);

    // 3. 温控
    Bmi088_read_temp(bmi088_dev);
    Imu_Temp_Control(bmi088_dev->data.acc_data.temperature);

    // 4. 准备算法数据 (扣除零偏)
    // 注意：bmi088.c 中已经转换了坐标系和单位，这里直接使用
    float ax = bmi088_dev->data.acc_data.acc_raw_data.x;
    float ay = bmi088_dev->data.acc_data.acc_raw_data.y;
    float az = bmi088_dev->data.acc_data.acc_raw_data.z;

    float gx = bmi088_dev->data.gyro_data.gyro_raw_data.roll  - gyro_offset[0];
    float gy = bmi088_dev->data.gyro_data.gyro_raw_data.pitch - gyro_offset[1];
    float gz = bmi088_dev->data.gyro_data.gyro_raw_data.yaw   - gyro_offset[2];

    // 5. 姿态解算 (Mahony)
    MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);

    // 6. 获取欧拉角并填充结构体
    float p, r, y;
    Mahony_GetEulerAngle(&p, &r, &y); // 输出为弧度

    // 赋值给子结构体 euler_angles (转换为角度)
    g_attitude.euler_angles.pitch = p * 57.29578f;
    g_attitude.euler_angles.roll  = r * 57.29578f;
    g_attitude.euler_angles.yaw   = y * 57.29578f;

    // 填充原始数据
    g_attitude.accel_raw.x = ax;
    g_attitude.accel_raw.y = ay;
    g_attitude.accel_raw.z = az;

    g_attitude.gyro_raw.roll  = gx;
    g_attitude.gyro_raw.pitch = gy;
    g_attitude.gyro_raw.yaw   = gz;

    g_attitude.temperature = bmi088_dev->data.acc_data.temperature;
    g_attitude.dt = dt;
}

const attitude_t* INS_Get_Attitude(void)
{
    return &g_attitude;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        Bmi088_DMA_RxCpltCallback(bmi088_dev);
    }
}