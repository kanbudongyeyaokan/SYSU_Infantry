/**
 * @file    bmi088.c
 * @brief   BMI088 驱动实现 (严格复刻旧代码时序)
 */

#include "bmi088.h"
#include "bmi088_reg_def.h"
#include "algorithm_ekf.h"
#include "bsp_dwt.h"
#include <string.h>
#include <math.h>
#include "spi.h"

// ================= 私有宏定义 =================
#define BMI088_ACC_BUF_LEN  8 
#define BMI088_GYRO_BUF_LEN 7 

// ================= 私有对象结构体 =================
typedef struct {
    Spi_device_t *spi_acc;
    Spi_device_t *spi_gyro;
    uint8_t acc_tx_buf[BMI088_ACC_BUF_LEN];
    uint8_t acc_rx_buf[BMI088_ACC_BUF_LEN];
    uint8_t gyro_tx_buf[BMI088_GYRO_BUF_LEN];
    uint8_t gyro_rx_buf[BMI088_GYRO_BUF_LEN];
    Ekf_state_t ekf_state;
    volatile bool acc_done;
    volatile bool gyro_done;
} Bmi088_Driver_t;

static Bmi088_Driver_t bmi_dev;

// ================= 内部函数声明 =================
// 【关键】使用严格时序的写函数
static void Bmi088_Write_Reg_Strict(Spi_device_t *dev, uint8_t addr, uint8_t data);
static void Bmi088_Read_Reg(Spi_device_t *dev, uint8_t addr, uint8_t *data, uint8_t len);
static void Bmi088_Config_HardWare(void);

// ================= 回调函数 =================
static void Acc_Callback(void *context) {
    Bmi088_Driver_t *dev = (Bmi088_Driver_t *)context;
    dev->acc_done = true;
}
static void Gyro_Callback(void *context) {
    Bmi088_Driver_t *dev = (Bmi088_Driver_t *)context;
    dev->gyro_done = true;
}

// ================= 接口实现 =================

static bool BMI088_Interface_Init(void) {
    // 1. 初始化 SPI
    Spi_init_config_t acc_conf = {
        .hspi = &hspi1, .cs_port = GPIOA, .cs_pin = GPIO_PIN_4,
        .callback = Acc_Callback, .context = &bmi_dev
    };
    bmi_dev.spi_acc = Spi_device_init(&acc_conf);

    Spi_init_config_t gyro_conf = {
        .hspi = &hspi1, .cs_port = GPIOB, .cs_pin = GPIO_PIN_0,
        .callback = Gyro_Callback, .context = &bmi_dev
    };
    bmi_dev.spi_gyro = Spi_device_init(&gyro_conf);

    if (!bmi_dev.spi_acc || !bmi_dev.spi_gyro) return false;

    // 2. 【关键修改】先配置(软复位+上电)，再读 ID
    // 旧代码逻辑：Bmi088_conf_init() -> Verify_id()
    // 如果芯片处于混乱状态，必须先复位才能读对 ID
    Bmi088_Config_HardWare();
    
    // 给一点时间让配置生效
    HAL_Delay(50);

    // 3. 校验 ID
    uint8_t chip_id[2] = {0};
    
    // Accel ID Check
    Bmi088_Read_Reg(bmi_dev.spi_acc, ACC_CHIP_ID_ADDR, chip_id, 2);
    if (chip_id[1] != ACC_CHIP_ID_VAL) return false;

    // Gyro ID Check
    Bmi088_Read_Reg(bmi_dev.spi_gyro, GYRO_CHIP_ID_ADDR, chip_id, 1);
    if (chip_id[0] != GYRO_CHIP_ID_VAL) return false;

    // 4. EKF 初始化
    Ekf_config_t ekf_conf = {
        .process_noise_q = 10.0f, .measurement_noise_r = 1000000.0f,
        .dt = 0.001f, .fading_factor = 0.9996f, .gyro_bias_noise = 0.001f,
        .enable_bias_correction = true
    };
    Ekf_init(&bmi_dev.ekf_state, &ekf_conf);

    // 5. 预填充 Buffer
    memset(bmi_dev.acc_tx_buf, 0xFF, BMI088_ACC_BUF_LEN);
    bmi_dev.acc_tx_buf[0] = ACC_X_LSB_ADDR | BMI088_SPI_READ_CODE; 
    
    memset(bmi_dev.gyro_tx_buf, 0xFF, BMI088_GYRO_BUF_LEN);
    bmi_dev.gyro_tx_buf[0] = GYRO_RATE_X_LSB_ADDR | BMI088_SPI_READ_CODE;

    return true;
}

static void BMI088_Interface_Start_Read(void) {
    bmi_dev.acc_done = false;
    
    // 【复刻旧代码玄学】旧代码在读数据前，先读了一次 Range 寄存器
    // 这可能起到了 Flush 或者 唤醒 的作用
    // 虽然这会降低一点点效率，但为了跑通，我们加上
    // (由于 DMA 是一次性配置，这里我们在 Process 里不好加，
    //  但如果上面 Init 成功了，这里通常不需要。先保持原样，如果不行再加)
    
    Spi_swap_data_dma(bmi_dev.spi_acc, bmi_dev.acc_tx_buf, bmi_dev.acc_rx_buf, BMI088_ACC_BUF_LEN);
}

static bool BMI088_Interface_Wait_Data(void) {
    uint32_t timeout = 0;
    while (!bmi_dev.acc_done) { if (++timeout > 5000) return false; }
    
    bmi_dev.gyro_done = false;
    Spi_swap_data_dma(bmi_dev.spi_gyro, bmi_dev.gyro_tx_buf, bmi_dev.gyro_rx_buf, BMI088_GYRO_BUF_LEN);
    
    timeout = 0;
    while (!bmi_dev.gyro_done) { if (++timeout > 5000) return false; }
    
    return true;
}

static void BMI088_Interface_Process(Ins_data_t *out_data, float dt_s) {
    // --- Accel 解析 ---
    uint8_t *pa = bmi_dev.acc_rx_buf;
    int16_t acc_int[3];
    // 索引偏移：Cmd[0], Dummy[1], Data[2]...
    acc_int[0] = (int16_t)((pa[3] << 8) | pa[2]);
    acc_int[1] = (int16_t)((pa[5] << 8) | pa[4]);
    acc_int[2] = (int16_t)((pa[7] << 8) | pa[6]);

    // 单位转换 (3G)
    // 你的旧代码: acc[0] * BMI088_ACCEL_3G_SEN (0.0008974...)
    // 结果是 g。为了统一单位为 m/s^2，需要 * 9.81
    const float ACC_K = BMI088_ACCEL_3G_SEN * 9.81f; 
    float acc_mzs[3] = { acc_int[0] * ACC_K, acc_int[1] * ACC_K, acc_int[2] * ACC_K };

    // --- Gyro 解析 ---
    uint8_t *pg = bmi_dev.gyro_rx_buf;
    int16_t gyro_int[3];
    // 索引偏移：Cmd[0], Data[1]...
    // 你的DMA接收 buffer[1] 对应 LSB，buffer[2] 对应 MSB
    gyro_int[0] = (int16_t)((pg[2] << 8) | pg[1]);
    gyro_int[1] = (int16_t)((pg[4] << 8) | pg[3]);
    gyro_int[2] = (int16_t)((pg[6] << 8) | pg[5]);

    // 单位转换 (500dps, 匹配旧代码配置)
    // 500dps -> 65.536 LSB/deg/s
    const float GYRO_K_DEG = 1.0f / 65.536f;
    float gyro_rad[3] = {
        gyro_int[0] * GYRO_K_DEG * DEG2SEC,
        gyro_int[1] * GYRO_K_DEG * DEG2SEC,
        gyro_int[2] * GYRO_K_DEG * DEG2SEC
    };

    Ekf_update(&bmi_dev.ekf_state, acc_mzs, gyro_rad, dt_s);

    out_data->acc_body.x = acc_mzs[0]; 
    out_data->acc_body.y = acc_mzs[1];
    out_data->acc_body.z = acc_mzs[2];
    out_data->gyro_body.x = gyro_int[0] * GYRO_K_DEG; 
    out_data->gyro_body.y = gyro_int[1] * GYRO_K_DEG;
    out_data->gyro_body.z = gyro_int[2] * GYRO_K_DEG;
    out_data->euler.roll  = bmi_dev.ekf_state.euler.roll;
    out_data->euler.pitch = bmi_dev.ekf_state.euler.pitch;
    out_data->euler.yaw   = bmi_dev.ekf_state.euler.yaw;
    out_data->total_yaw   = bmi_dev.ekf_state.yaw_total_angle;
    out_data->round_count = (int32_t)floorf((out_data->total_yaw + 180.0f) / 360.0f);
}

static const Ins_driver_interface_t bmi088_drv = {
    .init = BMI088_Interface_Init,
    .start_read = BMI088_Interface_Start_Read,
    .wait_data = BMI088_Interface_Wait_Data,
    .process_data = BMI088_Interface_Process
};

const Ins_driver_interface_t* BMI088_Get_Driver(void) { return &bmi088_drv; }

// ================= 底层配置 (严格复刻) =================

// 【核心修复】完全模仿旧代码 Write_data_to_acc 的行为
static void Bmi088_Write_Reg_Strict(Spi_device_t *dev, uint8_t addr, uint8_t data) {
    // 1. 手动拉低 CS
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    
    // 2. 发送地址 (单独发送1字节)
    uint8_t tx_addr = addr & BMI088_SPI_WRITE_CODE;
    HAL_SPI_Transmit(dev->hspi, &tx_addr, 1, 100);
    // 旧代码在这里没有显式 delay，但分两次调用 Transmit 本身就有间隙
    
    // 3. 发送数据 (单独发送1字节)
    HAL_SPI_Transmit(dev->hspi, &data, 1, 100);
    
    // 4. 【关键】旧代码在这里延时了 1ms
    HAL_Delay(1); 
    
    // 5. 拉高 CS
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// 读寄存器 (用于ID校验)
static void Bmi088_Read_Reg(Spi_device_t *dev, uint8_t addr, uint8_t *data, uint8_t len) {
    // 这里使用阻塞式读，模仿旧代码 Read_multi
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    
    uint8_t tx_addr = addr | BMI088_SPI_READ_CODE;
    HAL_SPI_Transmit(dev->hspi, &tx_addr, 1, 100);
    
    // 读 Dummy + Data
    // 旧代码是先 Receive 1 byte (Dummy)，再 Loop Receive Data
    uint8_t dummy;
    HAL_SPI_Receive(dev->hspi, &dummy, 1, 100); // Dummy
    
    HAL_SPI_Receive(dev->hspi, data, len, 100); // Data
    
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void Bmi088_Config_HardWare(void) {
    // ---------------- Accel ----------------
    // 1. 软复位
    Bmi088_Write_Reg_Strict(bmi_dev.spi_acc, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    
    // 2. 先开电源 (旧代码顺序)
    Bmi088_Write_Reg_Strict(bmi_dev.spi_acc, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 旧代码里这里没有显式 Delay，但 Write_Reg_Strict 内部自带 1ms
    
    // 3. 切换模式
    Bmi088_Write_Reg_Strict(bmi_dev.spi_acc, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    
    // 4. 配置量程 (3G)
    Bmi088_Write_Reg_Strict(bmi_dev.spi_acc, ACC_RANGE_ADDR, ACC_RANGE_3G);
    
    // 5. 配置带宽 (旧代码的位移逻辑: 0x80 | 0x80 | 0x0C = 0x8C)
    // 0x8C 对应: Reserved=1, BWP=Normal(如果左移4位的话..但旧代码移了6位), ODR=1600
    // 我们直接写 0x8C 确保一致
    Bmi088_Write_Reg_Strict(bmi_dev.spi_acc, ACC_CONF_ADDR, 0x8C);

    // ---------------- Gyro ----------------
    // 1. 软复位
    Bmi088_Write_Reg_Strict(bmi_dev.spi_gyro, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    
    // 2. 切换 Normal 模式
    Bmi088_Write_Reg_Strict(bmi_dev.spi_gyro, GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    
    // 3. 量程 (500dps, 匹配旧代码)
    Bmi088_Write_Reg_Strict(bmi_dev.spi_gyro, GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    
    // 4. 带宽
    Bmi088_Write_Reg_Strict(bmi_dev.spi_gyro, GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

// 占位
float BMI088_Get_Temp_Raw(void) { return 25.0f; }