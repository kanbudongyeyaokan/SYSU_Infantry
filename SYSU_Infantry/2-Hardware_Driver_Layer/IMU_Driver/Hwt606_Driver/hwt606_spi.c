/**
 * @file    hwt606_spi.c
 * @brief   HWT606 SPI 驱动 (全双工连续传输修正版)
 */

#include "hwt606_spi.h"
#include <string.h>
#include "bsp_dwt.h"

// ================= 寄存器定义 =================
// 起始地址 0x34 (AccX_L)
#define REG_START_ADDR      0x34
// 有效数据长度: Acc(6) + Gyro(6) + Mag(6) + Angle(6) = 24
#define DATA_LEN            24
// 总传输长度 = 命令头(2) + 数据(24) = 26
#define TOTAL_LEN           (2 + DATA_LEN)

// ================= 私有对象结构体 =================
typedef struct {
    SPI_HandleTypeDef *hspi;

    struct {
        int16_t acc_x, acc_y, acc_z;
        int16_t gyro_x, gyro_y, gyro_z;
        int16_t roll, pitch, yaw;
    } raw;

    bool is_ready;
    bool read_success;

    float yaw_offset;
    bool is_offset_init;
} HWT606_Driver_t;

static HWT606_Driver_t hwt_dev;

// 【关键】全双工缓冲区
static uint8_t tx_buf[TOTAL_LEN]; // 发送缓冲区 (命令+Dummy)
static uint8_t rx_buf[TOTAL_LEN]; // 接收缓冲区 (垃圾+数据)

// ================= 内部辅助函数 =================

static void HWT_CS_Low(void) {
    HAL_GPIO_WritePin(HWT_CS_GPIO_Port, HWT_CS_Pin, GPIO_PIN_RESET);
}

static void HWT_CS_High(void) {
    HAL_GPIO_WritePin(HWT_CS_GPIO_Port, HWT_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 使用 TransmitReceive 保证时钟不断
 */
static bool HWT606_SPI_Transaction(void) {
    HAL_StatusTypeDef status;

    // 1. 准备发送数据包
    // Byte 0: 读命令标志 0x80 (例程 usTemp >> 8)
    // Byte 1: 寄存器地址 0x34 (例程 usTemp & 0xff)
    tx_buf[0] = 0x80;
    tx_buf[1] = REG_START_ADDR;

    // 后面的 24 个字节发 0x00 (Dummy) 产生时钟，把数据“挤”出来
    memset(&tx_buf[2], 0x00, DATA_LEN);

    // 2. CS 拉低
    HWT_CS_Low();

    // 【关键修复】根据例程，CS拉低后必须等待一下 (至少2us)
    // 如果没有 DWT，可以用 for(volatile int i=0; i<50; i++); 代替
    DWT_Delay_us(10);

    // 3. 全双工交换数据 (核心！)
    // 这会在一个连续的时钟流中完成：发命令 -> 读数据
    status = HAL_SPI_TransmitReceive(hwt_dev.hspi, tx_buf, rx_buf, TOTAL_LEN, 20);

    // 4. CS 拉高
    HWT_CS_High();

    return (status == HAL_OK);
}

// ================= 接口实现 =================

static bool HWT606_Init(void) {
    if (hwt_dev.hspi == NULL) return false;

    hwt_dev.yaw_offset = 0.0f;
    hwt_dev.is_offset_init = false;

    HWT_CS_High();
    HAL_Delay(200); // 传感器上电慢，多等会

    // 试读一次
    if (HWT606_SPI_Transaction()) {
        hwt_dev.is_ready = true;
        return true;
    }
    return false;
}

static void HWT606_Start_Read(void) {
    if (!hwt_dev.is_ready) {
        HWT606_Init();
        return;
    }

    if (HWT606_SPI_Transaction()) {
        hwt_dev.read_success = true;
    } else {
        hwt_dev.read_success = false;
    }
}

static bool HWT606_Wait_Data(void) {
    return hwt_dev.read_success;
}

static void HWT606_Process(Ins_data_t *out_data, float dt_s) {
    if (!hwt_dev.read_success) return;

    // --- 数据解析 ---
    // rx_buf 的结构是：
    // [0]: 垃圾数据 (对应发送 0x80 时回来的)
    // [1]: 垃圾数据 (对应发送 0x34 时回来的)
    // [2]: AccX_Low
    // [3]: AccX_High
    // ... 以此类推

    // 指针指向有效数据的起始位置 (Offset = 2)
    uint8_t *pData = &rx_buf[2];

    hwt_dev.raw.acc_x  = (int16_t)(pData[0] | (pData[1] << 8));
    hwt_dev.raw.acc_y  = (int16_t)(pData[2] | (pData[3] << 8));
    hwt_dev.raw.acc_z  = (int16_t)(pData[4] | (pData[5] << 8));

    hwt_dev.raw.gyro_x = (int16_t)(pData[6] | (pData[7] << 8));
    hwt_dev.raw.gyro_y = (int16_t)(pData[8] | (pData[9] << 8));
    hwt_dev.raw.gyro_z = (int16_t)(pData[10] | (pData[11] << 8));

    // [12-17] 是 Mag，跳过

    hwt_dev.raw.roll   = (int16_t)(pData[18] | (pData[19] << 8));
    hwt_dev.raw.pitch  = (int16_t)(pData[20] | (pData[21] << 8));
    hwt_dev.raw.yaw    = (int16_t)(pData[22] | (pData[23] << 8));

    // --- 单位转换 ---
    const float K_ANGLE = 180.0f / 32768.0f;
    const float K_GYRO  = 2000.0f / 32768.0f;
    const float K_ACC   = 16.0f / 32768.0f;

    float current_yaw = hwt_dev.raw.yaw * K_ANGLE;

    // --- 零漂扣除 ---
    if (hwt_dev.is_offset_init == false && current_yaw != 0.0f) {
        hwt_dev.yaw_offset = current_yaw;
        hwt_dev.is_offset_init = true;
    }
    float final_yaw = current_yaw - hwt_dev.yaw_offset;

    if (final_yaw > 180.0f)  final_yaw -= 360.0f;
    if (final_yaw < -180.0f) final_yaw += 360.0f;

    out_data->euler.roll  = hwt_dev.raw.roll * K_ANGLE;
    out_data->euler.pitch = hwt_dev.raw.pitch * K_ANGLE;
    out_data->euler.yaw   = final_yaw;
    out_data->total_yaw   = out_data->euler.yaw;

    out_data->gyro_body.x = hwt_dev.raw.gyro_x * K_GYRO;
    out_data->gyro_body.y = hwt_dev.raw.gyro_y * K_GYRO;
    out_data->gyro_body.z = hwt_dev.raw.gyro_z * K_GYRO;

    out_data->acc_body.x  = hwt_dev.raw.acc_x * K_ACC;
    out_data->acc_body.y  = hwt_dev.raw.acc_y * K_ACC;
    out_data->acc_body.z  = hwt_dev.raw.acc_z * K_ACC;

    out_data->state = INS_STATE_READY;
}

// ================= Getter =================
static const Ins_driver_interface_t hwt606_drv = {
    .init         = HWT606_Init,
    .start_read   = HWT606_Start_Read,
    .wait_data    = HWT606_Wait_Data,
    .process_data = HWT606_Process
};

const Ins_driver_interface_t* HWT606_SPI_Get_Driver(SPI_HandleTypeDef *spi_handle) {
    hwt_dev.hspi = spi_handle;
    hwt_dev.is_ready = false;
    return &hwt606_drv;
}