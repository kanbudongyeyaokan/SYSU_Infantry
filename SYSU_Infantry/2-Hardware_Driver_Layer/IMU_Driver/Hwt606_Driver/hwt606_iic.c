/**
 * @file    hwt606_iic.c
 * @brief   HWT606 驱动实现 (18字节连续读取 + DMA + 轴向修正)
 */

#include "hwt606_iic.h"
#include <string.h>

// ================= 寄存器定义 =================
// 从角速度(0x37)开始，一次读到角度(0x3F)
// 顺序: Gyro(6) + Mag(6/跳过) + Angle(6)
#define REG_READ_START      0x37 
#define READ_LEN            18   

// ================= 私有对象结构体 =================
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    // 原始数据缓存 (经过解析后的 int16)
    struct {
        int16_t gyro[3];  // X, Y, Z
        int16_t angle[3]; // Roll, Pitch, Yaw
    } raw;

    bool is_ready;
    
    // 标志位：volatile 防止编译器优化中断变量
    volatile bool read_success; 
} HWT606_Driver_t;

static HWT606_Driver_t hwt606_dev;

// DMA 专用全局缓冲区 (防止栈内存释放导致 DMA 搬运错误)
static uint8_t hwt606_dma_buf[READ_LEN];

// ================= 内部辅助函数 =================

static void HWT606_Reset_I2C(void) {
    HAL_I2C_DeInit(hwt606_dev.hi2c);
    HAL_I2C_Init(hwt606_dev.hi2c);
}

// ================= 接口实现 =================

static bool HWT606_Init(void) {
    if (hwt606_dev.hi2c == NULL) return false;
    
    // 复位总线防死锁
    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY) {
        HWT606_Reset_I2C();
    }
    
    // 检查设备在线 (阻塞式检查一次即可)
    if (HAL_I2C_IsDeviceReady(hwt606_dev.hi2c, hwt606_dev.dev_addr, 3, 100) != HAL_OK) {
        return false;
    }
    
    hwt606_dev.is_ready = true;
    return true;
}

// 启动 DMA 读取
static void HWT606_Start_Read(void) {
    if (!hwt606_dev.is_ready) return;

    // 清除成功标志
    hwt606_dev.read_success = false;

    // 检查 I2C 状态，如果正忙则跳过本次读取
    if (HAL_I2C_GetState(hwt606_dev.hi2c) != HAL_I2C_STATE_READY) {
        // 视情况可添加复位逻辑
        return;
    }

    // 启动 DMA 传输 -> 数据直接存入全局数组 hwt606_dma_buf
    if (HAL_I2C_Mem_Read_DMA(hwt606_dev.hi2c, hwt606_dev.dev_addr, REG_READ_START, 
                             I2C_MEMADD_SIZE_8BIT, hwt606_dma_buf, READ_LEN) != HAL_OK) 
    {
        // 启动失败，这里可以加错误计数，目前测试发现能用
    }
}

// 接收完成回调 (被 bsp_iic.c 调用)
void HWT606_RxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (hwt606_dev.hi2c != NULL && hi2c == hwt606_dev.hi2c) {
        hwt606_dev.read_success = true;
    }
}

// 等待数据 (检查标志位)
static bool HWT606_Wait_Data(void) {
    uint32_t start_tick = HAL_GetTick();
    while (hwt606_dev.read_success == false) {
        // 超时退出 (1ms)
        if (HAL_GetTick() - start_tick > 1) {
            return false;
        }
    }
    return true;
}

// 解析数据
static void HWT606_Process(Ins_data_t *out_data, float dt_s) {
    // 引用全局 DMA 缓冲区
    uint8_t *buf = hwt606_dma_buf; 

    // --- 解析角速度 (前6字节) ---

    hwt606_dev.raw.gyro[1] = (int16_t)(buf[0] | (buf[1] << 8)); // 原 X -> 给 Y
    hwt606_dev.raw.gyro[0] = (int16_t)(buf[2] | (buf[3] << 8)); // 原 Y -> 给 X
    hwt606_dev.raw.gyro[2] = (int16_t)(buf[4] | (buf[5] << 8)); // Z 不变

    // --- 解析角度 (后6字节) ---
    hwt606_dev.raw.angle[1] = (int16_t)(buf[12] | (buf[13] << 8)); // 原 Roll -> 给 Pitch
    hwt606_dev.raw.angle[0] = (int16_t)(buf[14] | (buf[15] << 8)); // 原 Pitch -> 给 Roll
    hwt606_dev.raw.angle[2] = (int16_t)(buf[16] | (buf[17] << 8)); // Yaw 不变
    
    // --- 单位转换与填充 ---
    const float K_ANGLE = 180.0f / 32768.0f;  // 角度 (deg)
    const float K_GYRO  = 2000.0f / 32768.0f; // 角速度 (deg/s)

    // 填充角度
    out_data->euler.roll  = hwt606_dev.raw.angle[0] * K_ANGLE;
    out_data->euler.pitch = hwt606_dev.raw.angle[1] * K_ANGLE;
    out_data->euler.yaw   = hwt606_dev.raw.angle[2] * K_ANGLE;
    
    // 填充角速度 (单位: 度/秒)
    out_data->gyro_body.x = hwt606_dev.raw.gyro[0] * K_GYRO;
    out_data->gyro_body.y = hwt606_dev.raw.gyro[1] * K_GYRO;
    out_data->gyro_body.z = hwt606_dev.raw.gyro[2] * K_GYRO;

    // 清零加速度 
    out_data->acc_body.x = 0;
    out_data->acc_body.y = 0;
    out_data->acc_body.z = 0;

    out_data->total_yaw = out_data->euler.yaw;
    out_data->state = INS_STATE_READY;
}

// ================= Getter =================
static const Ins_driver_interface_t hwt606_drv = {
    .init         = HWT606_Init,
    .start_read   = HWT606_Start_Read,
    .wait_data    = HWT606_Wait_Data,
    .process_data = HWT606_Process
};

const Ins_driver_interface_t* HWT606_Get_Driver(I2C_HandleTypeDef *i2c_handle) {
    hwt606_dev.hi2c = i2c_handle;
    hwt606_dev.dev_addr = HWT606_IIC_ADDR; 
    hwt606_dev.is_ready = false;
    return &hwt606_drv;
}