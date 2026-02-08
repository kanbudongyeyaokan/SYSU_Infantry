/**
 * @file    hwt101_iic.c
 * @brief   HWT101 驱动实现 (仅读取 Yaw 和 GyroZ)
 */

#include "hwt101_iic.h"
#include <string.h>
#include "bsp_dwt.h" 

// ================= 寄存器定义 =================
// 仅保留需要的两个寄存器地址
#define HWT101_REG_GYRO_Z   0x39 // 角速度 Z,单位是度/秒,范围±2000dps,对应±32768原始值
#define HWT101_REG_YAW      0x3F // 航向角 Z，单位是度，范围±180度，对应±32768原始值

// ================= 私有对象结构体 =================
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    // 原始数据缓存 (只存需要的)
    struct {
        int16_t gyro_z;
        int16_t yaw;
    } raw;

    bool is_ready;
    bool read_success;

    
} HWT101_Driver_t;

static HWT101_Driver_t hwt_dev;

// ================= 内部辅助函数 =================

static HAL_StatusTypeDef HWT101_ReadReg16(uint16_t reg, int16_t *val) {
    return HAL_I2C_Mem_Read(hwt_dev.hi2c, hwt_dev.dev_addr, reg, 
                            I2C_MEMADD_SIZE_8BIT, (uint8_t*)val, 2, 100);
}

static void HWT101_Reset_I2C(void) {
    HAL_I2C_DeInit(hwt_dev.hi2c);
    HAL_I2C_Init(hwt_dev.hi2c);
}

// ================= 接口实现 =================

static bool HWT101_Init(void) {
    if (hwt_dev.hi2c == NULL) return false;
    
    // 检查总线状态
    if (HAL_I2C_GetState(hwt_dev.hi2c) != HAL_I2C_STATE_READY) {
        HWT101_Reset_I2C();
    }
    
    // 试读 Yaw 寄存器验证通信
    int16_t dummy;
    if (HWT101_ReadReg16(HWT101_REG_YAW, &dummy) != HAL_OK) {
        return false;
    }
    
    hwt_dev.is_ready = true;
    return true;
}

static void HWT101_Start_Read(void) {
    if (!hwt_dev.is_ready) return;
    
    // 假设读取成功，遇到错误则置 false
    bool all_ok = true;

    if (HAL_I2C_GetState(hwt_dev.hi2c) != HAL_I2C_STATE_READY) {
        HWT101_Reset_I2C();
        return;
    }

    // 读取 航向角 (Yaw)
    if (HWT101_ReadReg16(HWT101_REG_YAW, &hwt_dev.raw.yaw) != HAL_OK) {
        all_ok = false;
    }
    
    // 加微小延时，防止传感器来不及准备数据导致下一次读取 NACK
    HAL_Delay(1); // 1毫秒延时，足够让传感器准备好数据
    // DWT_Delay_us(100); // 100微秒延时，足够让传感器准备好数据
    // 读取 角速度 (Gyro Z)
    if (HWT101_ReadReg16(HWT101_REG_GYRO_Z, &hwt_dev.raw.gyro_z) != HAL_OK) {
        all_ok = false;
    }
    
    hwt_dev.read_success = all_ok;
}

static bool HWT101_Wait_Data(void) {
    return hwt_dev.read_success;
}

static void HWT101_Process(Ins_data_t *out_data, float dt_s) {
    // 角度转换系数: Raw / 32768 * 180 度
    const float K_ANGLE = 180.0f / 32768.0f;
    // 角速度转换系数: Raw / 32768 * 2000 度/秒
    const float K_GYRO = 2000.0f / 32768.0f;

    out_data->euler.yaw = hwt_dev.raw.yaw * K_ANGLE;
    out_data->total_yaw = out_data->euler.yaw; 
    out_data->gyro_body.z = hwt_dev.raw.gyro_z * K_GYRO; 

    // 将单轴的其他数据置零，防止未初始化数据的干扰
    out_data->euler.roll = 0.0f;
    out_data->euler.pitch = 0.0f;
    out_data->gyro_body.x = 0.0f;
    out_data->gyro_body.y = 0.0f;
    out_data->acc_body.x = 0.0f;
    out_data->acc_body.y = 0.0f;
    out_data->acc_body.z = 0.0f;

    out_data->state = INS_STATE_READY;
}

// ================= Getter =================
static const Ins_driver_interface_t hwt101_drv = {
    .init         = HWT101_Init,
    .start_read   = HWT101_Start_Read,
    .wait_data    = HWT101_Wait_Data,
    .process_data = HWT101_Process
};

const Ins_driver_interface_t* HWT101_Get_Driver(I2C_HandleTypeDef *i2c_handle) {
    hwt_dev.hi2c = i2c_handle;
    hwt_dev.dev_addr = 0xA0; // 0x50 << 1
    hwt_dev.is_ready = false;
    return &hwt101_drv;
}