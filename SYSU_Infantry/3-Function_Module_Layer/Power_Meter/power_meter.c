/**
 * @file    power_meter.c
 * @brief   底盘功率计数据解析模块实现
 * @author  SYSU电控组
 * @date    2025-03-22
 * @version 1.0
 */

#include "power_meter.h"
#include "bsp_can.h"
#include "error_handler.h"

#define POWER_METER_MODULE "POWER_METER"

/* 缩放系数：原始值 = 实际值 × 100 */
#define POWER_METER_SCALE_FACTOR  100.0f

/* 内部静态实例 */
static PowerMeter_t g_power_meter = {0.0f, 0.0f, 0.0f};
static Can_controller_t *g_power_meter_can = NULL;

/**
 * @brief CAN 接收回调函数
 * @note  在 CAN 中断中调用，解析功率计数据
 */
static void PowerMeter_RxCallback(Can_controller_t *can_dev, void *context)
{
    (void)context;
    
    if (can_dev == NULL)
    {
        ERROR_CRITICAL(POWER_METER_MODULE, "CAN device is NULL in RX callback");
        return;
    }
    
    PowerMeter_Parse(&g_power_meter, can_dev->rx_buffer);
}

void PowerMeter_Init(CAN_HandleTypeDef *hcan)
{
    Can_init_t can_cfg = {
        .can_handle = hcan,
        .can_id = 0,
        .tx_id = 0,
        .rx_id = POWER_METER_RX_ID,
        .context = NULL,
        .receive_callback = PowerMeter_RxCallback,
    };
    
    g_power_meter_can = Can_device_init(&can_cfg);
    
    if (g_power_meter_can == NULL)
    {
        ERROR_CRITICAL(POWER_METER_MODULE, "Power meter CAN init failed");
    }
}

void PowerMeter_Parse(PowerMeter_t *pm, const uint8_t *can_rx_data)
{
    if (pm == NULL || can_rx_data == NULL)
    {
        return;
    }
    
    /* 小端序：低字节在前，高字节在后 */
    /* DATA[0]: 电压低8位, DATA[1]: 电压高8位 */
    uint16_t voltage_raw = ((uint16_t)can_rx_data[1] << 8) | can_rx_data[0];
    
    /* DATA[2]: 电流低8位, DATA[3]: 电流高8位 */
    uint16_t current_raw = ((uint16_t)can_rx_data[3] << 8) | can_rx_data[2];
    
    /* 还原真实物理量 (除以缩放系数 100) */
    pm->real_voltage = (float)voltage_raw / POWER_METER_SCALE_FACTOR;
    pm->real_current = (float)current_raw / POWER_METER_SCALE_FACTOR;
    
    /* 计算实时功率 */
    pm->real_power = pm->real_voltage * pm->real_current;
}

PowerMeter_t* PowerMeter_GetData(void)
{
    return &g_power_meter;
}

float PowerMeter_GetVoltage(void)
{
    return g_power_meter.real_voltage;
}

float PowerMeter_GetCurrent(void)
{
    return g_power_meter.real_current;
}

float PowerMeter_GetPower(void)
{
    return g_power_meter.real_power;
}