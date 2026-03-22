#ifndef SUPER_CAP_COMM_H
#define SUPER_CAP_COMM_H

#include "stdint.h"
#include "can.h"



#define SUPER_CAP_TX_ID 0x061u
#define SUPER_CAP_RX_ID 0x051u

typedef struct __attribute__((packed))
{
    uint8_t enable_dcdc : 1;
    uint8_t system_restart : 1;
    uint8_t resv0 : 6;
    uint16_t feedback_referee_power_limit;
    uint16_t feedback_referee_energy_buffer;
    uint8_t resv1[3];
} SuperCap_TxData;

typedef struct __attribute__((packed))
{
    uint8_t error_code;
    float chassis_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
} SuperCap_RxData;

void SuperCap_Comm_Init(CAN_HandleTypeDef *hcan);
uint8_t SuperCap_Comm_Send(const SuperCap_TxData *data);
void SuperCap_Comm_GetRxData(SuperCap_RxData *out);
uint8_t SuperCap_Get_Cap_Energy(void);
uint16_t SuperCap_Get_Power_Limit(void);
float SuperCap_Get_Chassis_Power(void);
void Send2SuperCap(void);



#endif // SUPER_CAP_COMM_H
