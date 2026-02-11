#ifndef SUPER_CAP_COMM_H
#define SUPER_CAP_COMM_H

#include "stdint.h"
#include "can.h"



#define SUPER_CAP_TX_ID 0x061u
#define SUPER_CAP_RX_ID 0x051u

#pragma pack(1)
typedef struct
{
    uint8_t enable_dcdc : 1;
    uint8_t system_restart : 1;
    uint8_t resv0 : 6;
    uint16_t feedback_referee_power_limit;
    uint16_t feedback_referee_energy_buffer;
    uint8_t resv1[3];
} SuperCap_TxData;
#pragma pack()

#pragma pack(1)
typedef struct __attribute__((packed))
{
    uint8_t error_code;
    float chassis_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
} SuperCap_RxData;
#pragma pack()

void SuperCap_Comm_Init(CAN_HandleTypeDef *hcan);
uint8_t SuperCap_Comm_Send(const SuperCap_TxData *data);
void SuperCap_Comm_GetRxData(SuperCap_RxData *out);
uint8_t SuperCap_Get_Cap_Energy(void);



#endif // SUPER_CAP_COMM_H
