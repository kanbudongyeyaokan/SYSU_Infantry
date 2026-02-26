#include "supercap_comm.h"
#include "referee.h"
#include "bsp_can.h"
#include <string.h>

static Can_controller_t *supercap_can = NULL;

static SuperCap_RxData supercap_rx_data;
static SuperCap_TxData supercap_tx_data;

static void SuperCap_RxCallback(Can_controller_t *can_dev, void *context)
{
    (void) context;
    if (can_dev == NULL)
    {
        return;
    }

    memcpy((void *) &supercap_rx_data, can_dev->rx_buffer, sizeof(supercap_rx_data));
}



void SuperCap_Comm_Init_Ext(CAN_HandleTypeDef *hcan, uint32_t tx_id, uint32_t rx_id)
{
    Can_init_t can_cfg = {
        .can_handle = hcan,
        .can_id = tx_id,
        .tx_id = 0,
        .rx_id = rx_id,
        .context = NULL,
        .receive_callback = SuperCap_RxCallback,
    };

    supercap_can = Can_device_init(&can_cfg);
}


void SuperCap_Comm_Init(CAN_HandleTypeDef *hcan)
{
    //使用默认 ID：TX = 0x061, RX = 0x051
    SuperCap_Comm_Init_Ext(hcan, SUPER_CAP_TX_ID, SUPER_CAP_RX_ID);
}

uint8_t SuperCap_Comm_Send(const SuperCap_TxData *data)
{
    if (supercap_can == NULL)
    {
        return 0U;
    }

    if (data != NULL)
    {
        memcpy((void *) &supercap_tx_data, data, sizeof(supercap_tx_data));
    }

    return Can_send_data(supercap_can, (uint8_t *) &supercap_tx_data);
}


void SuperCap_Comm_GetRxData(SuperCap_RxData *out)
{
    if (out == NULL)
    {
        return;
    }

    memcpy(out, (const void *) &supercap_rx_data, sizeof(supercap_rx_data));
}

uint8_t SuperCap_Get_Cap_Energy(void)
{
    return supercap_rx_data.cap_energy;
}

