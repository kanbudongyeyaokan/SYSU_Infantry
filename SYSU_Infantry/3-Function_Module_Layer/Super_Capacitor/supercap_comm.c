#include "supercap_comm.h"
#include "referee.h"
#include "bsp_can.h"
#include "error_handler.h"
#include "main.h"
#include <string.h>

#define SUPER_CAP_MODULE "SUPERCAP"

#define SUPER_CAP_TX_INTERVAL_MS  50U
#define SUPER_CAP_DEBUG_INTERVAL_MS 500U

static Can_controller_t *supercap_can = NULL;

static SuperCap_RxData supercap_rx_data;
static SuperCap_TxData supercap_tx_data;

static uint16_t g_tx_fail_cd = 0U;
static uint32_t g_last_tx_tick = 0U;
static uint32_t g_last_debug_tick = 0U;

static void SuperCap_RxCallback(Can_controller_t *can_dev, void *context)
{
    (void) context;
    if (can_dev == NULL)
    {
        ERROR_CRITICAL(SUPER_CAP_MODULE, "CAN device is NULL in RX callback");
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

    uint8_t ret = Can_send_data(supercap_can, (uint8_t *) &supercap_tx_data);
    if (ret == 0U)
    {
        if (g_tx_fail_cd == 0U)
        {
            ERROR_WARN(SUPER_CAP_MODULE, "CAN TX failed, mailbox full or error");
            g_tx_fail_cd = 100U;
        }
        else
        {
            g_tx_fail_cd--;
        }
    }
    else
    {
        g_tx_fail_cd = 0U;
    }

    return ret;
}


void SuperCap_Comm_GetRxData(SuperCap_RxData *out)
{
    if (out == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out, (const void *) &supercap_rx_data, sizeof(supercap_rx_data));
    __set_PRIMASK(primask);
}

uint8_t SuperCap_Get_Cap_Energy(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t energy = supercap_rx_data.cap_energy;
    __set_PRIMASK(primask);
    return energy;
}

uint16_t SuperCap_Get_Power_Limit(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t limit = supercap_rx_data.chassis_power_limit;
    __set_PRIMASK(primask);
    return limit;
}

float SuperCap_Get_Chassis_Power(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float power = supercap_rx_data.chassis_power;
    __set_PRIMASK(primask);
    return power;
}

void Send2SuperCap(void)
{
    uint32_t now = HAL_GetTick();

    if (now - g_last_debug_tick >= SUPER_CAP_DEBUG_INTERVAL_MS)
    {
        g_last_debug_tick = now;
        // ERROR_INFO(SUPER_CAP_MODULE,
        //            "Rx: err=0x%02X power=%.2fW limit=%uW cap=%u%%",
        //            supercap_rx_data.error_code,
        //            supercap_rx_data.chassis_power,
        //            supercap_rx_data.chassis_power_limit,
        //            supercap_rx_data.cap_energy);
    }

    if (now - g_last_tx_tick < SUPER_CAP_TX_INTERVAL_MS)
    {
        return;
    }
    g_last_tx_tick = now;

    SuperCap_TxData tx_data = {0};
    tx_data.enable_dcdc = 1U;
    tx_data.system_restart = 0U;
    tx_data.feedback_referee_power_limit = ChassisPower_GetMaxLimit();
    tx_data.feedback_referee_energy_buffer = ChassisPower_GetBuffer();

    SuperCap_Comm_Send(&tx_data);
}

