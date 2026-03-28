/**
 * @file    Referee_task.c
 * @brief   Referee communication task.
 */

#include "Referee_task.h"

#include "cmsis_os.h"
#include "referee.h"

#define REFEREE_UI_TEST_BFDCB62_REPRO 1u

#if !REFEREE_UI_TEST_BFDCB62_REPRO
#include "referee_hud_app.h"
#endif

extern UART_HandleTypeDef huart6;

#if REFEREE_UI_TEST_BFDCB62_REPRO
static Referee_Data_t *referee_data;
#endif


void Referee_task(void const *argument)
{
    (void)argument;

#if REFEREE_UI_TEST_BFDCB62_REPRO
    // Reproduce commit bfdcb62: receive referee data on UART6, then keep sending one line UI test.
    referee_data = Referee_Get_Data(&huart6);
    (void)referee_data;

    osDelay(500);

    for (;;)
    {
        Referee_Send_UI_Test();
        osDelay(200);
    }
#else
    (void)Referee_Get_Data(&huart6);
    Referee_HUD_Reset();

    for (;;)
    {
        Referee_HUD_Update();
        osDelay(100);
    }
#endif
}
