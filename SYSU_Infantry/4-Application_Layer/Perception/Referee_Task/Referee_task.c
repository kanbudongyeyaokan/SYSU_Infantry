/**
 * @file    Referee_task.c
 * @brief   Referee communication and HUD task.
 */

#include "Referee_task.h"

#include "cmsis_os.h"
#include "referee.h"
#include "referee_hud_app.h"

extern UART_HandleTypeDef huart6;

void Referee_task(void const *argument)
{
    (void)argument;

    (void)Referee_Get_Data(&huart6);
    Referee_HUD_Reset();

    for (;;)
    {
        Referee_HUD_Update();
        osDelay(100);
    }
}
