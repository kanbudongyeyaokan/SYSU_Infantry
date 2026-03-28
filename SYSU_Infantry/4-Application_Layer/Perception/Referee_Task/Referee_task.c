/**
 * @file    Referee_task.c
 * @brief   Referee UI task.
 */

#include "Referee_task.h"

#include <stdlib.h>

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"

extern UART_HandleTypeDef huart6;

static Referee_Data_t *ref_data_ptr;

static void Draw_Tactical_HUD(void)
{
    Graph_Data_t static_figs[5];
    String_Data_t cap_text;

    UILineDraw(&static_figs[0], "CHH", UI_Graph_ADD, 1, UI_Color_Green, 2, 920, 540, 1000, 540);
    UILineDraw(&static_figs[1], "CHV", UI_Graph_ADD, 1, UI_Color_Green, 2, 960, 500, 960, 580);
    UICircleDraw(&static_figs[2], "CHC", UI_Graph_ADD, 1, UI_Color_Cyan, 4, 960, 540, 5);
    UILineDraw(&static_figs[3], "CBB", UI_Graph_ADD, 2, UI_Color_White, 10, 700, 200, 900, 200);
    UILineDraw(&static_figs[4], "CBF", UI_Graph_ADD, 3, UI_Color_Orange, 10, 700, 200, 700, 200);

    UIGraphRefresh(NULL, 5, static_figs[0], static_figs[1], static_figs[2], static_figs[3], static_figs[4]);

    UICharDraw(&cap_text, "TXT", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 2, 630, 215, "CAP:");
    UICharRefresh(NULL, cap_text);
}

static void Update_Dynamic_Capacitor(void)
{
    static uint16_t last_buffer = 999u;
    uint16_t current_buffer = ref_data_ptr->power_heat_data.buffer_energy;

    if (abs((int)current_buffer - (int)last_buffer) >= 1)
    {
        Graph_Data_t cap_fg;
        uint32_t end_x = 700u + (uint32_t)(current_buffer * 3.3f);
        uint8_t bar_color = (current_buffer < 20u) ? UI_Color_Pink : UI_Color_Orange;

        if (end_x > 900u) {
            end_x = 900u;
        }

        UILineDraw(&cap_fg, "CBF", UI_Graph_Change, 3, bar_color, 10, 700, 200, end_x, 200);
        UIGraphRefresh(NULL, 1, cap_fg);

        last_buffer = current_buffer;
    }
}

void Referee_task(void const *argument)
{
    uint32_t time_tick = 0u;

    (void)argument;

    ref_data_ptr = Referee_Get_Data(&huart6);
    osDelay(2000);

    UIDelete(NULL, UI_Data_Del_ALL, 0);
    osDelay(200);

    for (;;)
    {
        if ((time_tick % 20u) == 0u)
        {
            Draw_Tactical_HUD();
        }

        Update_Dynamic_Capacitor();

        time_tick++;
        osDelay(100);
    }
}
