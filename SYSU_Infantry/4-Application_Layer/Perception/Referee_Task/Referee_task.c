/**
 * @file    Referee_task.c
 * @brief   裁判系统处理任务 (双频心跳版，完美抗丢包、防掉线)
 */

#include "Referee_task.h"
#include "referee.h"
#include "referee_ui.h"
#include "cmsis_os.h"  
#include "stdlib.h"
extern UART_HandleTypeDef huart6;
static Referee_Data_t* ref_data_ptr;

// =========================================================
// 战术 HUD 绘制函数 (静态元素)
// =========================================================
static void Draw_Tactical_HUD(void)
{
    graphic_data_struct_t static_figs[5];
    ui_string_t cap_text;

    UI_Pack_Line(&static_figs[0], "CHH", UI_OPERATE_ADD, 1, UI_COLOR_GREEN, 2, 920, 540, 1000, 540);
    UI_Pack_Line(&static_figs[1], "CHV", UI_OPERATE_ADD, 1, UI_COLOR_GREEN, 2, 960, 500, 960, 580);
    UI_Pack_Circle(&static_figs[2], "CHC", UI_OPERATE_ADD, 1, UI_COLOR_CYAN, 4, 960, 540, 5);
    
    // 电容条背景
    UI_Pack_Line(&static_figs[3], "CBB", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 10, 700, 200, 900, 200);
    // 电容条前景初始层
    UI_Pack_Line(&static_figs[4], "CBF", UI_OPERATE_ADD, 3, UI_COLOR_ORANGE, 10, 700, 200, 700, 200);

    // 发送 5 图组合包
    UI_Send_Multi_Figures(5, static_figs);
    osDelay(30); 

    UI_Pack_String(&cap_text, "TXT", UI_OPERATE_ADD, 2, UI_COLOR_YELLOW, 20, 4, 630, 215, "CAP:");
    UI_Send_String(&cap_text);
}

// =========================================================
// 动态电容条更新 (动态元素)
// =========================================================
static void Update_Dynamic_Capacitor(void)
{
    static uint16_t last_buffer = 999;
    uint16_t current_buffer = ref_data_ptr->power_heat_data.buffer_energy;

    if (abs(current_buffer - last_buffer) >= 1) 
    {
        graphic_data_struct_t cap_fg;
        
        uint32_t end_x = 700 + (uint32_t)(current_buffer * 3.3f);
        if (end_x > 900) end_x = 900; 

        uint8_t bar_color = (current_buffer < 20) ? UI_COLOR_PINK : UI_COLOR_ORANGE;

        // MODIFY 更新
        UI_Pack_Line(&cap_fg, "CBF", UI_OPERATE_MODIFY, 3, bar_color, 10, 700, 200, end_x, 200);
        UI_Send_Single_Figure(&cap_fg);

        last_buffer = current_buffer;
    }
}

// =========================================================
// 任务主体
// =========================================================
void Referee_task(void const * argument)
{
    ref_data_ptr = Referee_Get_Data(&huart6);
    osDelay(2000); 

    UI_Delete_All(); 
    osDelay(200);

    uint32_t time_tick = 0;

    for(;;)
    {
        // 慢循环：每 2000ms (2秒) 发送一次静态图层“心跳包”
    
        if (time_tick % 20 == 0) 
        {
            Draw_Tactical_HUD();
        }

        // 快循环：每 100ms 检查并刷新一次动态电容条，保证丝滑
        Update_Dynamic_Capacitor();

        time_tick++;
        osDelay(100); // 基础节拍 100ms (10Hz)
    }
}