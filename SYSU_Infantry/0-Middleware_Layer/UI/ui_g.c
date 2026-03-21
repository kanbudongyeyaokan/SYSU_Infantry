//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 2
#define TOTAL_STRING 3

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_Ungroup_NewRound->figure_type = 2;
    ui_g_Ungroup_NewRound->operate_type = 1;
    ui_g_Ungroup_NewRound->layer = 0;
    ui_g_Ungroup_NewRound->color = 0;
    ui_g_Ungroup_NewRound->start_x = 883;
    ui_g_Ungroup_NewRound->start_y = 567;
    ui_g_Ungroup_NewRound->width = 1;
    ui_g_Ungroup_NewRound->r = 79;

    ui_g_Ungroup_NewFloat->figure_type = 5;
    ui_g_Ungroup_NewFloat->operate_type = 1;
    ui_g_Ungroup_NewFloat->layer = 0;
    ui_g_Ungroup_NewFloat->color = 0;
    ui_g_Ungroup_NewFloat->start_x = 1398;
    ui_g_Ungroup_NewFloat->start_y = 716;
    ui_g_Ungroup_NewFloat->width = 2;
    ui_g_Ungroup_NewFloat->font_size = 20;
    ui_g_Ungroup_NewFloat->number = 12345;

    ui_g_Ungroup_mode1->figure_type = 7;
    ui_g_Ungroup_mode1->operate_type = 1;
    ui_g_Ungroup_mode1->layer = 0;
    ui_g_Ungroup_mode1->color = 0;
    ui_g_Ungroup_mode1->start_x = 768;
    ui_g_Ungroup_mode1->start_y = 199;
    ui_g_Ungroup_mode1->width = 2;
    ui_g_Ungroup_mode1->font_size = 20;
    ui_g_Ungroup_mode1->str_length = 4;
    strcpy(ui_g_Ungroup_mode1->string, "Text");

    ui_g_Ungroup_mode2->figure_type = 7;
    ui_g_Ungroup_mode2->operate_type = 1;
    ui_g_Ungroup_mode2->layer = 0;
    ui_g_Ungroup_mode2->color = 0;
    ui_g_Ungroup_mode2->start_x = 1109;
    ui_g_Ungroup_mode2->start_y = 187;
    ui_g_Ungroup_mode2->width = 2;
    ui_g_Ungroup_mode2->font_size = 20;
    ui_g_Ungroup_mode2->str_length = 4;
    strcpy(ui_g_Ungroup_mode2->string, "Text");

    ui_g_Ungroup_mode3->figure_type = 7;
    ui_g_Ungroup_mode3->operate_type = 1;
    ui_g_Ungroup_mode3->layer = 0;
    ui_g_Ungroup_mode3->color = 0;
    ui_g_Ungroup_mode3->start_x = 693;
    ui_g_Ungroup_mode3->start_y = 828;
    ui_g_Ungroup_mode3->width = 10;
    ui_g_Ungroup_mode3->font_size = 100;
    ui_g_Ungroup_mode3->str_length = 4;
    strcpy(ui_g_Ungroup_mode3->string, "Text");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
