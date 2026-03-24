//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[2];
extern uint8_t ui_g_dirty_figure[2];
extern ui_interface_string_t ui_g_now_strings[3];
extern uint8_t ui_g_dirty_string[3];

extern uint8_t ui_g_max_send_count[5];

#define ui_g_Ungroup_NewRound ((ui_interface_round_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_NewFloat ((ui_interface_number_t*)&(ui_g_now_figures[1]))

#define ui_g_Ungroup_mode1 (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_mode2 (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_mode3 (&(ui_g_now_strings[2]))

#define ui_g_Ungroup_NewRound_max_send_count (ui_g_max_send_count[0])
#define ui_g_Ungroup_NewFloat_max_send_count (ui_g_max_send_count[1])

#define ui_g_Ungroup_mode1_max_send_count (ui_g_max_send_count[2])
#define ui_g_Ungroup_mode2_max_send_count (ui_g_max_send_count[3])
#define ui_g_Ungroup_mode3_max_send_count (ui_g_max_send_count[4])

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_NewRound_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_NewFloat_dirty (ui_g_dirty_figure[1])

#define ui_g_Ungroup_mode1_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_mode2_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_mode3_dirty (ui_g_dirty_string[2])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
