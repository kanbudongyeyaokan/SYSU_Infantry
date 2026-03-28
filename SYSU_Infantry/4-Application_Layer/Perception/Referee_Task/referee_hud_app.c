#include "referee_hud_app.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "decision_making.h"
#include "queue.h"
#include "referee.h"
#include "referee_ui.h"
#include "robot_task.h"

static Graph_Data_t ui_shoot_line[10];
static Graph_Data_t ui_enter_line[2];
static Graph_Data_t ui_rotate[2];
static String_Data_t ui_state_static[10];
static String_Data_t ui_state_dynamic[6];

static bool ui_initialized = false;
static bool rotate_arc_visible = false;
static uint16_t rotate_arc_start_angle = 75u;
static uint8_t rotate_arc_index = 0u;
static chassis_mode_e last_chassis_mode = (chassis_mode_e)0xFF;

static void Referee_HUD_DrawCrosshair(void)
{
    UILineDraw(&ui_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 710, 452, 1210, 450);
    UILineDraw(&ui_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, 952, 280, 952, 680);
    UIGraphRefresh(NULL, 2, ui_shoot_line[0], ui_shoot_line[1]);

    UICharDraw(&ui_state_static[5], "ss5", UI_Graph_ADD, 7, UI_Color_Yellow, 15, 2, 1225, 458, "<3m");
    UICharRefresh(NULL, ui_state_static[5]);
    UICharDraw(&ui_state_static[6], "ss6", UI_Graph_ADD, 7, UI_Color_Yellow, 15, 2, 1160, 421, "4.5m");
    UICharRefresh(NULL, ui_state_static[6]);
    UICharDraw(&ui_state_static[8], "ss8", UI_Graph_ADD, 7, UI_Color_Yellow, 15, 2, 1160, 381, "6m");
    UICharRefresh(NULL, ui_state_static[8]);

    UILineDraw(&ui_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 780, 415, 1140, 415);
    UILineDraw(&ui_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 780, 373, 1140, 373);
    UIGraphRefresh(NULL, 2, ui_shoot_line[6], ui_shoot_line[7]);
}

static void Referee_HUD_DrawEnterLines(void)
{
    UILineDraw(&ui_enter_line[0], "el0", UI_Graph_ADD, 7, UI_Color_Purplish_red, 2, 626, 113, 820, 475);
    UILineDraw(&ui_enter_line[1], "el1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 2, 1290, 113, 1104, 475);
    UIGraphRefresh(NULL, 2, ui_enter_line[0], ui_enter_line[1]);
}

static void Referee_HUD_DrawStatusLabels(void)
{
    UICharDraw(&ui_state_static[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 18, 2, 50, 650, "CHASS:");
    UICharRefresh(NULL, ui_state_static[0]);

    UICharDraw(&ui_state_static[2], "ss2", UI_Graph_ADD, 7, UI_Color_Main, 18, 2, 50, 600, "Shoot:");
    UICharRefresh(NULL, ui_state_static[2]);

    UICharDraw(&ui_state_static[3], "ss3", UI_Graph_ADD, 7, UI_Color_Main, 18, 2, 50, 550, "Loader:");
    UICharRefresh(NULL, ui_state_static[3]);
}

static void Referee_HUD_DrawInitialDynamicState(void)
{
    UICharDraw(&ui_state_dynamic[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 18, 2, 70, 650, "zeroforce");
    UICharRefresh(NULL, ui_state_dynamic[0]);

    UICharDraw(&ui_state_dynamic[2], "sd2", UI_Graph_ADD, 8, UI_Color_Main, 18, 2, 70, 600, "OFF");
    UICharRefresh(NULL, ui_state_dynamic[2]);

    UICharDraw(&ui_state_dynamic[3], "sd3", UI_Graph_ADD, 8, UI_Color_Main, 18, 2, 70, 550, "INIT");
    UICharRefresh(NULL, ui_state_dynamic[3]);
}

static void Referee_HUD_Init(void)
{
    if (!Referee_Is_Online() || Get_Robot_ID() == 0u) {
        return;
    }

    UIDelete(NULL, UI_Data_Del_ALL, 0);

    Referee_HUD_DrawCrosshair();
    Referee_HUD_DrawEnterLines();
    Referee_HUD_DrawStatusLabels();
    Referee_HUD_DrawInitialDynamicState();

    rotate_arc_start_angle = 75u;
    rotate_arc_index = 0u;
    rotate_arc_visible = false;
    last_chassis_mode = (chassis_mode_e)0xFF;
    ui_initialized = true;
}

static void Referee_HUD_UpdateShootState(const Shoot_cmd_send_t *shoot_cmd)
{
    const char *shoot_text = "OFF";
    const char *loader_text = "INIT";

    if (shoot_cmd != NULL && shoot_cmd->shoot_mode == SHOOT_ON) {
        shoot_text = "ON ";
    }

    if (shoot_cmd != NULL) {
        switch (shoot_cmd->loader_mode) {
            case LOAD_STOP:
                loader_text = "STOP    ";
                break;
            case LOAD_1_BULLET:
                loader_text = "1_BULLET";
                break;
            case LOAD_BURSTFIRE:
                loader_text = "BURST   ";
                break;
            case LOAD_REVERSE:
                loader_text = "REVERSE ";
                break;
            default:
                loader_text = "INIT";
                break;
        }
    }

    UICharDraw(&ui_state_dynamic[2], "sd2", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 170, 600, "%s", shoot_text);
    UICharRefresh(NULL, ui_state_dynamic[2]);

    UICharDraw(&ui_state_dynamic[3], "sd3", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 170, 550, "%s", loader_text);
    UICharRefresh(NULL, ui_state_dynamic[3]);
}

static void Referee_HUD_UpdateChassisState(const Chassis_cmd_send_t *chassis_cmd)
{
    const char *chassis_text = "zeroforce";

    if (chassis_cmd != NULL) {
        switch (chassis_cmd->chassis_mode) {
            case CHASSIS_ROTATE:
                chassis_text = "rotate   ";
                break;
            case CHASSIS_NO_FOLLOW:
                chassis_text = "nofollow ";
                break;
            case CHASSIS_FOLLOW_GIMBAL:
                chassis_text = "follow   ";
                break;
            case CHASSIS_ZERO_FORCE:
            default:
                chassis_text = "zeroforce";
                break;
        }
    }

    UICharDraw(&ui_state_dynamic[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 170, 650, "%s", chassis_text);
    UICharRefresh(NULL, ui_state_dynamic[0]);
}

static void Referee_HUD_UpdateRotateIndicator(const Chassis_cmd_send_t *chassis_cmd)
{
    chassis_mode_e current_mode = CHASSIS_ZERO_FORCE;

    if (chassis_cmd != NULL) {
        current_mode = chassis_cmd->chassis_mode;
    }

    if (current_mode == CHASSIS_ROTATE) {
        if (last_chassis_mode != CHASSIS_ROTATE) {
            rotate_arc_start_angle = 75u;
            rotate_arc_index = 0u;
        } else {
            rotate_arc_index = (uint8_t)((rotate_arc_index + 1u) % 8u);
            rotate_arc_start_angle = (uint16_t)(75u + rotate_arc_index * 45u);
        }

        UIArcDraw(&ui_rotate[1], "rt1", UI_Graph_Change, 6, UI_Color_Cyan,
                  rotate_arc_start_angle, rotate_arc_start_angle + 35u, 12, 960, 540, 300, 300);

        if (!rotate_arc_visible) {
            UICircleDraw(&ui_rotate[0], "rt0", UI_Graph_ADD, 6, UI_Color_Cyan, 2, 960, 540, 300);
            UIArcDraw(&ui_rotate[1], "rt1", UI_Graph_ADD, 6, UI_Color_Cyan,
                      rotate_arc_start_angle, rotate_arc_start_angle + 35u, 12, 960, 540, 300, 300);
            rotate_arc_visible = true;
        }

        UIGraphRefresh(NULL, 2, ui_rotate[0], ui_rotate[1]);
    } else if (rotate_arc_visible) {
        UICircleDraw(&ui_rotate[0], "rt0", UI_Graph_Del, 6, UI_Color_Cyan, 2, 960, 540, 300);
        UIArcDraw(&ui_rotate[1], "rt1", UI_Graph_Del, 6, UI_Color_Cyan,
                  rotate_arc_start_angle, rotate_arc_start_angle + 35u, 12, 960, 540, 300, 300);
        UIGraphRefresh(NULL, 2, ui_rotate[0], ui_rotate[1]);
        rotate_arc_visible = false;
    }

    last_chassis_mode = current_mode;
}

void Referee_HUD_Reset(void)
{
    memset(ui_shoot_line, 0, sizeof(ui_shoot_line));
    memset(ui_enter_line, 0, sizeof(ui_enter_line));
    memset(ui_rotate, 0, sizeof(ui_rotate));
    memset(ui_state_static, 0, sizeof(ui_state_static));
    memset(ui_state_dynamic, 0, sizeof(ui_state_dynamic));

    ui_initialized = false;
    rotate_arc_visible = false;
    rotate_arc_start_angle = 75u;
    rotate_arc_index = 0u;
    last_chassis_mode = (chassis_mode_e)0xFF;
}

void Referee_HUD_Update(void)
{
    Chassis_cmd_send_t chassis_cmd;
    Shoot_cmd_send_t shoot_cmd;

    if (!Referee_Is_Online() || Get_Robot_ID() == 0u) {
        return;
    }

    if (!ui_initialized) {
        Referee_HUD_Init();
    }

    if (!ui_initialized) {
        return;
    }

    if (xQueuePeek(Chassis_cmd_queue_handle, &chassis_cmd, 0) != pdTRUE) {
        return;
    }

    memset(&shoot_cmd, 0, sizeof(shoot_cmd));
    (void)xQueuePeek(Shoot_cmd_queue_handle, &shoot_cmd, 0);

    Referee_HUD_UpdateShootState(&shoot_cmd);
    Referee_HUD_UpdateChassisState(&chassis_cmd);
    Referee_HUD_UpdateRotateIndicator(&chassis_cmd);
}
