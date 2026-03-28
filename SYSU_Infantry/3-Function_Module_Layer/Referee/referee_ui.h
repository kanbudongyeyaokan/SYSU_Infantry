#ifndef _REFEREE_UI_H
#define _REFEREE_UI_H

#include "referee.h"

typedef enum {
    UI_Graph_Line = 0,
    UI_Graph_Rectangle = 1,
    UI_Graph_Circle = 2,
    UI_Graph_Ellipse = 3,
    UI_Graph_Arc = 4,
    UI_Graph_Float = 5,
    UI_Graph_Int = 6,
    UI_Graph_Char = 7,
} UI_Graph_Type_e;

typedef enum {
    UI_Graph_ADD = 1,
    UI_Graph_Change = 2,
    UI_Graph_Del = 3,
} UI_Graph_Operate_e;

typedef enum {
    UI_Data_Del_NoOperate = 0,
    UI_Data_Del_Layer = 1,
    UI_Data_Del_ALL = 2,
} UI_Delete_Operate_e;

typedef enum {
    UI_Color_Main = 0,
    UI_Color_Yellow = 1,
    UI_Color_Green = 2,
    UI_Color_Orange = 3,
    UI_Color_Purplish_red = 4,
    UI_Color_Pink = 5,
    UI_Color_Cyan = 6,
    UI_Color_Black = 7,
    UI_Color_White = 8,
} UI_Graph_Color_e;

typedef graphic_data_struct_t Graph_Data_t;

#pragma pack(push, 1)
typedef struct {
    Graph_Data_t Graph_Control;
    uint8_t show_Data[30];
} String_Data_t;
#pragma pack(pop)

typedef struct {
    uint8_t Robot_Color;
    uint16_t Robot_ID;
    uint16_t Cilent_ID;
    uint16_t Receiver_Robot_ID;
} referee_id_t;

void UIDelete(referee_id_t *id, uint8_t delete_operate, uint8_t delete_layer);

void UILineDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                uint32_t end_x, uint32_t end_y);

void UIRectangleDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                     uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                     uint32_t end_x, uint32_t end_y);

void UICircleDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                  uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                  uint32_t graph_radius);

void UIOvalDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                uint32_t end_x, uint32_t end_y);

void UIArcDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
               uint32_t graph_color, uint32_t graph_start_angle, uint32_t graph_end_angle,
               uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

void UIFloatDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                 uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width,
                 uint32_t start_x, uint32_t start_y, int32_t graph_float);

void UIIntDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
               uint32_t graph_color, uint32_t graph_size, uint32_t graph_width,
               uint32_t start_x, uint32_t start_y, int32_t graph_integer);

void UICharDraw(String_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_size, uint32_t graph_width,
                uint32_t start_x, uint32_t start_y, const char *fmt, ...);

void UIGraphRefresh(referee_id_t *id, int cnt, ...);
void UICharRefresh(referee_id_t *id, String_Data_t string_data);

#endif // _REFEREE_UI_H
