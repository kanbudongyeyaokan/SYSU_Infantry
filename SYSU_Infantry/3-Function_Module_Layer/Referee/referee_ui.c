#include "referee_ui.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

extern Uart_instance_t *referee_uart;

#define ROBOT_RED                0u
#define ROBOT_BLUE               1u
#define UI_DATA_DEL_ID           0x0100u
#define UI_DATA_DRAW_1_ID        0x0101u
#define UI_DATA_DRAW_2_ID        0x0102u
#define UI_DATA_DRAW_5_ID        0x0103u
#define UI_DATA_DRAW_7_ID        0x0104u
#define UI_DATA_DRAW_CHAR_ID     0x0110u
#define UI_INTERACTIVE_HEADER_LEN 6u
#define UI_DELETE_DATA_LEN       2u
#define UI_SINGLE_GRAPH_LEN      15u
#define UI_STRING_DATA_LEN       (UI_SINGLE_GRAPH_LEN + 30u)
#define UI_MAX_GRAPH_COUNT       7u
#define UI_SEND_INTERVAL_MS      115u

#pragma pack(push, 1)
typedef struct {
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
} ui_graph_frame_header_t;

typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    uint8_t Delete_Operate;
    uint8_t Layer;
    uint16_t frametail;
} UI_delete_t;

typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    String_Data_t String_Data;
    uint16_t frametail;
} UI_CharReFresh_t;
#pragma pack(pop)

static uint8_t ui_seq = 0;

static void UI_Send_Buffer(uint8_t *send, uint16_t tx_len)
{
    if (referee_uart == NULL || send == NULL || tx_len == 0u) {
        return;
    }

    Uart_sendData(referee_uart, send, tx_len);
    osDelay(UI_SEND_INTERVAL_MS);
}

static void UI_Fill_Default_ID(referee_id_t *id)
{
    uint8_t robot_id;

    if (id == NULL) {
        return;
    }

    memset(id, 0, sizeof(*id));

    robot_id = Get_Robot_ID();
    if (robot_id == 0u) {
        return;
    }

    id->Robot_Color = (robot_id > 7u) ? ROBOT_BLUE : ROBOT_RED;
    id->Robot_ID = robot_id;
    id->Cilent_ID = (uint16_t)(0x0100u + robot_id);
    id->Receiver_Robot_ID = 0u;
}

static const referee_id_t *UI_Resolve_Referee_ID(referee_id_t *id, referee_id_t *local_id)
{
    if (id != NULL) {
        return id;
    }

    UI_Fill_Default_ID(local_id);
    return local_id;
}

static bool UI_Referee_ID_Is_Valid(const referee_id_t *id)
{
    return (referee_uart != NULL) && (id != NULL) && (id->Robot_ID != 0u) && (id->Cilent_ID != 0u);
}

static void UI_Copy_Graphic_Name(uint8_t dest[3], const char *name)
{
    uint8_t i;

    memset(dest, 0, 3);
    if (name == NULL) {
        return;
    }

    for (i = 0; i < 3u && name[i] != '\0'; i++) {
        dest[2u - i] = (uint8_t)name[i];
    }
}

static uint16_t UI_Get_Graph_Data_Cmd_ID(uint8_t count)
{
    switch (count) {
        case 1u:
            return UI_DATA_DRAW_1_ID;
        case 2u:
            return UI_DATA_DRAW_2_ID;
        case 5u:
            return UI_DATA_DRAW_5_ID;
        case 7u:
            return UI_DATA_DRAW_7_ID;
        default:
            return 0u;
    }
}

static void UI_Send_GraphArray(const referee_id_t *id, uint8_t count, const Graph_Data_t *graphs)
{
    ui_graph_frame_header_t header;
    uint8_t buffer[REF_HEADER_LEN + REF_CMD_LEN + UI_INTERACTIVE_HEADER_LEN +
                   UI_SINGLE_GRAPH_LEN * UI_MAX_GRAPH_COUNT + REF_CRC16_LEN];
    uint16_t data_cmd_id = UI_Get_Graph_Data_Cmd_ID(count);
    uint16_t payload_len;
    uint16_t total_len;

    if (!UI_Referee_ID_Is_Valid(id) || graphs == NULL || data_cmd_id == 0u) {
        return;
    }

    payload_len = (uint16_t)(UI_INTERACTIVE_HEADER_LEN + UI_SINGLE_GRAPH_LEN * count);
    total_len = (uint16_t)(REF_HEADER_LEN + REF_CMD_LEN + payload_len + REF_CRC16_LEN);

    memset(&header, 0, sizeof(header));
    header.FrameHeader.SOF = REF_SOF;
    header.FrameHeader.DataLength = payload_len;
    header.FrameHeader.Seq = ui_seq;
    header.FrameHeader.CRC8 = crc_8((const uint8_t *)&header, 4);
    header.CmdID = INTERACTIVE_DATA_CMD_ID;
    header.datahead.data_cmd_id = data_cmd_id;
    header.datahead.sender_id = id->Robot_ID;
    header.datahead.receiver_id = id->Cilent_ID;

    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(header), graphs, (size_t)UI_SINGLE_GRAPH_LEN * count);
    Append_CRC16_Check_Sum(buffer, total_len - REF_CRC16_LEN);

    UI_Send_Buffer(buffer, total_len);
    ui_seq++;
}

void UIDelete(referee_id_t *id, uint8_t delete_operate, uint8_t delete_layer)
{
    UI_delete_t delete_frame;
    referee_id_t local_id;
    const referee_id_t *resolved_id = UI_Resolve_Referee_ID(id, &local_id);
    uint16_t data_len = UI_INTERACTIVE_HEADER_LEN + UI_DELETE_DATA_LEN;
    uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + data_len + REF_CRC16_LEN;

    if (!UI_Referee_ID_Is_Valid(resolved_id)) {
        return;
    }

    memset(&delete_frame, 0, sizeof(delete_frame));
    delete_frame.FrameHeader.SOF = REF_SOF;
    delete_frame.FrameHeader.DataLength = data_len;
    delete_frame.FrameHeader.Seq = ui_seq;
    delete_frame.FrameHeader.CRC8 = crc_8((const uint8_t *)&delete_frame, 4);
    delete_frame.CmdID = INTERACTIVE_DATA_CMD_ID;
    delete_frame.datahead.data_cmd_id = UI_DATA_DEL_ID;
    delete_frame.datahead.sender_id = resolved_id->Robot_ID;
    delete_frame.datahead.receiver_id = resolved_id->Cilent_ID;
    delete_frame.Delete_Operate = delete_operate;
    delete_frame.Layer = delete_layer;
    delete_frame.frametail = crc_16((const uint8_t *)&delete_frame, total_len - REF_CRC16_LEN);

    UI_Send_Buffer((uint8_t *)&delete_frame, total_len);
    ui_seq++;
}

void UILineDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Line;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void UIRectangleDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                     uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                     uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Rectangle;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void UICircleDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                  uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                  uint32_t graph_radius)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Circle;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->radius = graph_radius;
}

void UIOvalDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Ellipse;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void UIArcDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
               uint32_t graph_color, uint32_t graph_start_angle, uint32_t graph_end_angle,
               uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Arc;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->start_angle = graph_start_angle;
    graph->end_angle = graph_end_angle;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void UIFloatDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                 uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width,
                 uint32_t start_x, uint32_t start_y, int32_t graph_float)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Float;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->start_angle = graph_size;
    graph->end_angle = graph_digit;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->radius = (uint32_t)graph_float & 0x3FFu;
    graph->end_x = ((uint32_t)graph_float >> 10) & 0x7FFu;
    graph->end_y = ((uint32_t)graph_float >> 21) & 0x7FFu;
}

void UIIntDraw(Graph_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
               uint32_t graph_color, uint32_t graph_size, uint32_t graph_width,
               uint32_t start_x, uint32_t start_y, int32_t graph_integer)
{
    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->graphic_name, graphname);
    graph->operate_tpye = graph_operate;
    graph->graphic_tpye = UI_Graph_Int;
    graph->layer = graph_layer;
    graph->color = graph_color;
    graph->start_angle = graph_size;
    graph->width = graph_width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->radius = (uint32_t)graph_integer & 0x3FFu;
    graph->end_x = ((uint32_t)graph_integer >> 10) & 0x7FFu;
    graph->end_y = ((uint32_t)graph_integer >> 21) & 0x7FFu;
}

void UICharDraw(String_Data_t *graph, const char graphname[3], uint32_t graph_operate, uint32_t graph_layer,
                uint32_t graph_color, uint32_t graph_size, uint32_t graph_width,
                uint32_t start_x, uint32_t start_y, const char *fmt, ...)
{
    va_list ap;
    int written;

    if (graph == NULL) {
        return;
    }

    memset(graph, 0, sizeof(*graph));
    UI_Copy_Graphic_Name(graph->Graph_Control.graphic_name, graphname);
    graph->Graph_Control.operate_tpye = graph_operate;
    graph->Graph_Control.graphic_tpye = UI_Graph_Char;
    graph->Graph_Control.layer = graph_layer;
    graph->Graph_Control.color = graph_color;
    graph->Graph_Control.start_angle = graph_size;
    graph->Graph_Control.width = graph_width;
    graph->Graph_Control.start_x = start_x;
    graph->Graph_Control.start_y = start_y;

    va_start(ap, fmt);
    written = vsnprintf((char *)graph->show_Data, sizeof(graph->show_Data), fmt, ap);
    va_end(ap);

    if (written < 0) {
        graph->show_Data[0] = '\0';
        graph->Graph_Control.end_angle = 0u;
        return;
    }

    if ((size_t)written >= sizeof(graph->show_Data)) {
        graph->Graph_Control.end_angle = sizeof(graph->show_Data) - 1u;
    } else {
        graph->Graph_Control.end_angle = (uint32_t)written;
    }
}

void UIGraphRefresh(referee_id_t *id, int cnt, ...)
{
    Graph_Data_t graphs[UI_MAX_GRAPH_COUNT];
    referee_id_t local_id;
    const referee_id_t *resolved_id = UI_Resolve_Referee_ID(id, &local_id);
    va_list ap;
    uint8_t i;

    if (cnt != 1 && cnt != 2 && cnt != 5 && cnt != 7) {
        return;
    }

    va_start(ap, cnt);
    for (i = 0u; i < (uint8_t)cnt; i++) {
        graphs[i] = va_arg(ap, Graph_Data_t);
    }
    va_end(ap);

    UI_Send_GraphArray(resolved_id, (uint8_t)cnt, graphs);
}

void UICharRefresh(referee_id_t *id, String_Data_t string_data)
{
    UI_CharReFresh_t string_frame;
    referee_id_t local_id;
    const referee_id_t *resolved_id = UI_Resolve_Referee_ID(id, &local_id);
    uint16_t data_len = UI_INTERACTIVE_HEADER_LEN + UI_STRING_DATA_LEN;
    uint16_t total_len = REF_HEADER_LEN + REF_CMD_LEN + data_len + REF_CRC16_LEN;

    if (!UI_Referee_ID_Is_Valid(resolved_id)) {
        return;
    }

    memset(&string_frame, 0, sizeof(string_frame));
    string_frame.FrameHeader.SOF = REF_SOF;
    string_frame.FrameHeader.DataLength = data_len;
    string_frame.FrameHeader.Seq = ui_seq;
    string_frame.FrameHeader.CRC8 = crc_8((const uint8_t *)&string_frame, 4);
    string_frame.CmdID = INTERACTIVE_DATA_CMD_ID;
    string_frame.datahead.data_cmd_id = UI_DATA_DRAW_CHAR_ID;
    string_frame.datahead.sender_id = resolved_id->Robot_ID;
    string_frame.datahead.receiver_id = resolved_id->Cilent_ID;
    string_frame.String_Data = string_data;
    string_frame.frametail = crc_16((const uint8_t *)&string_frame, total_len - REF_CRC16_LEN);

    UI_Send_Buffer((uint8_t *)&string_frame, total_len);
    ui_seq++;
}
