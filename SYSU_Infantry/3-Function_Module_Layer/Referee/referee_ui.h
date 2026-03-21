#ifndef _REFEREE_UI_H
#define _REFEREE_UI_H

#include "referee.h"

// ---------------------------------------------------------
// 1. UI 交互指令子内容 ID 定义
// ---------------------------------------------------------
#define UI_DATA_DEL_ID        0x0100 // 删除图层
#define UI_DATA_DRAW_1_ID     0x0101 // 绘制 1 个图形
#define UI_DATA_DRAW_2_ID     0x0102 // 绘制 2 个图形
#define UI_DATA_DRAW_5_ID     0x0103 // 绘制 5 个图形
#define UI_DATA_DRAW_7_ID     0x0104 // 绘制 7 个图形
#define UI_DATA_DRAW_CHAR_ID  0x0110 // 绘制字符图形

// ---------------------------------------------------------
// 2. UI 图形属性枚举 (让代码拥有极高可读性)
// ---------------------------------------------------------
typedef enum {
    UI_OPERATE_NULL   = 0, // 空操作
    UI_OPERATE_ADD    = 1, // 增加图层
    UI_OPERATE_MODIFY = 2, // 修改图层
    UI_OPERATE_DEL    = 3  // 删除图层
} UI_Operate_e;

typedef enum {
    UI_GRAPHIC_LINE      = 0, // 直线
    UI_GRAPHIC_RECTANGLE = 1, // 矩形
    UI_GRAPHIC_CIRCLE    = 2, // 正圆
    UI_GRAPHIC_ELLIPSE   = 3, // 椭圆
    UI_GRAPHIC_ARC       = 4, // 圆弧
    UI_GRAPHIC_FLOAT     = 5, // 浮点数
    UI_GRAPHIC_INT       = 6, // 整型数
    UI_GRAPHIC_CHAR      = 7  // 字符
} UI_Graphic_Type_e;

typedef enum {
    UI_COLOR_TEAM     = 0, // 己方颜色 (红/蓝)
    UI_COLOR_YELLOW   = 1, // 黄色
    UI_COLOR_GREEN    = 2, // 绿色
    UI_COLOR_ORANGE   = 3, // 橙色
    UI_COLOR_PURPLE   = 4, // 紫红色
    UI_COLOR_PINK     = 5, // 粉色
    UI_COLOR_CYAN     = 6, // 青色
    UI_COLOR_BLACK    = 7, // 黑色
    UI_COLOR_WHITE    = 8  // 白色
} UI_Color_e;

// ---------------------------------------------------------
// 3. UI 通信结构体定义 (仅补充原版缺失的部分)
// ---------------------------------------------------------
#pragma pack(push, 1)

// 绘制字符专属结构体 (原生图形头 15 字节 + 30 字节字符 = 45字节)
typedef struct {
    graphic_data_struct_t figure;
    uint8_t string[30];
} ui_string_t;

#pragma pack(pop)

// ---------------------------------------------------------
// 4. API 接口函数声明
// ---------------------------------------------------------

// 清除图层操作
void UI_Delete_Layer(uint8_t layer);
void UI_Delete_All(void);

// 图形数据装填 (不会立即发送，用于准备数据)
void UI_Pack_Line(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
void UI_Pack_Rectangle(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
void UI_Pack_Circle(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t width, uint32_t center_x, uint32_t center_y, uint32_t radius);
void UI_Pack_Float(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t start_x, uint32_t start_y, float value);
void UI_Pack_Int(graphic_data_struct_t *pic, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t start_x, uint32_t start_y, int32_t value);
void UI_Pack_String(ui_string_t *str_struct, const char* name, uint8_t op, uint8_t layer, uint8_t color, uint32_t font_size, uint32_t str_len, uint32_t start_x, uint32_t start_y, const char* string);

// 发送指令操作
void UI_Send_Single_Figure(graphic_data_struct_t *pic);
void UI_Send_Multi_Figures(uint8_t count, graphic_data_struct_t *pics);
void UI_Send_String(ui_string_t *str_struct);

#endif // _REFEREE_UI_H