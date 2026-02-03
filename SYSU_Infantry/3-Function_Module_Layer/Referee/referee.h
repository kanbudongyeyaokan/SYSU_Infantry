#ifndef _REFEREE_H
#define _REFEREE_H

#include "main.h"
#include "bsp_usart.h"
#include "crc_referee.h" // 引用刚刚写的 CRC 库

// ---------------------------------------------------------
// 1. 协议常量 (RoboMaster 2026 V1.1.0)
// ---------------------------------------------------------
#define REF_SOF         0xA5
#define REF_HEADER_LEN  5    // SOF(1)+Len(2)+Seq(1)+CRC8(1)
#define REF_CMD_LEN     2    // CmdID
#define REF_CRC16_LEN   2    // FrameTail

// ---------------------------------------------------------
// 2. 命令码 ID 定义
// ---------------------------------------------------------
typedef enum {
    GAME_STATUS_ID          = 0x0001, // 比赛状态
    ROBOT_HP_ID             = 0x0003, // 机器人血量
    EVENT_DATA_ID           = 0x0101, // 场地事件
    ROBOT_STATUS_ID         = 0x0201, // 机器人状态 (验收核心：User口通信)
    POWER_HEAT_ID           = 0x0202, // 实时功率热量
    ROBOT_POS_ID            = 0x0203, // 机器人位置
    BUFF_MUSCLE_ID          = 0x0204, // 能量机关
    AERIAL_ENERGY_ID        = 0x0205, // 空中机器人能量
    ROBOT_HURT_ID           = 0x0206, // 伤害数据 (验收核心：装甲闪烁)
    SHOOT_DATA_ID           = 0x0207, // 射击数据 (验收核心：测速)
    RFID_STATUS_ID          = 0x0209, // RFID状态 (验收核心)
    INTERACTIVE_ID          = 0x0301, // 机器人交互/图传UI (验收核心)
} Ref_Cmd_Id_e;

// ---------------------------------------------------------
// 3. 协议数据结构 (1字节对齐)
// ---------------------------------------------------------
#pragma pack(push, 1)

// 帧头
typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_t;

// 0x0001 比赛状态
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

// 0x0201 机器人状态
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

// 0x0206 伤害数据
typedef struct {
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

// 0x0207 射击数据
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

// 0x0209 RFID
typedef struct {
    uint32_t rfid_status;
} ext_rfid_status_t;

// 0x0301 交互数据头 (用于发送UI)
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} ext_student_interactive_header_data_t;

// 基础图形数据
typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

#pragma pack(pop)

// ---------------------------------------------------------
// 4. 数据汇总
// ---------------------------------------------------------
typedef struct {
    ext_game_status_t       game_status;
    ext_game_robot_status_t robot_status;
    ext_robot_hurt_t        robot_hurt;
    ext_shoot_data_t        shoot_data;
    ext_rfid_status_t       rfid_status;

    uint8_t is_online; // 标志位
} Referee_Data_t;

// ---------------------------------------------------------
// 5. 接口声明
// ---------------------------------------------------------

Referee_Data_t* Referee_Get_Data(UART_HandleTypeDef *huart);
void Referee_Send_UI_Test(void); // 验收测试用

#endif