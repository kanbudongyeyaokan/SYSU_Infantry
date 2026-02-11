#ifndef _REFEREE_H
#define _REFEREE_H

#include "main.h"
#include "bsp_usart.h"
#include "crc_referee.h"

// ---------------------------------------------------------
// 1. 协议常量 (RoboMaster 2026 V1.1.0)
// ---------------------------------------------------------
#define REF_SOF         0xA5
#define REF_HEADER_LEN  5
#define REF_CMD_LEN     2
#define REF_CRC16_LEN   2

// ---------------------------------------------------------
// 2. 命令码 ID 定义 (CmdID)
// ---------------------------------------------------------
typedef enum {
    GAME_STATUS_CMD_ID          = 0x0001, // 比赛状态
    GAME_RESULT_CMD_ID          = 0x0002, // 比赛结果
    ROBOT_HP_CMD_ID             = 0x0003, // 机器人血量

    EVENT_DATA_CMD_ID           = 0x0101, // 场地事件
    referee_WARNING_CMD_ID      = 0x0104, // 裁判警告
    DART_INFO_CMD_ID            = 0x0105, // 飞镖发射信息

    ROBOT_STATUS_CMD_ID         = 0x0201, // 机器人状态
    POWER_HEAT_DATA_CMD_ID      = 0x0202, // 实时功率热量
    ROBOT_POS_CMD_ID            = 0x0203, // 机器人位置
    BUFF_MUSCLE_CMD_ID          = 0x0204, // 机器人增益
    ROBOT_HURT_CMD_ID           = 0x0206, // 伤害数据
    SHOOT_DATA_CMD_ID           = 0x0207, // 实时射击
    PROJECTILE_ALLOWANCE_CMD_ID = 0x0208, // 允许发弹量
    RFID_STATUS_CMD_ID          = 0x0209, // RFID状态
    DART_CLIENT_CMD_ID          = 0x020A, // 飞镖选手端指令
    GROUND_ROBOT_POS_CMD_ID     = 0x020B, // 地面机器人位置(哨兵用)
    RADAR_MARK_CMD_ID           = 0x020C, // 雷达标记进度
    SENTRY_INFO_CMD_ID          = 0x020D, // 哨兵自主决策信息
    RADAR_INFO_CMD_ID           = 0x020E, // 雷达自主决策信息

    INTERACTIVE_DATA_CMD_ID     = 0x0301, // 机器人交互/图传UI
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

// 0x0002 比赛结果
typedef struct {
    uint8_t winner;
} ext_game_result_t;

// 0x0003 机器人血量
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// 0x0101 场地事件
typedef struct {
    uint32_t event_type;
} ext_event_data_t;

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

// 0x0202 实时功率热量
typedef struct {
    uint16_t reserved_volt;         // 保留 (原电压)
    uint16_t reserved_current;      // 保留 (原电流)
    float    reserved_power;        // 保留 (原功率)
    uint16_t buffer_energy;         // 底盘缓冲能量 (J) [重要]
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

// 0x0203 机器人位置
typedef struct {
    float x;
    float y;
    float angle;
} ext_robot_pos_t;

// 0x0204 机器人增益
typedef struct {
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} ext_buff_t;

// 0x0206 伤害数据
typedef struct {
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

// 0x0207 实时射击数据
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

// 0x0208 允许发弹量
typedef struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

// 0x0209 RFID状态
typedef struct {
    uint32_t rfid_status;
} ext_rfid_status_t;

// 0x0301 交互数据头
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} ext_student_interactive_header_data_t;

// 图形数据结构 (0x0101)
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
// 4. 数据汇总对象
// ---------------------------------------------------------
typedef struct {
    ext_game_status_t           game_status;
    ext_game_result_t           game_result;
    ext_game_robot_HP_t         game_robot_HP;
    ext_event_data_t            event_data;
    ext_game_robot_status_t     robot_status;
    ext_power_heat_data_t       power_heat_data; // 包含缓冲能量
    ext_robot_pos_t             robot_pos;
    ext_buff_t                  buff;
    ext_robot_hurt_t            robot_hurt;
    ext_shoot_data_t            shoot_data;
    ext_projectile_allowance_t  projectile_allowance;
    ext_rfid_status_t           rfid_status;

    uint8_t is_online;
} Referee_Data_t;

// ---------------------------------------------------------
// 5. 接口声明
// ---------------------------------------------------------

Referee_Data_t* Referee_Get_Data(UART_HandleTypeDef *huart);
void Referee_Send_UI_Test(void);


uint16_t ChassisPower_GetMaxLimit(void);
uint16_t ChassisPower_GetBuffer(void);

#endif