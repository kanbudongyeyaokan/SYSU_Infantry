#ifndef SYSU_INFANTRY_REFEREE_H
#define SYSU_INFANTRY_REFEREE_H

#include "main.h"
#include "protocol.h"
#include "stdbool.h"

/* 机器人 ID 枚举，红方 1-7，蓝方 11-17 */
typedef enum {
    RED_HERO        = 1,  RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,  RED_STANDARD_2  = 4,  RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,  RED_SENTRY      = 7,
    BLUE_HERO       = 11, BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13, BLUE_STANDARD_2 = 14, BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16, BLUE_SENTRY     = 17,
} robot_id_t;

/* 比赛阶段枚举 */
typedef enum {
    PROGRESS_UNSTART = 0, PROGRESS_PREPARE = 1, PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3, PROGRESS_BATTLE = 4, PROGRESS_CALCULATING = 5,
} game_progress_t;

#pragma pack(push, 1)

/* 0x0001 比赛状态：类型、阶段、剩余时间、时间戳 */
typedef struct { uint8_t game_type:4; uint8_t game_progress:4; uint16_t stage_remain_time; uint64_t SyncTimeStamp; } game_status_t;
/* 0x0002 比赛结果 */
typedef struct { uint8_t winner; } game_result_t;
/* 0x0003 全场机器人血量 */
typedef struct {
    uint16_t red_1_robot_HP, red_2_robot_HP, red_3_robot_HP, red_4_robot_HP, red_5_robot_HP;
    uint16_t red_7_robot_HP, red_outpost_HP, red_base_HP;
    uint16_t blue_1_robot_HP, blue_2_robot_HP, blue_3_robot_HP, blue_4_robot_HP, blue_5_robot_HP;
    uint16_t blue_7_robot_HP, blue_outpost_HP, blue_base_HP;
} game_robot_HP_t;
/* 0x0101 场地事件 */
typedef struct { uint32_t event_data; } event_data_t;
/* 0x0104 裁判警告 */
typedef struct { uint8_t level, offending_robot_id, count; } referee_warning_t;
/* 0x0105 飞镖发射数据 */
typedef struct { uint8_t dart_launch_opening_status, dart_attack_target, target_change_time; } dart_launch_t;
/* 0x0201 机器人状态：ID、等级、血量、热量限制、功率限制、输出使能 */
typedef struct {
    uint8_t robot_id, robot_level; uint16_t current_HP, maximum_HP;
    uint16_t shooter_barrel_cooling_value, shooter_barrel_heat_limit, chassis_power_limit;
    uint8_t power_management_gimbal_output:1, power_management_chassis_output:1, power_management_shooter_output:1;
} robot_status_t;
/* 0x0202 功率热量数据：底盘电压/电流/功率/缓冲能量，枪管热量 */
typedef struct {
    uint16_t chassis_voltage, chassis_current; float chassis_power; uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat, shooter_17mm_2_barrel_heat, shooter_42mm_barrel_heat;
} power_heat_data_t;
/* 0x0203 机器人位置 */
typedef struct { float x, y, z, angle; } robot_pos_t;
/* 0x0204 增益状态 */
typedef struct { uint8_t recovery_buff, cooling_buff, defence_buff, vulnerability_buff; uint16_t attack_buff; } buff_t;
/* 0x0206 受击信息：装甲板 ID 与扣血原因 */
typedef struct { uint8_t armor_id:4, HP_deduction_reason:4; } hurt_data_t;
/* 0x0207 实时射击数据：弹丸类型、发射频率、初速度 */
typedef struct { uint8_t bullet_type, shooter_number, launching_frequency; float initial_speed; } shoot_data_t;
/* 0x0208 剩余弹量与金币 */
typedef struct { uint16_t projectile_allowance_17mm, projectile_allowance_42mm, remaining_gold_coin; } projectile_allowance_t;
/* 0x0209 RFID 状态 */
typedef struct { uint32_t rfid_status; uint8_t reserved; } rfid_status_t;
/* 0x0301 机器人间交互数据 */
typedef struct { uint16_t data_cmd_id, sender_id, receiver_id; uint8_t user_data[113]; } robot_interaction_data_t;
/* 0x0302 自定义控制器数据（30字节原始数据） */
typedef struct { uint8_t data[30]; } CustomControllerData_t;
/* 0x0305 雷达数据 */
typedef struct { uint8_t data[24]; } radar_data_t;
/* 0x0307 路径规划数据 */
typedef struct { uint8_t data[103]; } path_planning_t;

#pragma pack(pop)

/* 全局裁判系统数据，供其他模块直接读取 */
extern game_robot_HP_t game_robot_HP;
extern robot_status_t  robot_status;
extern game_status_t   game_status;

extern void     init_referee_struct_data(void);                                          // 清零所有裁判系统数据结构
extern void     referee_data_solve(uint8_t *frame);                                      // 解析一帧裁判系统数据
extern void     get_chassis_power_and_buffer(float *power, float *buffer);               // 获取底盘功率和缓冲能量
extern uint16_t get_shoot_heat(void);                                                    // 获取当前最大枪管热量
extern uint8_t  get_robot_id(void);                                                      // 获取本机器人 ID
extern uint8_t  get_team_color(void);                                                    // 获取队伍颜色：0=红，1=蓝，2=未知
extern void     get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0); // 获取 17mm 1 号枪管热量及上限
extern void     get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1); // 获取 17mm 2 号枪管热量及上限
extern CustomControllerData_t *GetCustomControllerDataPoint(void);                       // 获取自定义控制器数据指针
extern bool     GetRefereeOffline(void);                                                 // 裁判系统是否离线
extern float    GetCustomControllerPos(uint8_t index);                                   // 读取自定义控制器第 index 个浮点数

#endif //SYSU_INFANTRY_REFEREE_H
