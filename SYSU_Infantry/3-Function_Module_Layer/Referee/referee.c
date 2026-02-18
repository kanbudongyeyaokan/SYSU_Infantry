#include "referee.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "string.h"
#include "stdbool.h"

/* 超过此毫秒数未收到数据则判定为离线 */
#define REFEREE_TIMEOUT 300

static uint32_t referee_online_time  = 0; // 最后一次成功收帧的时刻（HAL_GetTick）
static uint32_t referee_receive_count = 0; // 有效帧计数，用于判断是否在线

/* 全局裁判系统数据（在 referee.h 中 extern 声明） */
game_status_t   game_status;
game_result_t   game_result;
game_robot_HP_t game_robot_HP;

static event_data_t                   field_event;
static ext_supply_projectile_action_t supply_projectile_action;
static ext_supply_projectile_booking_t supply_projectile_booking;
static referee_warning_t              referee_warning;

robot_status_t    robot_status;
power_heat_data_t power_heat_data;
static robot_pos_t         game_robot_pos;
static buff_t              buff_musk;
static air_support_data_t  robot_energy;
static hurt_data_t         robot_hurt;
static shoot_data_t        shoot_data;
static projectile_allowance_t   projectile_allowance;
static robot_interaction_data_t student_interactive_data;
static CustomControllerData_t   CUSTOM_CONTROLLER_DATA;
static ext_robot_command_t      robot_command;

/* 清零所有裁判系统数据结构，上电或重连时调用 */
void init_referee_struct_data(void)
{
    memset(&game_status,   0, sizeof(game_status_t));
    memset(&game_result,   0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&field_event,   0, sizeof(event_data_t));
    memset(&supply_projectile_action,  0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&robot_status,    0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos,  0, sizeof(robot_pos_t));
    memset(&buff_musk,       0, sizeof(buff_t));
    memset(&robot_energy,    0, sizeof(air_support_data_t));
    memset(&robot_hurt,      0, sizeof(hurt_data_t));
    memset(&shoot_data,      0, sizeof(shoot_data_t));
    memset(&student_interactive_data, 0, sizeof(robot_interaction_data_t));
    memset(&CUSTOM_CONTROLLER_DATA,   0, sizeof(CustomControllerData_t));
    memset(&robot_command,   0, sizeof(ext_robot_command_t));
}

/*
 * 解析一帧裁判系统数据
 * frame: 已通过 CRC 校验的完整帧缓冲区（帧头 + cmd_id + 数据 + CRC16）
 * 根据 cmd_id 将数据 memcpy 到对应全局结构体，并更新在线时间戳
 */
void referee_data_solve(uint8_t *frame)
{
    /* 超时后重置计数，防止旧计数误判为在线 */
    if (HAL_GetTick() - referee_online_time > REFEREE_TIMEOUT)
        referee_receive_count = 0;
    referee_receive_count++;

    uint16_t cmd_id = 0;
    uint8_t  index  = sizeof(frame_header_struct_t);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

/* 宏：按 cmd_id 匹配后将帧数据 memcpy 到目标结构体，并刷新在线时间戳 */
#define SOLVE(id, dst, type) case id: memcpy(&(dst), frame + index, sizeof(type)); referee_online_time = HAL_GetTick(); break
    switch (cmd_id) {
        SOLVE(GAME_STATE_CMD_ID,                game_status,              game_status_t);
        SOLVE(GAME_RESULT_CMD_ID,               game_result,              game_result_t);
        SOLVE(GAME_ROBOT_HP_CMD_ID,             game_robot_HP,            game_robot_HP_t);
        SOLVE(FIELD_EVENTS_CMD_ID,              field_event,              event_data_t);
        SOLVE(SUPPLY_PROJECTILE_ACTION_CMD_ID,  supply_projectile_action, ext_supply_projectile_action_t);
        SOLVE(SUPPLY_PROJECTILE_BOOKING_CMD_ID, supply_projectile_booking,ext_supply_projectile_booking_t);
        SOLVE(REFEREE_WARNING_CMD_ID,           referee_warning,          referee_warning_t);
        SOLVE(ROBOT_STATE_CMD_ID,               robot_status,             robot_status_t);
        SOLVE(POWER_HEAT_DATA_CMD_ID,           power_heat_data,          power_heat_data_t);
        SOLVE(ROBOT_POS_CMD_ID,                 game_robot_pos,           robot_pos_t);
        SOLVE(BUFF_MUSK_CMD_ID,                 buff_musk,                buff_t);
        SOLVE(AERIAL_ROBOT_ENERGY_CMD_ID,       robot_energy,             air_support_data_t);
        SOLVE(ROBOT_HURT_CMD_ID,                robot_hurt,               hurt_data_t);
        SOLVE(SHOOT_DATA_CMD_ID,                shoot_data,               shoot_data_t);
        SOLVE(BULLET_REMAINING_CMD_ID,          projectile_allowance,     projectile_allowance_t);
        SOLVE(STUDENT_INTERACTIVE_DATA_CMD_ID,  student_interactive_data, robot_interaction_data_t);
        SOLVE(CUSTOM_CONTROLLER_CMD_ID,         CUSTOM_CONTROLLER_DATA,   CustomControllerData_t);
        SOLVE(ROBOT_COMMAND_CMD_ID,             robot_command,            ext_robot_command_t);
        default: referee_receive_count--; break; // 未知 cmd_id，撤销本次计数
    }
#undef SOLVE
}

/* 获取底盘实时功率（W）和缓冲能量（J） */
void get_chassis_power_and_buffer(float *power, float *buffer)
{
    *power  = power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;
}

/* 获取本机器人 ID */
uint8_t get_robot_id(void) { return robot_status.robot_id; }

/* 获取 17mm 1 号枪管热量上限和当前热量 */
void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_status.shooter_barrel_heat_limit;
    *heat0       = power_heat_data.shooter_17mm_1_barrel_heat;
}

/* 获取 17mm 2 号枪管热量上限和当前热量 */
void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_status.shooter_barrel_heat_limit;
    *heat1       = power_heat_data.shooter_17mm_2_barrel_heat;
}

/* 根据机器人 ID 判断队伍颜色：0=红，1=蓝，2=未知 */
uint8_t get_team_color(void)
{
    uint8_t id = robot_status.robot_id;
    if (id >= 1  && id <= 11)  return 0; // red
    if (id >= 101 && id <= 111) return 1; // blue
    return 2;
}

/* 返回两个 17mm 枪管中热量较大的值，用于统一热量控制 */
uint16_t get_shoot_heat(void)
{
    return (power_heat_data.shooter_17mm_1_barrel_heat > power_heat_data.shooter_17mm_2_barrel_heat)
        ? power_heat_data.shooter_17mm_1_barrel_heat
        : power_heat_data.shooter_17mm_2_barrel_heat;
}

/* 获取自定义控制器原始数据指针 */
CustomControllerData_t *GetCustomControllerDataPoint(void) { return &CUSTOM_CONTROLLER_DATA; }

/*
 * 判断裁判系统是否离线
 * 条件：有效帧数 > 5 且距上次收帧未超时，两者同时满足才视为在线
 */
bool GetRefereeOffline(void)
{
    return !((referee_receive_count > 5) && (HAL_GetTick() - referee_online_time < REFEREE_TIMEOUT));
}

/*
 * 从自定义控制器数据中读取第 index 个浮点数（每个占 4 字节）
 * index: 0~6（data 共 30 字节，最多 7 个完整 float）
 */
float GetCustomControllerPos(uint8_t index)
{
    float data = 0;
    memcpy(&data, &CUSTOM_CONTROLLER_DATA.data[index * 4], 4);
    return data;
}
