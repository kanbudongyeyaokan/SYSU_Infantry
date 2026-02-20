#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include "main.h"

/* 帧起始字节 */
#define HEADER_SOF                        0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE       128
#define REF_PROTOCOL_HEADER_SIZE          sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE             2
#define REF_PROTOCOL_CRC16_SIZE           2
#define REF_HEADER_CRC_LEN                (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN              (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

/* 裁判系统命令 ID，对应各类数据包 */
typedef enum {
    GAME_STATE_CMD_ID                = 0x0001, // 比赛状态
    GAME_RESULT_CMD_ID               = 0x0002, // 比赛结果
    GAME_ROBOT_HP_CMD_ID             = 0x0003, // 全场血量
    FIELD_EVENTS_CMD_ID              = 0x0101, // 场地事件
    REFEREE_WARNING_CMD_ID           = 0x0104, // 裁判警告
    DART_LAUNCH_CMD_ID               = 0x0105, // 飞镖发射
    ROBOT_STATE_CMD_ID               = 0x0201, // 机器人状态
    POWER_HEAT_DATA_CMD_ID           = 0x0202, // 功率热量
    ROBOT_POS_CMD_ID                 = 0x0203, // 机器人位置
    BUFF_MUSK_CMD_ID                 = 0x0204, // 增益状态
    ROBOT_HURT_CMD_ID                = 0x0206, // 受击信息
    SHOOT_DATA_CMD_ID                = 0x0207, // 实时射击
    BULLET_REMAINING_CMD_ID          = 0x0208, // 剩余弹量
    RFID_STATUS_CMD_ID               = 0x0209, // RFID 状态
    STUDENT_INTERACTIVE_DATA_CMD_ID  = 0x0301, // 机器人交互
    CUSTOM_CONTROLLER_CMD_ID         = 0x0302, // 自定义控制器
    RADAR_DATA_CMD_ID                = 0x0305, // 雷达数据
    PATH_PLANNING_CMD_ID             = 0x0307, // 路径规划
} referee_cmd_id_t;

/* 帧头结构：SOF(1) + 数据长度(2) + 序号(1) + CRC8(1) */
typedef struct {
    uint8_t  SOF;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  CRC8;
} frame_header_struct_t;

/* 逐字节解包状态机的状态枚举 */
typedef enum {
    STEP_HEADER_SOF  = 0, // 等待帧头 0xA5
    STEP_LENGTH_LOW  = 1, // 读取数据长度低字节
    STEP_LENGTH_HIGH = 2, // 读取数据长度高字节
    STEP_FRAME_SEQ   = 3, // 读取帧序号
    STEP_HEADER_CRC8 = 4, // 读取并校验帧头 CRC8
    STEP_DATA_CRC16  = 5, // 读取数据段及 CRC16
} unpack_step_e;

/* 解包上下文，保存当前解包进度和缓冲区 */
typedef struct {
    frame_header_struct_t *p_header;
    uint16_t               data_len;
    uint8_t                protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e          unpack_step;
    uint16_t               index;
} unpack_data_t;

#pragma pack(pop)

#endif // REFEREE_PROTOCOL_H
