#ifndef __VISION_COMM_H
#define __VISION_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==========================================
// 协议宏定义
// ==========================================
#define VISION_SOF_TX        0xA5     // C板发给视觉的帧头
#define VISION_SOF_RX        0xA5     // 视觉发给C板的帧头 (新协议统一为 A5)

#define CMD_ID_POSE_TX       0x01     // C板发送姿态数据的命令码
#define CMD_ID_CTRL_RX       0x02     // C板接收控制数据的命令码

#define VISION_RX_FIFO_SIZE  1024     // 视觉接收缓冲区大小

// ==========================================
// 内部通信数据结构定义 (严格1字节对齐，严格匹配协议表格)
// ==========================================
#pragma pack(push, 1)

// --- C板 发给 NUC 的数据段 Payload (0x01) ---
typedef struct {
    uint32_t timestamp_us;
    float    linear_x;
    float    linear_y;
    float    linear_z;
    float    gyro_wz;
    float    angular_y;       // 相对于云台中心的 pitch
    float    angular_z;       // 相对于云台中心的 yaw
    float    angular_y_speed; // pitch 速度
    float    angular_z_speed; // yaw 速度
    float    distance;
    
    // --- 裁判系统预留字段 (默认填0) ---
    uint8_t  game_progress;              // bit 0~3
    uint16_t stage_remain_time;
    uint8_t  center_outpost_occupancy;   // bit 0~1
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_heat_limit;
    uint8_t  power_management_output;    // bit0: gimbal, bit1: chassis, bit2: shooter
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
    uint8_t  armor_id_and_reason;        // bit0~3: armor_id, bit4~7: HP_deduction_reason
    float    launching_frequency;
    float    initial_speed;
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint32_t rfid_status;
} Vision_Tx_Payload_t;

// --- TX 完整帧 ---
typedef struct {
    uint8_t  sof;           
    uint16_t data_length;   
    uint8_t  cmd_id;        
    Vision_Tx_Payload_t data; // 数据段
    uint16_t crc16;         
} EC2Vision_Pose_t;

// --- NUC 发给 C板 的数据段 Payload (0x02) ---
typedef struct {
    uint8_t  flags;         // bit0: armor_detected, bit1: tracking_state, bit2: fire
    float    linear_x;
    float    linear_y;
    float    linear_z;
    float    angular_x;     // 小陀螺角速度 / 底盘旋转角速度
    float    angular_y;     // Pitch 控制量
    float    angular_z;     // Yaw 控制量
    float    distance;
    uint16_t frame_x;
    uint16_t frame_y;
} Vision_Rx_Payload_t;

#pragma pack(pop)

// ==========================================
// 外部调用数据结构 (兼容旧接口，绝不修改外界调用的逻辑)
// ==========================================
typedef struct {
    // 兼容原有的标志位映射
    uint8_t  tracking_state;// 0:丢失, 1:追踪中, 2:锁死可开火
    
    // 兼容原有的目标角
    float    target_pitch;  // 映射自 angular_y
    float    target_yaw;    // 映射自 angular_z
    
    // 【警告】新协议中丢失了速度前馈，在此强制置 0，留作未来扩展
    float    target_pitch_v;
    float    target_yaw_v;  
    
    // 新增透传给底盘的控制量
    float    linear_x;
    float    linear_y;
    float    linear_z;
    float    angular_x;
} Vision_Ctrl_Data_t;

// ==========================================
// 外部接口函数声明
// ==========================================

void Vision_Comm_Init(void);
void Vision_Comm_Parse_Task(void);
void Vision_Send_Pose(uint32_t time_us, float pitch, float yaw, float roll, float pitch_v, float yaw_v);
const Vision_Ctrl_Data_t* Get_Vision_Ctrl_Data(void);
bool Is_Vision_Online(void);

#endif // __VISION_COMM_H