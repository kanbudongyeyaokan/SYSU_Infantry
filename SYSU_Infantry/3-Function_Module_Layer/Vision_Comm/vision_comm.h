#ifndef __VISION_COMM_H
#define __VISION_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==========================================
// 通信方式配置宏 (定义则使用 USB，注释则使用 UART)
// ==========================================
#define USE_VISION_USB

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
    uint32_t timestamp_us;               // 4-7
    float    linear_x;                   // 8-11
    float    linear_y;                   // 12-15
    float    linear_z;                   // 16-19
    float    gyro_wz;                    // 20-23
    float    angular_y;                  // 24-27 相对于云台中心的 pitch
    float    angular_z;                  // 28-31 相对于云台中心的 yaw
    float    angular_y_speed;            // 32-35 pitch 速度
    float    angular_z_speed;            // 36-39 yaw 速度
    float    distance;                   // 40-43
    
    // --- 裁判系统预留字段 ---
    uint8_t  game_progress;              // 44    bit 0~3
    uint16_t stage_remain_time;          // 45-46
    uint8_t  center_outpost_occupancy;   // 47    bit 0~1
    uint16_t current_HP;                 // 48-49
    uint16_t maximum_HP;                 // 50-51
    uint16_t shooter_barrel_heat_limit;  // 52-53
    uint8_t  power_management_output;    // 54    bit0: gimbal, bit1: chassis, bit2: shooter
    uint16_t shooter_17mm_barrel_heat;   // 55-56
    uint16_t shooter_42mm_barrel_heat;   // 57-58
    
    // 注意：为严格匹配表格，此处拆分为独立字节
    uint8_t  armor_id;                   // 59    bit 0~3
    uint8_t  HP_deduction_reason;        // 60    bit 0~3
    
    float    launching_frequency;        // 61-64
    float    initial_speed;              // 65-68
    uint16_t projectile_allowance_17mm;  // 69-70
    uint16_t projectile_allowance_42mm;  // 71-72
    uint32_t rfid_status;                // 73-76
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
    uint8_t  flags;         // 4      bit0: armor_detected, bit1: tracking_state, bit2: fire
    float    linear_x;      // 5-8
    float    linear_y;      // 9-12
    float    linear_z;      // 13-16
    float    angular_x;     // 17-20  小陀螺角速度 / 底盘旋转角速度
    float    angular_y;     // 21-24  Pitch 控制量
    float    angular_z;     // 25-28  Yaw 控制量
    float    distance;      // 29-32
    uint16_t frame_x;       // 33-34
    uint16_t frame_y;       // 35-36
} Vision_Rx_Payload_t;

#pragma pack(pop)

// ==========================================
// 外部调用数据结构
// ==========================================
typedef struct {
    uint8_t  tracking_state;// 0:丢失, 1:追踪中, 2:锁死可开火
    
    float    target_pitch;  // 映射自 angular_y
    float    target_yaw;    // 映射自 angular_z
    
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
void Vision_Send_Pose(uint32_t time_us, float pitch, float yaw, float pitch_v, float yaw_v, uint16_t current_hp, uint16_t max_hp);
bool Is_Vision_Online(void);
const Vision_Ctrl_Data_t* Get_Vision_Ctrl_Data(void);

#endif // __VISION_COMM_H