#ifndef __VISION_COMM_H
#define __VISION_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==========================================
// 协议宏定义
// ==========================================
#define VISION_SOF_TX        0xA5     // C板发给视觉的帧头
#define VISION_SOF_RX        0x5A     // 视觉发给C板的帧头

#define CMD_ID_POSE_TX       0x01     // C板发送姿态数据的命令码
#define CMD_ID_CTRL_RX       0x02     // C板接收控制数据的命令码

#define VISION_RX_FIFO_SIZE  1024     // 视觉接收缓冲区大小

// ==========================================
// 数据结构定义 (严格1字节对齐)
// ==========================================
#pragma pack(push, 1)

// --- EC 发给 Vision 的姿态包 (TX) ---
typedef struct {
    uint8_t  sof;           // 0xA5
    uint16_t data_length;   // 数据段长度
    uint8_t  cmd_id;        // 0x01
    
    // 数据段开始
    uint32_t timestamp_us;  // 绝对时间戳 (微秒)
    float    pitch_angle;   // 绝对 Pitch 角
    float    yaw_angle;     // 绝对 Yaw 角 (连续的多圈角度)
    float    roll_angle;    
    
    float    pitch_speed;   // 纯净 Pitch 角速度
    float    yaw_speed;     // 纯净 Yaw 角速度
    
    uint16_t crc16;         // 整包 CRC16 校验
} EC2Vision_Pose_t;

// --- Vision 发给 EC 的控制包 (RX) ---
// 注意：为了跟通用解析逻辑兼容，我们将数据段独立出来
typedef struct {
    uint8_t  tracking_state;// 0:丢失, 1:追踪中, 2:锁死
    
    float    target_pitch;  // 预测后的绝对 Pitch 目标角
    float    target_yaw;    // 预测后的绝对 Yaw 目标角
    
    float    target_pitch_v;// 预测的 Pitch 轴目标角速度 (前馈用)
    float    target_yaw_v;  // 预测的 Yaw 轴目标角速度 (前馈用)
} Vision_Ctrl_Data_t;

typedef struct {
    uint8_t  sof;           // 0x5A
    uint16_t data_length;   // 数据段长度 (sizeof(Vision_Ctrl_Data_t))
    uint8_t  cmd_id;        // 0x02
    
    Vision_Ctrl_Data_t data; // 数据段
    
    uint16_t crc16;         // 整包 CRC16 校验
} Vision2EC_Cmd_t;

#pragma pack(pop)

// ==========================================
// 外部接口函数声明
// ==========================================

void Vision_Comm_Init(void);

/**
 * @brief  视觉数据解析处理函数 (放在独立的解析任务中死循环调用)
 */
void Vision_Comm_Parse_Task(void);

/**
 * @brief  向视觉发送当前的绝对位姿 (放在 1000Hz 的发送任务中调用)
 */
void Vision_Send_Pose(uint32_t time_us, float pitch, float yaw, float roll, float pitch_v, float yaw_v);

/**
 * @brief  获取最新的一帧有效视觉控制指令
 */
const Vision_Ctrl_Data_t* Get_Vision_Ctrl_Data(void);

/**
 * @brief  检查视觉是否离线 (结合超时保护机制)
 */
bool Is_Vision_Online(void);

#endif // __VISION_COMM_H