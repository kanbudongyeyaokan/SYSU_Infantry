/**
  * @file    error_code.h
  * @brief   错误码定义
  *
  * 错误码格式 (32 位):
  * [31:24] 模块 ID (8 位)
  * [23:8]  错误 ID (16 位)
  * [7:0]   错误等级 (8 位)
  */

#ifndef __ERROR_CODE_H
#define __ERROR_CODE_H



#include "error_config.h"

/* ================= 错误码生成宏 ================= */

#define MAKE_ERROR_CODE(module, id, level) \
    (((uint32_t)((module) & 0xFFu) << 24u) | \
     ((uint32_t)((id) & 0xFFFFu) << 8u) | \
     ((uint32_t)((level) & 0xFFu)))

/* 错误码提取宏 */
#define ERROR_GET_MODULE(code)    (((code) >> 24u) & 0xFFu)
#define ERROR_GET_ID(code)        (((code) >> 8u) & 0xFFFFu)
#define ERROR_GET_LEVEL(code)     ((code) & 0xFFu)

/* ================= 通用错误 ID ================= */

/* 系统模块错误 */
#define ERROR_SYSTEM_NULL_POINTER       0x0001u
#define ERROR_SYSTEM_OUT_OF_MEMORY      0x0002u
#define ERROR_SYSTEM_INVALID_PARAM      0x0003u
#define ERROR_SYSTEM_TIMEOUT            0x0004u
#define ERROR_SYSTEM_STATE_ERROR        0x0005u

/* CAN 模块错误 */
#define ERROR_CAN_INIT_FAIL             0x0001u
#define ERROR_CAN_SEND_FAIL             0x0002u
#define ERROR_CAN_RECV_FAIL             0x0003u
#define ERROR_CAN_FIFO_OVERFLOW         0x0004u
#define ERROR_CAN_BUS_OFF               0x0005u
#define ERROR_CAN_TIMEOUT               0x0006u
#define ERROR_CAN_INVALID_ID            0x0007u

/* 电机模块错误 */
#define ERROR_MOTOR_INIT_FAIL           0x0001u
#define ERROR_MOTOR_COMM_FAIL           0x0002u
#define ERROR_MOTOR_OVER_CURRENT        0x0003u
#define ERROR_MOTOR_OVER_TEMP           0x0004u
#define ERROR_MOTOR_INVALID_SPEED       0x0005u
#define ERROR_MOTOR_LOST_FEEDBACK       0x0006u

/* IMU 模块错误 */
#define ERROR_IMU_INIT_FAIL             0x0001u
#define ERROR_IMU_COMM_FAIL             0x0002u
#define ERROR_IMU_DATA_INVALID          0x0003u
#define ERROR_IMU_CALIB_ERROR           0x0004u
#define ERROR_IMU_DRDY_TIMEOUT          0x0005u

/* 云台模块错误 */
#define ERROR_GIMBAL_INIT_FAIL          0x0001u
#define ERROR_GIMBAL_ANGLE_LIMIT        0x0002u
#define ERROR_GIMBAL_MOTOR_FAIL         0x0003u
#define ERROR_GIMBAL_SENSOR_FAIL        0x0004u

/* 底盘模块错误 */
#define ERROR_CHASSIS_INIT_FAIL         0x0001u
#define ERROR_CHASSIS_POWER_LIMIT       0x0002u
#define ERROR_CHASSIS_MOTOR_FAIL        0x0003u
#define ERROR_CHASSIS_ODOM_FAIL         0x0004u

/* 射击模块错误 */
#define ERROR_SHOOT_INIT_FAIL           0x0001u
#define ERROR_SHOOT_JAMMED              0x0002u
#define ERROR_SHOOT_MOTOR_FAIL          0x0003u
#define ERROR_SHOOT_FRIC_FAIL           0x0004u
#define ERROR_SHOOT_NO_BULLET           0x0005u

/* 裁判系统错误 */
#define ERROR_REFEREE_INIT_FAIL         0x0001u
#define ERROR_REFEREE_COMM_FAIL         0x0002u
#define ERROR_REFEREE_CRC_ERROR         0x0003u
#define ERROR_REFEREE_DATA_TIMEOUT      0x0004u

/* 遥控器错误 */
#define ERROR_REMOTE_INIT_FAIL          0x0001u
#define ERROR_REMOTE_LOST_SIGNAL        0x0002u
#define ERROR_REMOTE_DATA_ERROR         0x0003u

/* 电源模块错误 */
#define ERROR_POWER_LOW_VOLTAGE         0x0001u
#define ERROR_POWER_OVER_CURRENT        0x0002u
#define ERROR_POWER_OVER_TEMP           0x0003u
#define ERROR_POWER_CAP_FAIL            0x0004u

/* 音频模块错误 */
#define ERROR_AUDIO_INIT_FAIL           0x0001u
#define ERROR_AUDIO_PLAY_FAIL           0x0002u

/* USB 模块错误 */
#define ERROR_USB_INIT_FAIL             0x0001u
#define ERROR_USB_ENUM_FAIL             0x0002u
#define ERROR_USB_SEND_FAIL             0x0003u
#define ERROR_USB_RECV_FAIL             0x0004u



#endif /* __ERROR_CODE_H */
