#ifndef _CRC_REFEREE_H
#define _CRC_REFEREE_H

#include "main.h"

// ------------------------------------------------
// 基础计算函数
// ------------------------------------------------
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);

// ------------------------------------------------
// 裁判系统专用封装 (添加/校验)
// ------------------------------------------------
/**
 * @brief 校验帧头的 CRC8
 * @param pchMessage 帧头指针 (包含SOF到CRC8共5字节)
 * @param dwLength 帧头长度 (固定为5)
 * @return 1:校验通过, 0:失败
 */
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

/**
 * @brief 在数据尾部添加 CRC8
 * @param pchMessage 数据指针
 * @param dwLength 数据长度 (不包含CRC8字节本身)
 */
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

/**
 * @brief 校验整帧的 CRC16
 * @param pchMessage 帧指针
 * @param dwLength 整帧长度
 * @return 1:校验通过, 0:失败
 */
uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief 在数据尾部添加 CRC16
 * @param pchMessage 数据指针
 * @param dwLength 数据长度 (不包含CRC16两字节)
 */
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif