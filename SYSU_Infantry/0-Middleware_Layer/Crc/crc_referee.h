#ifndef _CRC_REFEREE_H
#define _CRC_REFEREE_H

#include "main.h"

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t get_crc16_check_sum(const uint8_t *data, uint32_t len);

uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

// Hero-compatible interface: dwLength is the full frame length including the trailing CRC16 bytes.
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
