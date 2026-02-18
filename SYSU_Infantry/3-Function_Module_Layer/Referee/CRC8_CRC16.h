#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#include "main.h"

/* 计算 CRC8 校验值（查表法，初始值由调用方传入） */
extern uint8_t  get_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
/* 验证数据包末字节是否为正确的 CRC8，返回 1 表示通过 */
extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
/* 在数据包末字节写入 CRC8 校验值 */
extern void     append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/* 计算 CRC16 校验值（查表法，初始值由调用方传入） */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
/* 验证数据包末两字节是否为正确的 CRC16，返回 1 表示通过 */
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
/* 在数据包末两字节写入 CRC16 校验值（小端序） */
extern void     append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
