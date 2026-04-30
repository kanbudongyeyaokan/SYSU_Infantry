/**
 * @file    bridge_link.h
 * @brief   上位机 Bridge Link 协议 —— 下位机实现
 *
 * 帧格式（与 Nero/core/mcu_bridge/bridge_link.hpp 完全对应）：
 *   [0xAA][type:1B][len_lo:1B][len_hi:1B][payload:nB][crc16_lo:1B][crc16_hi:1B]
 *
 *   type = 0x01 : CAN 帧  （双向）
 *   type = 0x02 : 裁判系统原始字节  （下→上，此处暂不使用）
 *
 * CAN 帧 payload（固定 13 字节）：
 *   [can_id: 4B LE][dlc: 1B][data: 8B]
 *
 * CRC16：CRC16-CCITT，多项式 0x1021，初始值 0xFFFF
 *         覆盖范围：SOF + type + len_lo + len_hi + payload（不含 CRC 自身）
 *
 * 本模块当前任务：
 *   - 初始化 USB CDC 接收回调
 *   - 以固定周期向上位机发送测试 CAN 帧（type=0x01）
 *   - 解析上位机下发的帧并进行 CRC16 校验（丢弃校验失败的帧）
 */

#ifndef BRIDGE_LINK_H
#define BRIDGE_LINK_H

#include <stdint.h>
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 协议常量 ──────────────────────────────────────────────────────────────── */

#define BRIDGE_SOF          0xAA    /**< 帧头 */
#define BRIDGE_TYPE_CAN     0x01    /**< CAN 帧类型 */
#define BRIDGE_TYPE_REF     0x02    /**< 裁判系统原始数据类型（暂不用）*/

#define BRIDGE_CAN_PAYLOAD  13      /**< CAN payload 固定长度：id(4)+dlc(1)+data(8) */
#define BRIDGE_FRAME_OH     6       /**< 帧开销：SOF(1)+type(1)+len(2)+CRC16(2) */
#define BRIDGE_CAN_FRAME_LEN (BRIDGE_FRAME_OH + BRIDGE_CAN_PAYLOAD)  /**< = 19 字节 */

#define BRIDGE_RX_FIFO_SIZE 512     /**< 接收 FIFO 大小 */
#define BRIDGE_MAX_PAYLOAD  256     /**< 单帧最大 payload（接收侧保护） */

/* ── 数据结构 ──────────────────────────────────────────────────────────────── */

/** CAN 帧（与上位机 CANFrame 结构体对齐） */
#pragma pack(1)
typedef struct {
    uint32_t id;        /**< CAN ID（小端） */
    uint8_t  dlc;       /**< 数据长度码 0-8 */
    uint8_t  data[8];   /**< CAN 数据，不足 8 字节时高位补 0 */
} Bridge_CANPayload_t;
#pragma pack()

/* ── 对外接口 ──────────────────────────────────────────────────────────────── */

/**
 * @brief  初始化 Bridge Link 模块
 * @note   内部调用 Usb_Init() 注册接收回调，须在 USB 设备枚举完成后调用
 */
void Bridge_Link_Init(void);

/**
 * @brief  发送一个 CAN 帧到上位机（带 CRC16 封帧）
 * @param  can_id   CAN 报文 ID
 * @param  dlc      数据长度 0-8
 * @param  data     数据指针（最多取 dlc 字节，其余填 0）
 */
void Bridge_Send_CAN(uint32_t can_id, uint8_t dlc, const uint8_t *data);

/**
 * @brief  解析接收 FIFO 中的帧（须在任务循环中轮询调用）
 * @note   成功解析的 CAN 帧将通过 Bridge_On_CAN_Received() 回调上报
 */
void Bridge_Link_Parse(void);

/**
 * @brief  收到来自上位机的合法 CAN 帧后的弱回调（用户可重写）
 * @param  frame  指向解析结果的指针（生命周期仅在回调内有效）
 */
__attribute__((weak))
void Bridge_On_CAN_Received(const Bridge_CANPayload_t *frame);

/**
 * @brief  发送测试帧任务（在 FreeRTOS 任务中以固定周期调用）
 * @note   每次调用发送一帧递增计数的测试报文，用于验证链路连通性
 */
void Bridge_Send_Test_Tick(void);

/**
 * @brief  CAN1 sniffer 回调（注册到 bsp_can），在 ISR 中被调用
 * @note   仅写环形缓冲，不做任何阻塞操作
 */
void Bridge_CAN_Sniffer_Cb(CAN_HandleTypeDef *hcan,
                            uint32_t std_id,
                            uint8_t dlc,
                            const uint8_t *data);

/**
 * @brief  将上行环形缓冲中的 CAN 帧批量发送给上位机（在任务上下文调用）
 */
void Bridge_Uplink_Drain(void);

/**
 * @brief  阻塞等待 USB RX 或 CAN sniffer 事件，超时后返回
 * @param  timeout_ms  最长等待时间（ms）
 */
void Bridge_Link_WaitRx(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_LINK_H */
