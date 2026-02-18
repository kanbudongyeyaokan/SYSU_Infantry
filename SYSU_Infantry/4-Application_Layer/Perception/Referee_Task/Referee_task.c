#include "Referee_task.h"
#include "referee.h"
#include "protocol.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"

static Uart_instance_t *referee_uart;       // 裁判系统 UART 实例
static unpack_data_t    referee_unpack_obj; // 解包上下文（状态机 + 缓冲区）

/* 环形缓冲区：中断写入，任务读取，解耦 DMA 回调与解包处理 */
#define REFEREE_RING_SIZE 512
static uint8_t           ring_buf[REFEREE_RING_SIZE];
static volatile uint16_t ring_head = 0; // 读指针（任务侧）
static volatile uint16_t ring_tail = 0; // 写指针（中断侧）

/*
 * 逐字节解包裁判系统数据帧（状态机实现）
 * 流程：等待 SOF → 读长度 → 读序号 → 校验帧头 CRC8 → 收数据并校验 CRC16 → 解析
 */
static void referee_unpack_buf(const uint8_t *buf, uint16_t len)
{
    unpack_data_t *p = &referee_unpack_obj;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        switch (p->unpack_step) {
            case STEP_HEADER_SOF:
                if (byte == HEADER_SOF) {
                    p->unpack_step = STEP_LENGTH_LOW;
                    p->protocol_packet[p->index++] = byte;
                } else {
                    p->index = 0; // 非 SOF 字节，丢弃并重置
                }
                break;
            case STEP_LENGTH_LOW:
                p->data_len = byte;
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_LENGTH_HIGH;
                break;
            case STEP_LENGTH_HIGH:
                p->data_len |= (byte << 8);
                p->protocol_packet[p->index++] = byte;
                // 数据长度合法则继续，否则丢帧
                if (p->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                    p->unpack_step = STEP_FRAME_SEQ;
                else { p->unpack_step = STEP_HEADER_SOF; p->index = 0; }
                break;
            case STEP_FRAME_SEQ:
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_HEADER_CRC8;
                break;
            case STEP_HEADER_CRC8:
                p->protocol_packet[p->index++] = byte;
                if (p->index == REF_PROTOCOL_HEADER_SIZE) {
                    // 帧头 CRC8 校验通过则继续接收数据段
                    if (verify_CRC8_check_sum(p->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                        p->unpack_step = STEP_DATA_CRC16;
                    else { p->unpack_step = STEP_HEADER_SOF; p->index = 0; }
                }
                break;
            case STEP_DATA_CRC16:
                if (p->index < (REF_HEADER_CRC_CMDID_LEN + p->data_len))
                    p->protocol_packet[p->index++] = byte;
                if (p->index >= (REF_HEADER_CRC_CMDID_LEN + p->data_len)) {
                    p->unpack_step = STEP_HEADER_SOF;
                    p->index = 0;
                    // 整帧 CRC16 校验通过后交给 referee_data_solve 解析
                    if (verify_CRC16_check_sum(p->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p->data_len))
                        referee_data_solve(p->protocol_packet);
                }
                break;
            default:
                p->unpack_step = STEP_HEADER_SOF;
                p->index = 0;
                break;
        }
    }
}

/* UART DMA 接收完成回调，将收到的数据压入环形缓冲区（中断上下文，不做解包） */
static void referee_rx_callback(void)
{
    const uint8_t *src = referee_uart->rx_buffer;
    uint16_t       len = referee_uart->rx_buf_length;
    for (uint16_t i = 0; i < len; i++) {
        ring_buf[ring_tail] = src[i];
        ring_tail = (ring_tail + 1) % REFEREE_RING_SIZE;
    }
}

/* 裁判系统任务：初始化后循环从环形缓冲区取字节送入解包状态机 */
void Referee_task(void const *argument)
{
    init_referee_struct_data();
    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
    referee_uart = Uart_register(&huart6, referee_rx_callback); // 注册 USART6 及接收回调

    while (1) {
        /* 排空环形缓冲区，逐字节送入解包状态机 */
        while (ring_head != ring_tail) {
            uint8_t byte = ring_buf[ring_head];
            ring_head = (ring_head + 1) % REFEREE_RING_SIZE;
            referee_unpack_buf(&byte, 1);
        }
        osDelay(1);
    }
}
