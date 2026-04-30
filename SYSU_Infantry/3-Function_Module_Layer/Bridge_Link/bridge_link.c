/**
 * @file    bridge_link.c
 * @brief   上位机 Bridge Link 协议 —— 下位机实现
 *
 * 当前功能：
 *   1. CRC16-CCITT 封帧 / 验帧
 *   2. USB CDC 接收回调 → FIFO → 状态机解析
 *   3. 以固定周期向上位机发送测试 CAN 帧（不依赖真实 CAN 总线）
 */

#include "bridge_link.h"

#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp_usb.h"
#include "bsp_can.h"
#include "can.h"           /* hcan1 extern */
#include "error_handler.h"

/* ── CRC16-CCITT 查表法（与上位机 bridge_link.cpp 完全一致）──────────────── */

static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

/**
 * @brief  计算 CRC16-CCITT
 * @param  data  数据起始指针
 * @param  len   字节数
 * @return 16 位校验值
 */
static uint16_t bridge_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/* ── 事件信号量（USB RX / CAN sniffer ISR → 任务）────────────────────────── */

static SemaphoreHandle_t s_rx_sem = NULL;

/* ── 接收 FIFO ─────────────────────────────────────────────────────────────── */

static uint8_t  rx_fifo[BRIDGE_RX_FIFO_SIZE];
static uint16_t rx_head = 0;   /* 写指针（USB 回调写入） */
static uint16_t rx_tail = 0;   /* 读指针（解析任务消费） */

static inline uint16_t fifo_len(void)
{
    if (rx_head >= rx_tail)
        return rx_head - rx_tail;
    return (uint16_t)(BRIDGE_RX_FIFO_SIZE - rx_tail + rx_head);
}

/** 从 FIFO 中以相对偏移 offset 读取 len 字节（不移动 tail） */
static void fifo_peek(uint8_t *dst, uint16_t offset, uint16_t len)
{
    uint16_t idx = (uint16_t)((rx_tail + offset) % BRIDGE_RX_FIFO_SIZE);
    for (uint16_t i = 0; i < len; i++) {
        dst[i] = rx_fifo[idx];
        idx = (uint16_t)((idx + 1) % BRIDGE_RX_FIFO_SIZE);
    }
}

/** 丢弃 FIFO 中最老的 n 字节 */
static inline void fifo_consume(uint16_t n)
{
    rx_tail = (uint16_t)((rx_tail + n) % BRIDGE_RX_FIFO_SIZE);
}

/* ── USB 接收回调 ──────────────────────────────────────────────────────────── */

static void bridge_usb_rx_cb(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return;
    for (uint32_t i = 0; i < len; i++) {
        rx_fifo[rx_head] = buf[i];
        rx_head = (uint16_t)((rx_head + 1) % BRIDGE_RX_FIFO_SIZE);
    }
    if (s_rx_sem != NULL) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_rx_sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ── 初始化 ────────────────────────────────────────────────────────────────── */

void Bridge_Link_Init(void)
{
    memset(rx_fifo, 0, sizeof(rx_fifo));
    rx_head = 0;
    rx_tail = 0;
    s_rx_sem = xSemaphoreCreateBinary();
    Usb_Init(bridge_usb_rx_cb);
    Can_Register_Sniffer(Bridge_CAN_Sniffer_Cb);
}

void Bridge_Link_WaitRx(uint32_t timeout_ms)
{
    if (s_rx_sem != NULL) {
        xSemaphoreTake(s_rx_sem, pdMS_TO_TICKS(timeout_ms));
    }
}

/* ── 发送 CAN 帧 ───────────────────────────────────────────────────────────── */

void Bridge_Send_CAN(uint32_t can_id, uint8_t dlc, const uint8_t *data)
{
    /* 构造 payload（13 字节）*/
    uint8_t payload[BRIDGE_CAN_PAYLOAD];
    /* CAN ID，小端 */
    payload[0] = (uint8_t)(can_id & 0xFF);
    payload[1] = (uint8_t)((can_id >> 8)  & 0xFF);
    payload[2] = (uint8_t)((can_id >> 16) & 0xFF);
    payload[3] = (uint8_t)((can_id >> 24) & 0xFF);
    /* DLC */
    if (dlc > 8) dlc = 8;
    payload[4] = dlc;
    /* Data（不足 8 字节补 0） */
    memset(&payload[5], 0, 8);
    if (data != NULL && dlc > 0) {
        memcpy(&payload[5], data, dlc);
    }

    /* 组装完整帧：SOF + type + len_lo + len_hi + payload */
    uint8_t frame[BRIDGE_CAN_FRAME_LEN];
    frame[0] = BRIDGE_SOF;
    frame[1] = BRIDGE_TYPE_CAN;
    frame[2] = (uint8_t)(BRIDGE_CAN_PAYLOAD & 0xFF);   /* len_lo */
    frame[3] = (uint8_t)(BRIDGE_CAN_PAYLOAD >> 8);     /* len_hi */
    memcpy(&frame[4], payload, BRIDGE_CAN_PAYLOAD);

    /* CRC16 覆盖 SOF..payload（不含 CRC 本身），追加到帧尾 */
    uint16_t crc = bridge_crc16(frame, 4 + BRIDGE_CAN_PAYLOAD);
    frame[4 + BRIDGE_CAN_PAYLOAD]     = (uint8_t)(crc & 0xFF);   /* crc_lo */
    frame[4 + BRIDGE_CAN_PAYLOAD + 1] = (uint8_t)(crc >> 8);     /* crc_hi */

    Usb_Send(frame, BRIDGE_CAN_FRAME_LEN);
}

/* ── 接收解析 ──────────────────────────────────────────────────────────────── */

/**
 * @brief  解析 FIFO 中所有完整的 Bridge 帧
 *
 * 状态机逻辑（与上位机 read_loop 镜像对称）：
 *   等待 SOF → 读 type → 读 len_lo/hi → 读 payload → 读 CRC → 校验 → 分发
 */
void Bridge_Link_Parse(void)
{
    /* 接收侧用于暂存单帧 payload 的静态缓冲，最大支持 BRIDGE_MAX_PAYLOAD */
    static uint8_t pay_buf[BRIDGE_MAX_PAYLOAD];

    while (1) {
        /* ① 至少需要帧头 4 字节才能判断帧类型和长度 */
        if (fifo_len() < 4) break;

        /* ② 检查 SOF */
        uint8_t sof;
        fifo_peek(&sof, 0, 1);
        if (sof != BRIDGE_SOF) {
            fifo_consume(1);    /* 不是帧头，丢弃并重新同步 */
            continue;
        }

        /* ③ 读取 type 和 len */
        uint8_t hdr[4];
        fifo_peek(hdr, 0, 4);  /* [SOF, type, len_lo, len_hi] */
        uint8_t  type    = hdr[1];
        uint16_t pay_len = (uint16_t)(hdr[2] | ((uint16_t)hdr[3] << 8));

        /* ④ 合法性保护 */
        if (pay_len > BRIDGE_MAX_PAYLOAD) {
            fifo_consume(1);    /* 异常帧，跳过 SOF 重新同步 */
            continue;
        }

        /* ⑤ 等待完整帧（4 字节头 + payload + 2 字节 CRC） */
        uint16_t total = (uint16_t)(4 + pay_len + 2);
        if (fifo_len() < total) break;  /* 数据还没到齐，等下次 */

        /* ⑥ 提取 payload */
        if (pay_len > 0) {
            fifo_peek(pay_buf, 4, pay_len);
        }

        /* ⑦ 提取接收到的 CRC（小端） */
        uint8_t crc_bytes[2];
        fifo_peek(crc_bytes, (uint16_t)(4 + pay_len), 2);
        uint16_t recv_crc = (uint16_t)(crc_bytes[0] | ((uint16_t)crc_bytes[1] << 8));

        /* ⑧ 重建原始帧字节序列用于本地 CRC 计算（SOF + type + len + payload）*/
        uint8_t raw[4 + BRIDGE_MAX_PAYLOAD];
        fifo_peek(raw, 0, (uint16_t)(4 + pay_len));
        uint16_t calc_crc = bridge_crc16(raw, (uint16_t)(4 + pay_len));

        /* ⑨ 消费整帧 */
        fifo_consume(total);

        /* ⑩ CRC 校验 */
        if (recv_crc != calc_crc) {
            /* 校验失败，静默丢弃（生产环境可在此触发错误计数） */
            ERROR_WARN("BRIDGE", "CRC Error");
            continue;
        }

        /* ⑪ 分发 */
        if (type == BRIDGE_TYPE_CAN && pay_len == BRIDGE_CAN_PAYLOAD) {
            Bridge_CANPayload_t frame;
            memcpy(&frame, pay_buf, BRIDGE_CAN_PAYLOAD);
            Bridge_On_CAN_Received(&frame);
        }
        /* type == BRIDGE_TYPE_REF 等其他类型暂不处理 */
    }
}

/* ── 下行：上位机 → CAN1 ────────────────────────────────────────────────────── */

/* 覆盖弱符号：将上位机下发的 CAN 帧直接透传到 hcan1 */
void Bridge_On_CAN_Received(const Bridge_CANPayload_t *frame)
{
    uint8_t dlc = (frame->dlc <= 8u) ? frame->dlc : 8u;
    uint8_t ok  = Can_send_raw(&hcan1, frame->id & 0x7FFu, dlc, frame->data);
    if (ok) {
        // ERROR_INFO("BRIDGE", "DN id=0x%03lx dlc=%u d=%02x%02x%02x%02x%02x%02x%02x%02x",
        //            (unsigned long)(frame->id & 0x7FFu), dlc,
        //            frame->data[0], frame->data[1], frame->data[2], frame->data[3],
        //            frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
    } else {
        ERROR_WARN("BRIDGE", "DN tx fail id=0x%03lx mailbox full",
                   (unsigned long)(frame->id & 0x7FFu));
    }
}

/* ── 上行环形缓冲（ISR producer / task consumer）────────────────────────────── */

#define UPLINK_FIFO_CAP  32u

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} UplinkFrame_t;

static UplinkFrame_t uplink_buf[UPLINK_FIFO_CAP];
/* volatile：两侧各自只写自己的那个指针，Cortex-M uint8_t 访问天然原子 */
static volatile uint8_t uplink_head = 0;  /* ISR 写 */
static volatile uint8_t uplink_tail = 0;  /* 任务写 */

/* 上行掉帧计数（调试用） */
static uint32_t uplink_dropped = 0;

/* ── CAN1 sniffer 回调（ISR 上下文）────────────────────────────────────────── */

void Bridge_CAN_Sniffer_Cb(CAN_HandleTypeDef *hcan,
                            uint32_t std_id,
                            uint8_t dlc,
                            const uint8_t *data)
{
    /* 只转发 CAN1 */
    if (hcan != &hcan1) return;

    uint8_t next = (uint8_t)((uplink_head + 1u) % UPLINK_FIFO_CAP);
    if (next == uplink_tail) {
        /* 缓冲满，丢帧并计数 */
        uplink_dropped++;
        return;
    }

    UplinkFrame_t *slot = &uplink_buf[uplink_head];
    slot->id  = std_id;
    slot->dlc = (dlc <= 8u) ? dlc : 8u;
    /* data 指向 Can_fifo_process 的静态局部变量，仅在本回调期间有效，立即拷贝 */
    for (uint8_t i = 0; i < slot->dlc; i++) slot->data[i] = data[i];

    /* 写完数据再更新 head，确保 consumer 看到完整帧 */
    uplink_head = next;

    if (s_rx_sem != NULL) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_rx_sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ── 上行批量发送（任务上下文）──────────────────────────────────────────────── */

void Bridge_Uplink_Drain(void)
{
    /* 先检查 ISR 里是否有掉帧 */
    if (uplink_dropped > 0u) {
        ERROR_WARN("BRIDGE", "UP fifo overflow dropped=%lu", (unsigned long)uplink_dropped);
        uplink_dropped = 0u;
    }

    while (uplink_tail != uplink_head) {
        const UplinkFrame_t *f = &uplink_buf[uplink_tail];
        Bridge_Send_CAN(f->id, f->dlc, f->data);
        // ERROR_INFO("BRIDGE", "UP id=0x%03lx dlc=%u d=%02x%02x%02x%02x%02x%02x%02x%02x",
        //            (unsigned long)f->id, f->dlc,
        //            f->data[0], f->data[1], f->data[2], f->data[3],
        //            f->data[4], f->data[5], f->data[6], f->data[7]);
        uplink_tail = (uint8_t)((uplink_tail + 1u) % UPLINK_FIFO_CAP);
    }
}

/* ── 测试发送（供 FreeRTOS 任务轮询调用）──────────────────────────────────── */

/**
 * @brief  每次调用发送一帧计数递增的测试 CAN 帧
 *
 * 测试帧格式（CAN ID = 0x700, DLC = 8）：
 *   data[0..3] = 本次调用序号（小端 uint32_t）
 *   data[4]    = 0xDE
 *   data[5]    = 0xAD
 *   data[6]    = 0xBE
 *   data[7]    = 0xEF
 *
 * 上位机收到后可通过序号判断丢帧率，通过固定尾部验证数据完整性。
 */
void Bridge_Send_Test_Tick(void)
{
    static uint32_t counter = 0;

    uint8_t data[8];
    data[0] = (uint8_t)(counter & 0xFF);
    data[1] = (uint8_t)((counter >> 8)  & 0xFF);
    data[2] = (uint8_t)((counter >> 16) & 0xFF);
    data[3] = (uint8_t)((counter >> 24) & 0xFF);
    data[4] = 0xDE;
    data[5] = 0xAD;
    data[6] = 0xBE;
    data[7] = 0xEF;

    Bridge_Send_CAN(0x700, 8, data);
    counter++;
}
