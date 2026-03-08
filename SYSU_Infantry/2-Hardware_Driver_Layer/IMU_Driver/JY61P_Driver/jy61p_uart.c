/**
 * @file    jy61p_uart.c
 * @brief   JY61P 串口协议解析实现
 *
 * 帧格式：
 *   0x55 TYPE D1L D1H D2L D2H D3L D3H D4L D4H SUM
 *   SUM = 前 10 字节累加和低 8 位
 *
 * 支持类型：
 *   0x51: 加速度 + 温度
 *   0x52: 角速度 + 电压字段
 *   0x53: 欧拉角 + 版本号
 */

#include "jy61p_uart.h"

#include <string.h>

#include "error_handler.h"

#define JY61P_UART_HEADER         0x55u
#define JY61P_UART_FRAME_LEN      11u
#define JY61P_UART_TYPE_ACC       0x51u
#define JY61P_UART_TYPE_GYRO      0x52u
#define JY61P_UART_TYPE_ANGLE     0x53u
#define JY61P_UART_STREAM_BUF_LEN 256u
#define JY61P_GRAVITY             9.80665f

typedef struct {
    Uart_instance_t *uart_inst; /* 来自 bsp_usart 的实例 */
    JY61P_UART_Data_t data;

    uint8_t stream_buf[JY61P_UART_STREAM_BUF_LEN]; /* 流式拼帧缓存 */
    uint16_t stream_len;                            /* 当前缓存有效长度 */

    bool is_ready;
} JY61P_UART_Driver_t;

static JY61P_UART_Driver_t jy61p_uart_dev;

static int16_t JY61P_ToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8u) | low);
}

static uint8_t JY61P_Frame_Checksum(const uint8_t *frame)
{
    uint8_t sum = 0u;
    uint8_t i = 0u;
    for (i = 0u; i < (JY61P_UART_FRAME_LEN - 1u); i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return sum;
}

static void JY61P_Parse_One_Frame(const uint8_t *frame)
{
    const float k_acc = 16.0f / 32768.0f * JY61P_GRAVITY;
    const float k_gyro = 2000.0f / 32768.0f;
    const float k_angle = 180.0f / 32768.0f;

    int16_t d1 = JY61P_ToInt16(frame[2], frame[3]);
    int16_t d2 = JY61P_ToInt16(frame[4], frame[5]);
    int16_t d3 = JY61P_ToInt16(frame[6], frame[7]);
    int16_t d4 = JY61P_ToInt16(frame[8], frame[9]);

    switch (frame[1]) {
        case JY61P_UART_TYPE_ACC:
            /*
             * 与 I2C 驱动保持同样的坐标轴映射：
             * body_x <- sensor_y, body_y <- sensor_x, body_z <- sensor_z
             */
            jy61p_uart_dev.data.acc_mps2.x = (float)d2 * k_acc;
            jy61p_uart_dev.data.acc_mps2.y = (float)d1 * k_acc;
            jy61p_uart_dev.data.acc_mps2.z = (float)d3 * k_acc;
            jy61p_uart_dev.data.temp_deg_c = (float)d4 * 0.01f;
            break;

        case JY61P_UART_TYPE_GYRO:
            jy61p_uart_dev.data.gyro_dps.x = (float)d2 * k_gyro;
            jy61p_uart_dev.data.gyro_dps.y = (float)d1 * k_gyro;
            jy61p_uart_dev.data.gyro_dps.z = (float)d3 * k_gyro;
            jy61p_uart_dev.data.voltage_raw = (uint16_t)d4;
            break;

        case JY61P_UART_TYPE_ANGLE:
            jy61p_uart_dev.data.euler_deg.x = (float)d2 * k_angle;
            jy61p_uart_dev.data.euler_deg.y = (float)d1 * k_angle;
            jy61p_uart_dev.data.euler_deg.z = (float)d3 * k_angle;
            jy61p_uart_dev.data.version = (uint16_t)d4;
            jy61p_uart_dev.data.update_count++;
            break;

        default:
            break;
    }

    jy61p_uart_dev.data.frame_count++;
}

static void JY61P_Parse_Stream_Buffer(void)
{
    uint16_t idx = 0u;

    /* 从缓存中连续提取 11 字节完整帧。 */
    while ((uint16_t)(jy61p_uart_dev.stream_len - idx) >= JY61P_UART_FRAME_LEN) {
        const uint8_t *frame = &jy61p_uart_dev.stream_buf[idx];

        if (frame[0] != JY61P_UART_HEADER) {
            idx++;
            continue;
        }

        if (JY61P_Frame_Checksum(frame) != frame[JY61P_UART_FRAME_LEN - 1u]) {
            /* 校验失败时只前进 1 字节，继续搜索下一处帧头。 */
            jy61p_uart_dev.data.crc_error_count++;
            idx++;
            continue;
        }

        JY61P_Parse_One_Frame(frame);
        idx = (uint16_t)(idx + JY61P_UART_FRAME_LEN);
    }

    if (idx > 0u) {
        const uint16_t remain = (uint16_t)(jy61p_uart_dev.stream_len - idx);
        if (remain > 0u) {
            memmove(jy61p_uart_dev.stream_buf, &jy61p_uart_dev.stream_buf[idx], remain);
        }
        /* 把未处理完的尾部残帧挪到起始位置，等待下次拼包。 */
        jy61p_uart_dev.stream_len = remain;
    }
}

static void JY61P_UART_Rx_Callback(void)
{
    uint16_t rx_len;
    uint16_t free_len;
    uint16_t copy_len;

    if (!jy61p_uart_dev.is_ready || jy61p_uart_dev.uart_inst == NULL) {
        return;
    }

    rx_len = jy61p_uart_dev.uart_inst->rx_data_len;
    if (rx_len == 0u) {
        return;
    }

    free_len = (uint16_t)(JY61P_UART_STREAM_BUF_LEN - jy61p_uart_dev.stream_len);
    if (rx_len > free_len) {
        /* 缓存溢出时保留最新一半数据，优先保证后续实时性。 */
        const uint16_t keep = (uint16_t)(JY61P_UART_STREAM_BUF_LEN / 2u);
        if (jy61p_uart_dev.stream_len > keep) {
            memmove(jy61p_uart_dev.stream_buf,
                    &jy61p_uart_dev.stream_buf[jy61p_uart_dev.stream_len - keep],
                    keep);
            jy61p_uart_dev.stream_len = keep;
        } else {
            jy61p_uart_dev.stream_len = 0u;
        }
        free_len = (uint16_t)(JY61P_UART_STREAM_BUF_LEN - jy61p_uart_dev.stream_len);
    }

    copy_len = rx_len;
    if (copy_len > free_len) {
        copy_len = free_len;
    }

    memcpy(&jy61p_uart_dev.stream_buf[jy61p_uart_dev.stream_len],
           jy61p_uart_dev.uart_inst->rx_buffer,
           copy_len);
    jy61p_uart_dev.stream_len = (uint16_t)(jy61p_uart_dev.stream_len + copy_len);

    JY61P_Parse_Stream_Buffer();
}

JY61P_UART_Data_t *JY61P_UART_Init(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL) {
        ERROR_RAISE("JY61P", "UART handle is NULL");
        return NULL;
    }

    memset(&jy61p_uart_dev, 0, sizeof(jy61p_uart_dev));
    jy61p_uart_dev.uart_inst = Uart_register(uart_handle, JY61P_UART_Rx_Callback);

    if (jy61p_uart_dev.uart_inst == NULL) {
        ERROR_CRITICAL("JY61P", "Uart_register failed");
        return NULL;
    }

    jy61p_uart_dev.is_ready = true;
    return &jy61p_uart_dev.data;
}

JY61P_UART_Data_t *JY61P_UART_GetData(void)
{
    return &jy61p_uart_dev.data;
}
