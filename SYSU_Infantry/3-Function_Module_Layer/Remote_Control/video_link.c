#include "video_link.h"
#include "crc_referee.h"
#include "string.h"

static Uart_instance_t *video_uart = NULL;
static uint8_t tx_seq = 0;

static Video_RC_ctrl_t vrc_data[2];
static Uart_instance_t *vrc_uart = NULL;

#define VIDEO_RC_FRAME_LEN 21u

// void Video_Link_Init(UART_HandleTypeDef *huart) {
//     video_uart = Uart_register(huart, NULL);
// }

void Video_link_ReceiveRCData(uint8_t *data, uint16_t length){
    if (length > 21 ) length = 21;

    video_rc_data_t rc_data;
    rc_data.sof = 0x53A9;  // little-endian: A9 53
    memcpy(rc_data.data, data, length);
    //rc_data.crc16 = crc_16_ccitt_false((uint8_t *)&rc_data, offsetof(video_rc_data_t, crc16));

}

void Video_Link_SendCustomData(uint8_t *data, uint16_t length) {
    if (length > 30) length = 30;

    video_link_frame_t frame;
    frame.sof         = 0xA5;
    frame.data_length = length;
    frame.seq         = tx_seq++;
    frame.cmd_id      = CMD_ROBOT_TO_CONTROLLER;
    memcpy(frame.data, data, length);
    frame.crc8  = crc_8((uint8_t *)&frame, 4);

    uint16_t frame_len = 5 + 2 + length;
    frame.crc16 = crc_16((uint8_t *)&frame, frame_len);

    Uart_sendData(video_uart, (uint8_t *)&frame, frame_len + 2);
}

// void Video_Link_SetChannel(uint8_t channel) {
//     video_link_frame_t frame;
//     frame.sof         = 0xA5;
//     frame.data_length = 1;
//     frame.seq         = tx_seq++;
//     frame.cmd_id      = CMD_SET_VIDEO_CHANNEL;
//     frame.data[0]     = channel;
//     frame.crc8  = crc_8((uint8_t *)&frame, 4);
//     frame.crc16 = crc_16((uint8_t *)&frame, 8);

//     Uart_sendData(video_uart, (uint8_t *)&frame, 10);
// }

static const uint8_t *video_rc_find_frame(volatile const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < VIDEO_RC_FRAME_LEN)
    {
        return NULL;
    }

    for (uint16_t i = 0; i + VIDEO_RC_FRAME_LEN <= len; ++i)
    {
        uint8_t *frame = (uint8_t *)&buf[i];

        if (frame[0] != 0xA9 || frame[1] != 0x53)
        {
            continue;
        }

        if (Verify_CRC16_Check_Sum(frame, VIDEO_RC_FRAME_LEN))
        {
            return frame;
        }
    }

    return NULL;
}

static void video_rc_parse(volatile const uint8_t *buf, uint16_t len)
{
    const uint8_t *frame = video_rc_find_frame(buf, len);
    if (frame == NULL) return;

    vrc_data[LAST] = vrc_data[CURRENT];
    const uint8_t *d = &frame[2];

#define CH(raw) ((int16_t)((int16_t)(raw) - 1024))
    vrc_data[CURRENT].rc.Rrocker_x  = CH((d[0] | d[1]<<8) & 0x7FF);
    vrc_data[CURRENT].rc.Rrocker_y  = CH(((d[1]>>3) | (d[2]<<5)) & 0x7FF);
    vrc_data[CURRENT].rc.Lrocker_y  = CH(((d[2]>>6) | (d[3]<<2) | (d[4]<<10)) & 0x7FF);
    vrc_data[CURRENT].rc.Lrocker_x  = CH(((d[4]>>1) | (d[5]<<7)) & 0x7FF);
    vrc_data[CURRENT].rc.mode_switch = (d[5]>>4) & 0x03;
    vrc_data[CURRENT].rc.pause       = (d[5]>>6) & 0x01;
    vrc_data[CURRENT].rc.btn_left    = (d[5]>>7) & 0x01;
    vrc_data[CURRENT].rc.btn_right   = d[6] & 0x01;
    vrc_data[CURRENT].rc.dial        = CH(((d[6]>>1) | (d[7]<<7)) & 0x7FF);
    vrc_data[CURRENT].rc.trigger     = (d[7]>>4) & 0x01;
    vrc_data[CURRENT].mouse.x        = (int16_t)(d[8]  | d[9]<<8);
    vrc_data[CURRENT].mouse.y        = (int16_t)(d[10] | d[11]<<8);
    vrc_data[CURRENT].mouse.z        = (int16_t)(d[12] | d[13]<<8);
    vrc_data[CURRENT].mouse.press_l  = d[14] & 0x03;
    vrc_data[CURRENT].mouse.press_r  = (d[14]>>2) & 0x03;
    vrc_data[CURRENT].mouse.press_m  = (d[14]>>4) & 0x03;
    vrc_data[CURRENT].keyboard       = (uint16_t)(d[15] | d[16]<<8);
#undef CH
}

static void video_rc_callback(void)
{
    if (vrc_uart == NULL)
    {
        return;
    }

    video_rc_parse(vrc_uart->rx_buffer, vrc_uart->rx_data_len);
}

Video_RC_ctrl_t *Video_RC_Data_Get(UART_HandleTypeDef *huart)
{
    vrc_uart = Uart_register(huart, video_rc_callback);
    return vrc_data;
}
