#include "SBUS.h"

#include <stdio.h>

#include "string.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"

/**SBUS遥控器数据定义区**/
static SBUS_ctrl_t sbus_data[2]; // 0-当前数据，1-上一次数据
static Uart_instance_t *sbus_uart; // 获取SBUS数据的串口实例

/**
 * @brief 矫正SBUS通道的值，对超过660或者小于-660的值进行处理
 */
static void Rectify_sbus_data(void)
{
    int16_t *ch_ptr = &sbus_data[CURRENT].rc.Ch1;
    for (int i = 0; i < 16; ++i)
    {
        if (ch_ptr[i] > 660)
            ch_ptr[i] = 660;
        else if (ch_ptr[i] < -660)
            ch_ptr[i] = -660;
    }
}

/**
 * @brief SBUS协议解析 - 从25字节原始数据解析16通道+开关
 * @param sbus_buf 原生数据指针(25字节)
 * @note SBUS数据格式:
 *       帧头(0x0F) + 22字节通道数据(16×11位) + 标志位 + 帧尾(0x00)
 *       通道数据打包方式: 每11位一个通道，共16通道
 */
static void sbus_to_ctrl(volatile const uint8_t *sbus_buf)
{
    // 安全检查
    if (sbus_buf == NULL)
    {
        return;
    }

    // 检查帧头和帧尾
    if (sbus_buf[0] != 0x0F || sbus_buf[24] != 0x00)
    {
        return; // 帧格式错误，丢弃
    }

    // 解析16个11位通道数据(从第2字节开始，共22字节)
    const uint8_t *data = &sbus_buf[1];
    uint16_t ch_raw[16];

    // 通道1-8
    ch_raw[0]  = ((data[0]    | data[1] << 8))                  & 0x07FF;
    ch_raw[1]  = ((data[1]>>3 | data[2] << 5))                  & 0x07FF;
    ch_raw[2]  = ((data[2]>>6 | data[3] << 2 | data[4]<<10))    & 0x07FF;
    ch_raw[3]  = ((data[4]>>1 | data[5] << 7))                  & 0x07FF;
    ch_raw[4]  = ((data[5]>>4 | data[6] << 4))                  & 0x07FF;
    ch_raw[5]  = ((data[6]>>7 | data[7] << 1 | data[8]<<9))     & 0x07FF;
    ch_raw[6]  = ((data[8]>>2 | data[9] << 6))                  & 0x07FF;
    ch_raw[7]  = ((data[9]>>5 | data[10]<< 3))                  & 0x07FF;

    // 通道9-16
    ch_raw[8]  = ((data[11]   | data[12]<<8))                   & 0x07FF;
    ch_raw[9]  = ((data[12]>>3| data[13]<<5))                   & 0x07FF;
    ch_raw[10] = ((data[13]>>6| data[14]<<2 | data[15]<<10))    & 0x07FF;
    ch_raw[11] = ((data[15]>>1| data[16]<<7))                   & 0x07FF;
    ch_raw[12] = ((data[16]>>4| data[17]<<4))                   & 0x07FF;
    ch_raw[13] = ((data[17]>>7| data[18]<<1 | data[19]<<9))     & 0x07FF;
    ch_raw[14] = ((data[19]>>2| data[20]<<6))                   & 0x07FF;
    ch_raw[15] = ((data[20]>>5| data[21]<<3))                   & 0x07FF;

    // 转换为-660~+660范围(与DBUS兼容)
    // SBUS原始值: 0~2047 (中心值1024)
    // 转换公式: (raw - 1024) * 660 / 1024
    int16_t *ch_ptr = &sbus_data[CURRENT].rc.Ch1;
    for (int i = 0; i < 16; ++i)
    {
        ch_ptr[i] = -(int16_t)((ch_raw[i] - SBUS_CH_VALUE_OFFSET) );
    }

    // 矫正数据范围
    Rectify_sbus_data();

    // 解析开关状态(从标志字节提取)
    uint8_t flag = sbus_buf[23];
    sbus_data[CURRENT].S1 = (flag & 0x03);       // 开关1: bit0-1
    sbus_data[CURRENT].S2 = ((flag >> 2) & 0x03); // 开关2: bit2-3

    // 映射开关值: 0->1(下), 1->2(上), 2->3(中)
    if (sbus_data[CURRENT].S1 == 0) sbus_data[CURRENT].S1 = 1;
    else if (sbus_data[CURRENT].S1 == 1) sbus_data[CURRENT].S1 = 2;
    else sbus_data[CURRENT].S1 = 3;

    if (sbus_data[CURRENT].S2 == 0) sbus_data[CURRENT].S2 = 1;
    else if (sbus_data[CURRENT].S2 == 1) sbus_data[CURRENT].S2 = 2;
    else sbus_data[CURRENT].S2 = 3;

    sbus_data[CURRENT].Frame_flag = flag;

    // 保存上一次的数据，用于按键持续按下和切换的判断
    memcpy(&sbus_data[LAST], &sbus_data[CURRENT], sizeof(SBUS_ctrl_t));

    //输出
    // 打印所有16个通道 + 开关状态
    // printf("Ch1:%d Ch2:%d Ch3:%d Ch4:%d Ch5:%d Ch6:%d Ch7:%d Ch8:%d\r\n",
    //        sbus_data[CURRENT].rc.Ch1, sbus_data[CURRENT].rc.Ch2,
    //        sbus_data[CURRENT].rc.Ch3, sbus_data[CURRENT].rc.Ch4,
    //        sbus_data[CURRENT].rc.Ch5, sbus_data[CURRENT].rc.Ch6,
    //        sbus_data[CURRENT].rc.Ch7, sbus_data[CURRENT].rc.Ch8);
    // printf("Ch9:%d Ch10:%d Ch11:%d Ch12:%d Ch13:%d Ch14:%d Ch15:%d Ch16:%d S1:%d S2:%d\r\n",
    //        sbus_data[CURRENT].rc.Ch9, sbus_data[CURRENT].rc.Ch10,
    //        sbus_data[CURRENT].rc.Ch11, sbus_data[CURRENT].rc.Ch12,
    //        sbus_data[CURRENT].rc.Ch13, sbus_data[CURRENT].rc.Ch14,
    //        sbus_data[CURRENT].rc.Ch15, sbus_data[CURRENT].rc.Ch16,
    //        sbus_data[CURRENT].S1, sbus_data[CURRENT].S2);
    // printf("Ch1:%d Ch15:%d\r\n", sbus_data[CURRENT].rc.Ch1, sbus_data[CURRENT].rc.Ch15);
}

/**
 * @brief 对sbus_to_ctrl的简单封装，用于注册到串口实例的回调函数中
 */
static void SBUS_receive_callback(void)
{
    // 测量SBUS帧间隔
    // static uint32_t dwt_cnt = 0;
    // float dt = DWT_GetDeltaT(&dwt_cnt);  // 返回两次调用的时间间隔(秒)
    // uint32_t interval_us = (uint32_t)(dt * 1000000.0f);  // 转换为微秒
    // printf("SBUS interval: %lu us\n", interval_us);  // 14400 us = 14.4 ms
    
    sbus_to_ctrl(sbus_uart->rx_buffer); // 进行协议解析
}

/**
 * @brief 获取SBUS遥控器数据
 * @param sbus_uart_handle SBUS串口句柄
 * @return SBUS_ctrl_t* 返回SBUS数据数组指针(索引0为当前，索引1为上一次)
 * @note 使用示例:
 *       SBUS_ctrl_t *sbus = SBUS_Data_Get(&huart3);
 *       int16_t ch1 = sbus[CURRENT].rc.Ch1;  // 读取当前通道1
 *       int16_t last_ch1 = sbus[LAST].rc.Ch1; // 读取上一次通道1
 */
SBUS_ctrl_t *SBUS_Data_Get(UART_HandleTypeDef *sbus_uart_handle)
{
    // 注册管理SBUS数据的串口
    // SBUS要求: 100Kbps波特率, 8位数据, 偶校验, 2停止位
    sbus_uart = Uart_register(sbus_uart_handle, SBUS_receive_callback);
    return sbus_data;
}
