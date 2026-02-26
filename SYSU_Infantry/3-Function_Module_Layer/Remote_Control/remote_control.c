#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "bsp_wdg.h"
#include "buzzer_alarm.h"
/**遥控器数据定义区**/
static RC_ctrl_t rc_data[2];//0-当前数据 ， 1-上一次数据
static Uart_instance_t *rc_uart;//获取遥控器数据的串口实例
static Watchdog_device_t *rc_wdg; //看门狗实例

/**
 * @brief 遥控器离线回调函数 (由看门狗触发)
 */
static void RC_Offline_Callback(void *arg)
{
    // 遥控器断联，为了安全，必须将数据全部清零
    memset(&rc_data[CURRENT], 0, sizeof(RC_ctrl_t));
    memset(&rc_data[LAST], 0, sizeof(RC_ctrl_t));

    Watchdog_buzzer_alarm("RC");
}


/**
 * @brief 矫正遥控器摇杆的值,对超过660或者小于-660的值进行处理
 *
 */
static void Rectify_rc_data(void)
{
    // 显式赋值，比指针偏移更安全，编译器会自动优化
    int16_t *rockers[] = {
        &rc_data[CURRENT].rc.Lrocker_x,
        &rc_data[CURRENT].rc.Lrocker_y,
        &rc_data[CURRENT].rc.Rrocker_x,
        &rc_data[CURRENT].rc.Rrocker_y,
        &rc_data[CURRENT].rc.dial
    };

    for (uint8_t i = 0; i < 5; ++i)
    {
        if (*rockers[i] > 660)      *rockers[i] = 660;
        else if (*rockers[i] < -660) *rockers[i] = -660;

    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_data: 遥控器数据指针
  * @retval         none
*/
static void sbus_to_rc(volatile const uint8_t *sbus_buf)
{
    // 安全检查
    if (sbus_buf == NULL) return;

    // 备份旧数据
    rc_data[LAST] = rc_data[CURRENT];

    // 喂狗
    if (rc_wdg) Watchdog_feed(rc_wdg);

    /* --- 摇杆数据解析 --- */

    // Channel 0, 1, 2 (正常解析)
    rc_data[CURRENT].rc.Lrocker_x = -(((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET);
    rc_data[CURRENT].rc.Lrocker_y = -((((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET);
    rc_data[CURRENT].rc.Rrocker_x = -((((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET);

    // ---------------- [针对右摇杆Y轴的特殊校准] ----------------
    // 先获取原始的错误值
    int16_t raw_ry = -((((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET);

    // 根据实际情况微调这几个宏
    const int16_t REAL_MID = -200; // 实际松手时的中值
    const int16_t REAL_TOP = 420;  // 实际推到最上面的值
    const int16_t REAL_BOT = -660; // 实际推到最下面的值

    // 分段映射
    if (raw_ry >= REAL_MID)
    {
        // 上半段：将 [REAL_MID, REAL_TOP] 映射到 [0, 660]
        // 公式：Out = 0 + (In - In_Min) * (Out_Range / In_Range)
        float scale = 660.0f / (float)(REAL_TOP - REAL_MID);
        rc_data[CURRENT].rc.Rrocker_y = (int16_t)((float)(raw_ry - REAL_MID) * scale);
    }
    else
    {
        // 下半段：将 [REAL_BOT, REAL_MID] 映射到 [-660, 0]
        float scale = 660.0f / (float)(REAL_MID - REAL_BOT);
        rc_data[CURRENT].rc.Rrocker_y = (int16_t)((float)(raw_ry - REAL_BOT) * scale) - 660;
    }
    // --------------------------------------------------------

    // 拨轮
    rc_data[CURRENT].rc.dial = -(((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);

    // 最后的限幅处理 (防止校准后稍微超出 660)
    Rectify_rc_data();
    rc_data[CURRENT].rc.Rswitch = ((sbus_buf[5] >> 4) & 0x0003);
    rc_data[CURRENT].rc.Lswitch = ((sbus_buf[5] >> 4) & 0x000C) >> 2;

    rc_data[CURRENT].mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc_data[CURRENT].mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc_data[CURRENT].mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc_data[CURRENT].mouse.press_l = sbus_buf[12];
    rc_data[CURRENT].mouse.press_r = sbus_buf[13];
    *(uint16_t *)&rc_data[CURRENT].keyboard = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到串口实例的回调函数中
 *
 */
static void RC_receive_callback()
{
    sbus_to_rc(rc_uart->rx_buffer); // 进行协议解析
}

RC_ctrl_t *RC_Data_Get(UART_HandleTypeDef *rc_uart_handle)
{
    //注册管理遥控器数据的串口,如果是自研板，填加了反相器的串口，C板是串口3
    rc_uart = Uart_register(rc_uart_handle, RC_receive_callback);
    // 注册看门狗
    // 遥控器发送频率 14ms (两帧)，我们设超时时间 30ms (允许丢1帧，丢2帧判离线)
    // 假设 Watchdog 单位是任务周期(1ms)，则 reload_count = 30
    Watchdog_init_t wdg_config = {
        .owner_id = rc_uart,
        .reload_count = 30,
        .online_callback = NULL,
        .callback = RC_Offline_Callback,
        .name = "RC"
    };
    
    rc_wdg = Watchdog_register(&wdg_config);

    return rc_data;
}

// 提供给外部判断遥控器状态
uint8_t RC_Is_Online(void)
{
    return Watchdog_is_online(rc_wdg);
}
