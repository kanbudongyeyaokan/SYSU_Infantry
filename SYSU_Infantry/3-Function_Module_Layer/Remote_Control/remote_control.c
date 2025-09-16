#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"

/**遥控器数据定义区**/
static RC_ctrl_t rc_data[2];//0-当前数据 ， 1-上一次数据
static Uart_instance_t *rc_uart;//获取遥控器数据的串口实例

/**
 * @brief 矫正遥控器摇杆的值,对超过660或者小于-660的值进行处理
 *
 */
static void Rectify_rc_data() {
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (*(&rc_data[CURRNET].rc.Lrocker_x+ i) > 660)
            *(&rc_data[CURRNET].rc.Lrocker_x + i) = 660;
        else if (*(&rc_data[CURRNET].rc.Lrocker_x + i) < -660)
            *(&rc_data[CURRNET].rc.Lrocker_x + i) = -660;
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
    //安全检查
    if (sbus_buf == NULL ) // rc_data 是已经静态分配好的内存，不需要检查
    {
        return;
    }
    /*数据解析与处理*/
    //摇杆数据
    rc_data[CURRNET].rc.Lrocker_x = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff)-RC_CH_VALUE_OFFSET;        //!< Channel 0
    rc_data[CURRNET].rc.Lrocker_y = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff)-RC_CH_VALUE_OFFSET; //!< Channel 1
    rc_data[CURRNET].rc.Rrocker_x= (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff)-RC_CH_VALUE_OFFSET;
    rc_data[CURRNET].rc.Rrocker_y= (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff)-RC_CH_VALUE_OFFSET; //!< Channel 3
    Rectify_rc_data();
    //左右开关数据
    rc_data[CURRNET].rc.Lswitch= ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_data[CURRNET].rc.Rswitch= ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    //鼠标数据解析
    rc_data[CURRNET].mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_data[CURRNET].mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_data[CURRNET].mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_data[CURRNET].mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_data[CURRNET].mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?

    //键盘数据
    *(uint16_t *)&rc_data[CURRNET].keyboard = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8)); //键盘值


    // 保存上一次的数据,用于按键持续按下和切换的判断
    memcpy(&rc_data[LAST], &rc_data[CURRNET], sizeof(RC_ctrl_t));

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
    return rc_data;
}
