#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.Lrocker_x = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.Lrocker_y = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.Rrocker_x= ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.Rrocker_y= ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.Lswitch= ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.Rswitch= ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    *(uint16_t *)&rc_ctrl->keyboard = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8)); //键盘值

    /*遥控通道值映射*/
    rc_ctrl->rc.Lrocker_x-= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Lrocker_y-= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Rrocker_x-= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Rrocker_y-= RC_CH_VALUE_OFFSET;

}

RC_ctrl_t *RC_Data_Get(RC_ctrl_t *rc_ctrl)
{
  

}