/**
 * @file    rc_test_task.c
 * @brief   遥控器测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 1.0
 * 
 * @note    用于单独测试遥控器功能
 */

#include "rc_test_task.h"
// #include "remote_control.h"
// #include "bsp_log.h"
#include "bsp_usart.h"
#include <stdio.h>
#include "bsp_can.h"

//extern UartInstance_t *uart_instance;

/**
 * @brief 遥控器测试任务函数
 * @param argument 任务参数（未使用）
 */
void Rc_test_task(void const *argument)
{
    // TODO: 初始化遥控器实例
    // static RemoteControlInstance *rc_instance;
    // RC_Init_Config_s rc_config = {
    //     // 配置参数需要根据实际硬件设置
    // };
    // rc_instance = RemoteControlInit(&rc_config);
    //定义CAN设备初始化结构体
    Can_init_t can_config =
    {
        .can_handle = &hcan1,
        .can_id     = 0x1FF,
        .tx_id      = 2,
        .rx_id      = 0x206
    };
    //定义CAN设备结构体
   // Can_controller_t *can_dev = Can_device_init(&can_config);
   // uint8_t tx_0x1ff_buffer[8]={0x11,0x32,0xff,0x10,0,0,0,0};//只有前两个字节有用


    for (;;)
    {
        // TODO: 读取遥控器数据并输出
        // RC_ctrl_t *rc_data = RemoteControlGetData(rc_instance);
        // if (rc_data != NULL)
        // {
        //     // 输出摇杆数据
        //     printf("CH0: %d, CH1: %d, CH2: %d, CH3: %d\r\n", 
        //            rc_data->rc.ch[0], rc_data->rc.ch[1], 
        //            rc_data->rc.ch[2], rc_data->rc.ch[3]);
        //     
        //     // 输出开关状态
        //     printf("S1: %d, S2: %d\r\n", rc_data->rc.s[0], rc_data->rc.s[1]);
        //     
        //     // 输出鼠标数据（如果有）
        //     printf("Mouse: X=%d, Y=%d, Z=%d, Press_L=%d, Press_R=%d\r\n",
        //            rc_data->mouse.x, rc_data->mouse.y, rc_data->mouse.z,
        //            rc_data->mouse.press_l, rc_data->mouse.press_r);
        // }
        
        // 暂时输出测试信息
       // Uart_printf(uart_instance,"Hello World\r\n");
        //调用发送函数
      //  Can_send_data(can_dev,tx_0x1ff_buffer);
        // 任务延时20ms，50Hz频率
        osDelay(20);
    }
}
