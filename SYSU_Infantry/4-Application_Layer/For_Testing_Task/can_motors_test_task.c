/**
 * @file    can_motors_test_task.c
 * @brief   CAN电机测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-12
 * @version 1.0
 * 
 * @note    用于测试CAN总线电机功能
 */

#include "can_motors_test_task.h"
 #include "dji_motor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include "bsp_can.h"


UartInstance_t* uart_instance;

static Djimotor_measure_t test_measure;

/**
 * @brief CAN电机测试任务函数
 * @param argument 任务参数（未使用）
 */
void Can_motors_test_task(void const *argument)
{
    // TODO: 初始化电机实例
    // static DJIMotorInstance *test_motor;
    // Motor_Init_Config_s motor_config = {
    //     // 配置参数需要根据实际硬件设置
    // };
    // test_motor = DJIMotorInit(&motor_config);
    uart_instance = Uart_register(&huart1,NULL);

    Djimotor_init_config_t motor_config = {
        .motor_name = "TSET_MOTOR",
        .motor_type = M3508,
        .motor_status = MOTOR_ENABLED,
        .motor_controller_init = {
            .close_loop = OPEN_LOOP,
        },
        .can_init = {
            .can_handle = &hcan1,
            .can_id = 0X200,       // 电机ID (1-8)
            .tx_id = 1,   // 发送ID
            .rx_id = 0x201
            // 接收ID
            }
    };
    Djimotor_device_t *test_motor = DJI_Mobtor_Init(&motor_config);
    /*
    Can_init_t can_config={
        .can_handle = &hcan1,
        .tx_id = 2,
        .rx_id = 0x206,
        .can_id = 0x1ff
    };
    Can_controller_t *can_dev = Can_device_init(&can_config);
    static uint8_t tx_buffer[8]={0,0,0x30,0x30,0,0,0,0};
*/
    static uint32_t test_counter = 0;


    for (;;)
    {
        // TODO: 测试电机控制
        // 可以实现简单的电机转动测试，如正弦波控制等
        
        // 暂时输出测试信息

        //printf("CAN Motors Test Task Running... Counter: %lu\r\n", test_counter);
       // Can_send_data(can_dev,tx_buffer);


       // Uart_printf(debug_uart, "CAN Motors Test Task Running... Counter: %lu\r\n", test_counter);
        Djimotor_set_target(test_motor,500.0);

        test_measure = Djimotor_get_measure(test_motor);



        // Uart_printf(uart_instance, "pid:%.2f\r\n",temp);
        Djimotor_control_all();
        // 任务延时50ms，控制频率200Hz
        osDelay(5);

    }
}
