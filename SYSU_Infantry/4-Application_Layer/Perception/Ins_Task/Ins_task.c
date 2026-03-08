#include <stdlib.h>

#include "Ins_task.h"
#include "bmi088.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "hwt606_iic.h"
#include "i2c.h"
#include "ins.h"
#include "jy61p_iic.h"
#include "robot_task.h"
#include "vofa.h"
#include "bsp_usart.h" // 用于调试输出
#include "error_handler.h"

void Ins_task(void const *argument)
{
    (void)argument;

    extern I2C_HandleTypeDef hi2c2;
    extern SPI_HandleTypeDef hspi2;
    // 目前兼容四个IMU驱动：BMI088、HWT101、HWT606、JY61P
    // const Ins_driver_interface_t *driver = BMI088_Get_Driver();
    // const Ins_driver_interface_t *driver = HWT101_IIC_Get_Driver(&hi2c2);
    // const Ins_driver_interface_t *driver = JY61P_IIC_Get_Driver(&hi2c2);
    const Ins_driver_interface_t *driver = HWT606_IIC_Get_Driver(&hi2c2); // 获取 HWT606 IIC 驱动接口
    // const Ins_data_t *data;
    // 初始化 INS 层 
    Ins_init(driver);
    (void)hspi2;
    const Ins_data_t *data;

    Ins_init(driver);

    for (;;) {
        Ins_update();

        // 如果需要数据，直接 Ins_get_data()
        // data = Ins_get_data();
        // Uart_printf(test_uart, "Yaw:%.2f, Pitch:%.2f, Temp:%.2f, State:%d\r\n", 
        //             data->euler.yaw, 
        //             data->euler.pitch, 
        //             data->temp, 
        //             data->state);
        //  VOFA_Send(test_uart,
        //       data->euler.yaw,
        //       data->euler.pitch,
        //       data->temp,
        //       (float)data->state); 
         //打印 欧拉角 + Z轴加速度 + 采样时间dt
       /* Uart_printf(test_uart, "Yaw:%.2f, AccZ:%.2f, dt:%.4f\r\n", 
                    data->euler.yaw, 
                     data->acc_body.z, 
                     data->dt_s);  */

        osDelay(2); 
        data = Ins_get_data();
        (void)data;
        osDelay(2);
    }
}

