#include <stdlib.h>

#include "Ins_task.h"
#include "ins.h"
#include "bmi088.h" // 引用驱动头文件
#include "cmsis_os.h"
#include "robot_task.h"
#include "hwt606_iic.h" // 引用 HWT606 IIC 驱动头文件
#include "i2c.h" // 引用 IIC 底层驱动头文件
#include "vofa.h"
#include "bsp_usart.h" // 用于调试输出
#include "error_handler.h"

void Ins_task(void const *argument)
{
    // 获取 C板 BMI088 的标准驱动
    // const Ins_driver_interface_t *driver = BMI088_Get_Driver();
    extern I2C_HandleTypeDef hi2c2;
    extern SPI_HandleTypeDef hspi2;
    // 目前兼容三个IMU驱动：BMI088、HWT101、HWT606
    // const Ins_driver_interface_t *driver = BMI088_Get_Driver();
    // const Ins_driver_interface_t *driver = HWT101_IIC_Get_Driver(&hi2c2);
    const Ins_driver_interface_t *driver = HWT606_IIC_Get_Driver(&hi2c2); // 获取 HWT606 IIC 驱动接口
    // const Ins_data_t *data;
    // 初始化 INS 层 
    Ins_init(driver);
    const Ins_data_t *data;
    for(;;)
    {
        Ins_update();

        // 如果需要数据，直接 Ins_get_data()
        // data = Ins_get_data();
        // Uart_printf(test_uart, "Yaw:%.2f, Pitch:%.2f, Temp:%.2f, State:%d\r\n", 
        //             data->euler.yaw, 
        //             data->euler.pitch, 
        //             data->temp, 
        //             data->state);
        // 反馈数据
        // Uart_printf(test_uart,"yaw_speed:%.2f,%.2f\r\n",data->gyro_body.z,data->gyro_body.x);
         // 将当前的 INS 数据发布给决策层，用于下一帧的闭环控制或逻辑判断 
        osDelay(2); 
    }
}
