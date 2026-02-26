#include <stdlib.h>

#include "Ins_task.h"
#include "ins.h"
#include "bmi088.h" // 引用驱动头文件
#include "cmsis_os.h"
#include "robot_task.h"
#include "hwt606_iic.h" // 引用 HWT606 IIC 驱动头文件
#include "i2c.h" // 引用 IIC 底层驱动头文件

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
    // data = Ins_get_data();
    for(;;)
    {
        Ins_update();

        // 如果需要数据，直接 Ins_get_data()

        // Uart_printf(test_uart,"yaw:%.2f,pitch:%.2f,state:%d\r\n",data->euler.yaw,data->euler.pitch,data->state);
        // 打印 欧拉角 + Z轴加速度 + 采样时间dt
        // Uart_printf(test_uart, "Yaw:%.2f, AccZ:%.2f, dt:%.4f\r\n", 
        //             data->euler.yaw, 
        //             data->acc_body.z, 
        //             data->dt_s);

        osDelay(1); 
    }
}
