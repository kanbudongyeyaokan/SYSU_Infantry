#include <stdlib.h>

#include "ins_task.h"
#include "ins.h"
#include "bmi088.h" // 引用驱动头文件
#include "cmsis_os.h"
#include "robot_task.h"
#include "hwt101_iic.h" // 【新增】
#include "main.h"       // 为了获取 hi2c2



void Ins_task(void const *argument)
{
    // 获取 C板 BMI088 的标准驱动
    // const Ins_driver_interface_t *driver = BMI088_Get_Driver();

    // 2. (新) 使用 HWT101 (I2C DMA)
    extern I2C_HandleTypeDef hi2c2; // 引用 CubeMX 生成的句柄
    const Ins_driver_interface_t *driver = HWT101_Get_Driver(&hi2c2);

    const Ins_data_t *data;
    // 初始化 INS 层 
    Ins_init(driver);

    for(;;)
    {
        Ins_update();
        data = Ins_get_data();
        // 如果需要数据，直接 Ins_get_data()

         Uart_printf(test_uart,"yaw:%.2f,yaw_speed:%.2f,state:%d\r\n",data->euler.yaw,data->gyro_body.z,data->state);
        // 打印 欧拉角 + Z轴加速度 + 采样时间dt
        // Uart_printf(test_uart, "Yaw:%.2f, AccZ:%.2f, dt:%.4f\r\n", 
        //             data->euler.yaw, 
        //             data->acc_body.z, 
        //             data->dt_s);

        osDelay(5); 
    }
}
