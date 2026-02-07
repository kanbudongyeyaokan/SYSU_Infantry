#include <stdlib.h>

#include "ins_task.h"
#include "ins.h"
#include "bmi088.h" // 引用驱动头文件
#include "cmsis_os.h"
#include "robot_task.h"

void Ins_task(void const *argument)
{
    // 获取 C板 BMI088 的标准驱动
    const Ins_driver_interface_t *driver = BMI088_Get_Driver();
    const Ins_data_t *data;
    // 初始化 INS 层 
    Ins_init(driver);

    for(;;)
    {
        Ins_update();
        data = Ins_get_data();
        // 如果需要数据，直接 Ins_get_data()

         Uart_printf(test_uart,"yaw:%.2f,pitch:%.2f,state:%d\r\n",data->euler.yaw,data->euler.pitch,data->state);
        // 打印 欧拉角 + Z轴加速度 + 采样时间dt
        // Uart_printf(test_uart, "Yaw:%.2f, AccZ:%.2f, dt:%.4f\r\n", 
        //             data->euler.yaw, 
        //             data->acc_body.z, 
        //             data->dt_s);

        osDelay(1); 
    }
}
