#include "Ins_task.h"
#include "ins.h"
#include "cmsis_os.h"
#include <stdio.h>

void Ins_task(void const *argument)
{
    INS_Init();

    uint32_t count = 0;

    for (;;)
    {
        INS_Task();


            const attitude_t* att = INS_Get_Attitude();

            // 使用嵌套结构体访问欧拉角
            printf("Pitch:%.2f Roll:%.2f Yaw:%.2f Temp:%.1f\r\n",
                   att->euler_angles.pitch,
                   att->euler_angles.roll,
                   att->euler_angles.yaw,
                   att->temperature);


        osDelay(1); // 1kHz 频率
    }
}