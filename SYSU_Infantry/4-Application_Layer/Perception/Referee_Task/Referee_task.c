/**
 * @file    Referee_task.c
 * @brief   裁判系统处理任务 (验收专用版)
 * @note    包含 User口接收检查 和 图传UI发送测试
 */

#include "Referee_task.h"
#include "referee.h"
#include "cmsis_os.h"  // 包含 osDelay
#include "robot_task.h"


// ---------------------------------------------------------
// 外部引用
// ---------------------------------------------------------
// 假设裁判系统连接的是 UART6，且在 main.c 中已生成 huart6

static Referee_Data_t* referee_data;


// ---------------------------------------------------------
// 任务主体
// ---------------------------------------------------------
void Referee_task(void const * argument)
{
    // =====================================================
    // 硬件初始化
    // =====================================================
    // 启动裁判系统串口接收
    referee_data = Referee_Get_Data(&huart3);

    // 延时等待系统稳定，防止上电瞬间数据不稳定
    osDelay(500);

    // =====================================================
    // 任务主循环
    // =====================================================
    for(;;)
    {

        // Uart_printf(test_uart, "Online:%d, HP:%d\r\n",
        //             referee_data->is_online,
        //             referee_data->robot_status.current_hp);

        // Referee_Send_UI_Test();

        osDelay(200);
    }
}