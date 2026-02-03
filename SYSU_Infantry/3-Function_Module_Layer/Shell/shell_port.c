/**
 * @file    shell_port.c
 * @brief   Letter Shell 3.x 移植接口 (修正版)
 */

#include "shell.h"
#include "shell_port.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stdlib.h> // 用于 atof
#include <string.h> // 用于 strcmp

// ===================================================
// 外部引用
// ===================================================
// 引用 robot_task.c 中的全局串口实例
extern Uart_instance_t *test_uart;

// ===================================================
// 变量定义
// ===================================================
Shell shell;
char shell_buffer[512];
static QueueHandle_t shell_rx_queue = NULL;

// ===================================================
// 接口实现
// ===================================================

/**
 * @brief Shell 写函数
 * @note  【修正】返回类型改为 short，匹配 shell.h 定义
 */
short userShellWrite(char *data, unsigned short len)
{
    if (test_uart == NULL) return 0;

    // 调用 BSP 发送接口
    Uart_sendData(test_uart, (uint8_t *)data, len);

    return len;
}

/**
 * @brief Shell 读函数 (阻塞式)
 * @note  【修正】返回类型改为 short，匹配 shell.h 定义
 */
short userShellRead(char *data, unsigned short len)
{
    if (shell_rx_queue == NULL) return 0;

    unsigned short read_cnt = 0;
    char ch;

    while (read_cnt < len)
    {
        // 第一个字节死等，后续字节不等
        TickType_t timeout = (read_cnt == 0) ? portMAX_DELAY : 0;

        if (xQueueReceive(shell_rx_queue, &ch, timeout) == pdPASS)
        {
            data[read_cnt++] = ch;
        }
        else
        {
            break;
        }
    }
    return read_cnt;
}

/**
 * @brief 串口接收回调 (挂载到 bsp_usart)
 */
void Shell_Rx_Callback(void)
{
    if (test_uart == NULL || shell_rx_queue == NULL) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t len = test_uart->rx_data_len;

    for (uint16_t i = 0; i < len; i++)
    {
        xQueueSendFromISR(shell_rx_queue, &test_uart->rx_buffer[i], &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Shell 初始化
 */
void User_Shell_Init(void)
{
    // 1. 创建队列
    if (shell_rx_queue == NULL) {
        shell_rx_queue = xQueueCreate(64, sizeof(char));
    }

    // 2. 注册读写接口
    // 现在类型匹配了，不会再报错
    shell.write = userShellWrite;
    shell.read = userShellRead;

    // 3. 初始化 Shell 核心
    shellInit(&shell, shell_buffer, 512);

    // 4. 挂载回调
    // 确保 robot_task.c 里已经完成了 Uart_register 并且 test_uart 不为 NULL
    if (test_uart != NULL)
    {
        test_uart->receive_callback = Shell_Rx_Callback;
    }
}


/********测试部分*******/
// 1. 定义一个简单的测试函数
int shell_test_hello(int argc, char *argv[])
{
    // 打印一句话
    shellPrint(&shell, "Hello RoboMaster! This is a test.\r\n");

    // 如果有参数，打印参数
    if (argc > 1) {
        shellPrint(&shell, "You said: %s\r\n", argv[1]);
    }
    return 0;
}

// 2. 导出这个命令
// 权限:0, 类型:MAIN, 命令名:hello, 函数名:shell_test_hello, 描述:Test command
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), hello, shell_test_hello, print hello);