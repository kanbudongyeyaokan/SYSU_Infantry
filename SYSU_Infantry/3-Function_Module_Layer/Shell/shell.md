
详细教程暂时看github仓库的ReadMe文件
https://github.com/NevermindZZT/letter-shell?tab=readme-ov-file#%E5%8A%9F%E8%83%BD


Letter Shell 移植模块使用说明
1. 简介
   本模块实现了 Letter Shell 3.x 在 STM32 + FreeRTOS 环境下的移植。 它利用 FreeRTOS 队列 和 DMA 串口接收 实现了高效、无阻塞的命令行交互。

主要特性：

非阻塞发送：使用 bsp_usart 的 DMA 环形缓冲区发送，不占用 CPU。

阻塞式接收：Shell 任务平时处于 Block 状态，仅在串口收到数据时被唤醒，不消耗 CPU 资源。

RTOS 友好：完全适配 FreeRTOS 任务调度。

2. 依赖环境
   在使用本模块前，请确保工程包含以下组件：

Letter Shell 3.x 源码 (shell.c, shell.h 等)。

BSP USART：支持 DMA 和 RingBuffer 的串口驱动 (且必须定义 Uart_instance_t)。

FreeRTOS：需要 task.h 和 queue.h。

链接脚本修改：必须在 .ld 文件中添加 shellCommand 段 (见第 5 节)。

3. 快速接入步骤
   3.1 定义全局串口实例
   在 robot_task.c 或 main.c 中，必须定义一个全局的串口实例指针，供 Shell 模块调用。

C
// robot_task.c
#include "bsp_usart.h"

// 【注意】必须是全局变量，且不能加 static
Uart_instance_t *test_uart = NULL;
3.2 启动 Shell 任务
在你的 Shell 任务函数（例如 Shell_task.c）中，按以下顺序初始化：

C
#include "shell_port.h"
#include "bsp_usart.h"

// 引用外部变量
extern Uart_instance_t *test_uart;

void Shell_task(void const * argument)
{
// 1. 【关键】等待串口初始化完成
// 防止 Robot_task 还没注册串口，Shell 就开始运行导致空指针崩溃
while (test_uart == NULL)
{
osDelay(10);
}

    // 2. 初始化 Shell 底层 (创建队列、绑定回调)
    User_Shell_Init();

    // 3. 打印启动信息 (可选，测试 TX 是否正常)
    char *log = "\r\n[System] Shell Task Started!\r\n";
    Uart_sendData(test_uart, (uint8_t *)log, strlen(log));

    // 4. 进入 Shell 自动托管循环 (死循环)
    shellTask(&shell);
}
3.3 调整任务堆栈
重要：Letter Shell 处理字符串和浮点数需要较大的栈空间。

推荐堆栈大小：512 Words (2048 Bytes) 或更大。

如果堆栈太小 (如 128 Words)，会导致系统 HardFault 或其他任务内存被踩踏。

4. 如何添加自定义命令
   你可以在工程的任意 .c 文件中导出命令，无需修改 Shell 源码。

4.1 普通函数命令 (推荐)
Shell 会自动将输入的字符串转换为参数。

C
// 示例：在 gimbal.c 中修改 PID
#include "shell.h"
#include <stdlib.h> // for atof

// 命令函数原型：int func(int argc, char *argv[])
int set_yaw_pid(int argc, char *argv[])
{
if (argc != 4) {
shellPrint(&shell, "Usage: yaw_pid <p> <i> <d>\r\n");
return -1;
}

    // 字符串转浮点
    float p = atof(argv[1]);
    float i = atof(argv[2]);
    float d = atof(argv[3]);

    // 修改你的全局变量或结构体
    // gimbal.pid.kp = p; ...

    shellPrint(&shell, "Set Yaw PID: %.2f %.2f %.2f\r\n", p, i, d);
    return 0;
}

// 导出命令
// 参数：权限 | 类型, 命令名, 函数名, 描述
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), yaw_pid, set_yaw_pid, Change Yaw PID);
4.2 变量导出
直接在命令行查看或修改全局变量。

int g_speed = 100;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), speed, &g_speed, global speed);

