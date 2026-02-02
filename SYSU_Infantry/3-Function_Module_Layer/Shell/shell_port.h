#ifndef _SHELL_PORT_H
#define _SHELL_PORT_H

#include "shell.h"

// 暴露 shell 对象，方便其他文件调用 (如 shellPrint)
extern Shell shell;

/**
 * @brief Shell 底层初始化
 * @note  创建队列、注册读写函数、绑定串口回调
 */
void User_Shell_Init(void);


int shell_test_hello(int argc, char *argv[]);
#endif // _SHELL_PORT_H