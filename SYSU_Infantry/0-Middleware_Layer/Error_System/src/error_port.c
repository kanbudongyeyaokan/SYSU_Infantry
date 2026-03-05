/**
  * @file    error_port.c
  * @brief   错误处理系统平台移植层
  *
  * @note    本平台固定配置：
  *          - FreeRTOS 任务管理
  *          - HAL_GetTick() 时间戳
  *          - UART 输出
  *          - 蜂鸣器告警联动
  */

#include "error_handler.h"
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "buzzer_alarm.h"
#include "robot_task.h"

#include "bsp_usart.h"
extern Uart_instance_t* test_uart;

/* 声明获取 UART 句柄的内部函数 */
void* error_get_uart_handle(void);

/* ================= 私有变量 ================= */

/* 输出缓冲区 */
static char error_output_buf[256];

/* 蜂鸣器报警去重标志 */
static const char* buzzer_last_critical_module_name = NULL;
static uint32_t buzzer_last_critical_time = 0;
#define BUZZER_COOLDOWN_MS  2000u

/* ================= 平台相关实现 ================= */

uint16_t error_port_get_task_id(void)
{
    if (osKernelRunning())
    {
        osThreadId tid = osThreadGetId();
        if (tid != NULL)
        {
            return (uint16_t)((uintptr_t)tid & 0xFFFFu);
        }
    }
    return 0u;
}

uint32_t error_port_get_timestamp(void)
{
    return HAL_GetTick();
}

void error_port_output(const error_record_t* record)
{
    if (record == NULL)
    {
        return;
    }

    /* 错误等级字符串 */
    static const char* level_str[] = {
        "INFO",
        "WARN",
        "ERR",
        "CRIT"
    };

    uint8_t level = ERROR_GET_LEVEL(record->error_code);

    /* 格式化输出（固定包含函数名和行号） */
    memset(error_output_buf, 0, sizeof(error_output_buf));

    if (record->function != NULL)
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s] %s:%lu %s",
                 level_str[level],
                 record->module_name ? record->module_name : "UNK",
                 record->function,
                 record->line,
                 record->message);
    }
    else
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s] %s",
                 level_str[level],
                 record->module_name ? record->module_name : "UNK",
                 record->message);
    }

    /* UART 输出 */
    strcat(error_output_buf, "\r\n");
    Uart_instance_t* uart = (Uart_instance_t*)error_get_uart_handle();
    if (uart != NULL)
    {
        Uart_printf(uart, "%s", error_output_buf);
    }

    /* Critical 错误蜂鸣器报警 */
    if (level == ERROR_LEVEL_CRITICAL)
    {
        uint32_t now = HAL_GetTick();
        const char* current_module = record->module_name ? record->module_name : "UNK";

        if (buzzer_last_critical_module_name != current_module ||
            (now - buzzer_last_critical_time) > BUZZER_COOLDOWN_MS)
        {
            buzzer_last_critical_module_name = current_module;
            buzzer_last_critical_time = now;

            if (Buzzer_cmd_queue_handle != NULL)
            {
                uint8_t alarm_code = 99;
                xQueueSend(Buzzer_cmd_queue_handle, &alarm_code, 0);
            }
        }
    }
}
