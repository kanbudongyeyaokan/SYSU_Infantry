/**
  * @file    error_port.c
  * @brief   错误处理系统平台移植层
  *
  * @note    本平台固定配置：
  *          - FreeRTOS 任务管理
  *          - HAL_GetTick() 时间戳
  *          - UART 输出
  *          - 蜂鸣器告警联动
  *          - 环形缓冲区：32 条记录
  */

#include "error_handler.h"
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "buzzer_alarm.h"
#include "robot_task.h"

#include "bsp_usart.h"
#include "bsp_rtt.h"
extern Uart_instance_t* test_uart;

/* 声明获取 UART 句柄的内部函数 */
void* error_get_uart_handle(void);

/* 声明设置 Critical 标志的函数 */
void error_set_critical_flag(void);

/* ================= 私有变量 ================= */

/* 输出缓冲区 */
static char error_output_buf[256];

/* 蜂鸣器报警去重标志 */
static const char* buzzer_last_critical_module_name = NULL;
static uint32_t buzzer_last_critical_time = 0;
#define BUZZER_COOLDOWN_MS  2000u

static uint8_t error_port_in_isr(void)
{
    return (__get_IPSR() != 0u) ? 1u : 0u;
}

/* ================= 平台相关实现 ================= */

uint16_t error_port_get_task_id(void)
{
    if (error_port_in_isr())
    {
        return 0u;
    }

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
    uint8_t in_isr = error_port_in_isr();
    if (level > ERROR_LEVEL_CRITICAL)
    {
        level = ERROR_LEVEL_ERROR;
    }

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
    if (uart != NULL && !in_isr)
    {
        Uart_printf(uart, "%s", error_output_buf);
    }

    /* RTT 输出 */
    Rtt_Printf(0, "%s", error_output_buf);

    /* Critical 错误处理：设置标志 + 蜂鸣器报警 */
    if (level == ERROR_LEVEL_CRITICAL)
    {
        /* 设置 Critical 标志 */
        error_set_critical_flag();

        /* 蜂鸣器报警（带去重） */
        uint32_t now = HAL_GetTick();
        const char* current_module = record->module_name ? record->module_name : "UNK";

        /* 使用 strcmp 比较字符串内容，而不是指针地址 */
        if (buzzer_last_critical_module_name == NULL ||
            strcmp(buzzer_last_critical_module_name, current_module) != 0 ||
            (now - buzzer_last_critical_time) > BUZZER_COOLDOWN_MS)
        {
            buzzer_last_critical_module_name = current_module;
            buzzer_last_critical_time = now;

            if (Buzzer_cmd_queue_handle != NULL)
            {
                uint8_t alarm_code = 99;
                if (in_isr)
                {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    xQueueSendFromISR(Buzzer_cmd_queue_handle, &alarm_code, &xHigherPriorityTaskWoken);
                }
                else
                {
                    xQueueSend(Buzzer_cmd_queue_handle, &alarm_code, 0);
                }
            }
        }
    }
}
