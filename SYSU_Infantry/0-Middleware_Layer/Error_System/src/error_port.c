/**
  * @file    error_port.c
  * @brief   错误处理系统平台移植层
  *
  * 本文件需要根据实际平台修改：
  * - FreeRTOS 任务获取
  * - 时间戳获取
  * - 输出方式 (RTT/UART)
  */

#include "error_handler.h"
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

/* ================= 平台相关配置 ================= */

/* 包含 FreeRTOS 头文件 */

/* 包含 RTT 头文件 */
#if ERROR_OUTPUT_RTT
#include "SEGGER_RTT.h"
#endif

/* 包含 UART 头文件 */
#if ERROR_OUTPUT_UART
#include "bsp_usart.h"
extern Uart_instance_t* test_uart;
#endif

/* 包含 GPIO 头文件 */
#include "main.h"

/* 包含蜂鸣器告警头文件 */
#include "buzzer_alarm.h"
#include "robot_task.h"  // 获取队列句柄

/* 声明获取 UART 句柄的内部函数 */
void* error_get_uart_handle(void);

/* ================= 私有变量 ================= */

/* 输出缓冲区 */
static char error_output_buf[256];

/* 蜂鸣器报警去重标志 - 防止同一错误重复触发 */
static uint8_t buzzer_last_critical_module = 0;
static uint32_t buzzer_last_critical_time = 0;
#define BUZZER_COOLDOWN_MS  2000u  // 同一模块 Critical 错误，2 秒内只报警一次

/**
  * @brief  获取当前任务 ID
  * @retval 任务标识符
  */
uint16_t error_port_get_task_id(void)
{
    if (osKernelRunning())
    {
        osThreadId tid = osThreadGetId();
        if (tid != NULL)
        {
            /* 返回任务地址低 16 位作为 ID */
            return (uint16_t)((uintptr_t)tid & 0xFFFFu);
        }
    }
    return 0u; /* 裸机或任务未启动 */
}

/**
  * @brief  获取系统时间戳 (ms)
  * @retval 毫秒级时间戳
  */
uint32_t error_port_get_timestamp(void)
{
    /* 使用 HAL_GetTick() */
    return HAL_GetTick();
}

/**
  * @brief  格式化输出错误信息
  * @param  record: 错误记录
  */
void error_port_output(const error_record_t* record) {
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

    /* 模块名称字符串 */
    static const char* module_str[] = {
        "SYS",     /* 0x00 */
        "CAN",     /* 0x01 */
        "MOTOR",   /* 0x02 */
        "IMU",     /* 0x03 */
        "GIMBAL",  /* 0x04 */
        "CHASSIS", /* 0x05 */
        "SHOOT",   /* 0x06 */
        "REFEREE", /* 0x07 */
        "REMOTE",  /* 0x08 */
        "POWER",   /* 0x09 */
        "AUDIO",   /* 0x0A */
        "USB",     /* 0x0B */
        "FREE",    /* 0x0C */
        "UNK",     /* 0x0D */
        "UNK",     /* 0x0E */
        "UNK"      /* 0x0F */
    };

    uint8_t module = ERROR_GET_MODULE(record->error_code);
    uint8_t level = ERROR_GET_LEVEL(record->error_code);
    uint16_t error_id = ERROR_GET_ID(record->error_code);

    /* 格式化输出 */
    memset(error_output_buf, 0, sizeof(error_output_buf));

#if ERROR_USE_SOURCE_INFO
    if (record->function != NULL)
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s][0x%04X] %s:%lu %s",
                 level_str[level],
                 module_str[module > 15 ? 15 : module],
                 error_id,
                 record->function,
                 record->line,
                 record->message);
    }
    else
#endif
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s][0x%04X] %s",
                 level_str[level],
                 module_str[module > 15 ? 15 : module],
                 error_id,
                 record->message);
    }


#if ERROR_OUTPUT_UART
    strcat(error_output_buf, "\r\n");
    Uart_instance_t* uart = (Uart_instance_t*)error_get_uart_handle();
    if (uart != NULL)
    {
        Uart_printf(uart, "%s", error_output_buf);
    }
#endif

    /* Critical 错误时蜂鸣器报警（发送队列消息，非阻塞）*/
    if (level == ERROR_LEVEL_CRITICAL)
    {
        /* 去重检查：同一模块 2 秒内只报警一次 */
        uint32_t now = HAL_GetTick();
        if (buzzer_last_critical_module != module ||
            (now - buzzer_last_critical_time) > BUZZER_COOLDOWN_MS)
        {
            buzzer_last_critical_module = module;
            buzzer_last_critical_time = now;

            /* 发送蜂鸣器报警码到队列 */
            if (Buzzer_cmd_queue_handle != NULL)
            {
                uint8_t alarm_code = 99;  // 错误系统专用报警码
                xQueueSend(Buzzer_cmd_queue_handle, &alarm_code, 0);
            }
        }
    }
}