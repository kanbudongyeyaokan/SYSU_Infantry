/**
  * @file    error_handler.c
  * @brief   错误处理系统核心实现
  *
  * @note    本项目固定配置：
  *          - 环形缓冲区：32 条记录
  *          - 启用时间戳、函数名、行号
  */

#include "error_handler.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_gcc.h"

/* ================= 私有宏定义 ================= */

#define ERROR_BUFFER_MASK   (ERROR_BUFFER_SIZE - 1u)
#define ERROR_BUFFER_INDEX(i) ((i) & ERROR_BUFFER_MASK)

/* ================= 私有变量 ================= */

/* 环形缓冲区 */
static error_record_t error_buffer[ERROR_BUFFER_SIZE];
static volatile uint32_t error_head = 0u;
static volatile uint32_t error_count = 0u;

/* 系统状态 */
static error_system_status_t error_status;

/* Critical 错误标志 */
static volatile bool error_has_critical_flag = false;

/* UART 句柄指针 */
static void* error_uart_handle = NULL;

/* ================= 私有函数声明 ================= */

static void error_buffer_push(const error_record_t* record);

/* ================= 初始化 ================= */

void error_system_init(void* uart_handle)
{
    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;
    memset(&error_status, 0, sizeof(error_status));
    error_has_critical_flag = false;
    error_uart_handle = uart_handle;
}

/* ================= 核心实现 ================= */

void error_report_core(error_level_t level,
                       const char* module_name,
                       const char* function,
                       uint32_t line,
                       const char* format, ...)
{
    static char formatted_message[128];
    va_list args;

    error_record_t record;

    /* 格式化消息 */
    va_start(args, format);
    vsnprintf(formatted_message, sizeof(formatted_message), format, args);
    va_end(args);

    /* 构建错误记录 */
    memset(&record, 0, sizeof(record));

    record.error_code = MAKE_ERROR_CODE(level);
    record.module_name = module_name;
    record.timestamp = error_port_get_timestamp();
    record.function = function;
    record.line = line;
    record.message = formatted_message;

    record.task_id = error_port_get_task_id();
    record.cpu_id = 0u;
    record.reserved = 0u;

    /* 写入缓冲区 */
    error_buffer_push(&record);

    /* 更新状态 */
    error_status.total_count++;

    /* 输出错误信息 */
    error_port_output(&record);
}

void error_report_ctx(error_level_t level,
                      const char* module_name,
                      const char* function,
                      uint32_t line,
                      const char* message,
                      uint32_t ctx0, uint32_t ctx1,
                      uint32_t ctx2, uint32_t ctx3)
{
    error_record_t record;

    memset(&record, 0, sizeof(record));

    record.error_code = MAKE_ERROR_CODE(level);
    record.module_name = module_name;
    record.timestamp = error_port_get_timestamp();
    record.function = function;
    record.line = line;
    record.message = message;

    record.task_id = error_port_get_task_id();
    record.cpu_id = 0u;
    record.reserved = 0u;

    error_buffer_push(&record);
    error_status.total_count++;
    error_port_output(&record);
}

/* ================= 缓冲区操作 ================= */

static void error_buffer_push(const error_record_t* record)
{
    /* 临界区保护 */
    __disable_irq();

    /* 写入缓冲区 */
    error_buffer[ERROR_BUFFER_INDEX(error_head)] = *record;
    error_head++;

    /* 更新有效数量 */
    if (error_count < ERROR_BUFFER_SIZE)
    {
        error_count++;
    }
    else
    {
        /* 缓冲区已满，记录溢出 */
        error_status.overflow_count++;
    }

    /* 累计总数 */
    error_status.total_count++;

    __enable_irq();
}

/* ================= 查询接口 ================= */

uint32_t error_get_total_count(void)
{
    return error_status.total_count;
}

const error_record_t* error_get_latest(void)
{
    if (error_count == 0u)
    {
        return NULL;
    }

    /* 返回最新记录（head-1） */
    uint32_t index = ERROR_BUFFER_INDEX(error_head - 1u);
    return &error_buffer[index];
}

const error_record_t* error_get_history(uint32_t index)
{
    if (error_count == 0u)
    {
        return NULL;
    }

    if (index >= error_count)
    {
        return NULL;
    }

    /* index=0 是最新记录，所以需要倒推 */
    uint32_t actual_index = ERROR_BUFFER_INDEX(error_head - 1u - index);
    return &error_buffer[actual_index];
}

void error_get_system_status(error_system_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    __disable_irq();
    *status = error_status;
    __enable_irq();
}

void error_clear_records(void)
{
    __disable_irq();

    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;

    __enable_irq();
}

bool error_has_critical(void)
{
    return error_has_critical_flag;
}

/* ================= 内部接口（供 error_port 使用） ================= */

/**
  * @brief  获取 UART 句柄
  * @retval UART 句柄指针
  */
void* error_get_uart_handle(void)
{
    return error_uart_handle;
}

/**
  * @brief  设置 Critical 错误标志
  */
void error_set_critical_flag(void)
{
    error_has_critical_flag = true;
}
