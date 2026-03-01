/**
  * @file    error_handler.c
  * @brief   错误处理系统核心实现
  */

#include "error_handler.h"
#include <string.h>
#include "cmsis_gcc.h"
/* ================= 私有宏定义 ================= */

/* 环形缓冲区掩码 (BUFFER_SIZE 必须是 2 的幂) */
#define ERROR_BUFFER_MASK   (ERROR_BUFFER_SIZE - 1u)
#define ERROR_BUFFER_INDEX(i) ((i) & ERROR_BUFFER_MASK)

/* ================= 私有变量 ================= */

/* 环形缓冲区 */
static error_record_t error_buffer[ERROR_BUFFER_SIZE];

/* 缓冲区索引 */
static volatile uint32_t error_head = 0u;     /* 写入位置 */
static volatile uint32_t error_count = 0u;    /* 当前有效数量 */

/* 系统状态 */
static error_system_status_t error_status;

/* Critical 错误标志 */
static volatile bool error_has_critical_flag = false;

/* UART 句柄指针 - 由初始化时传入 */
static void* error_uart_handle = NULL;

/* ================= 私有函数声明 ================= */

static void error_buffer_push(const error_record_t* record);
static void error_update_status(uint8_t module);

/* ================= 初始化 ================= */

void error_system_init(void* uart_handle)
{
    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;
    memset(&error_status, 0, sizeof(error_status));
    error_has_critical_flag = false;
    error_uart_handle = uart_handle;  /* 保存 UART 句柄 */
}

/* ================= 核心实现 ================= */

void error_report_core(error_level_t level,
                       uint8_t module,
                       uint16_t error_id,
                       const char* function,
                       uint32_t line,
                       const char* message)
{
    error_record_t record;

    /* 构建错误记录 */
    memset(&record, 0, sizeof(record));

    record.error_code = MAKE_ERROR_CODE(module, error_id, level);

#if ERROR_USE_TIMESTAMP
    record.timestamp = error_port_get_timestamp();
#endif

#if ERROR_USE_SOURCE_INFO
    record.function = function;
    record.line = line;
#endif

    record.message = message;

#if ERROR_USE_CONTEXT
    record.context[0] = 0u;
    record.context[1] = 0u;
    record.context[2] = 0u;
    record.context[3] = 0u;
#endif

    record.task_id = error_port_get_task_id();
    record.cpu_id = 0u;
    record.reserved = 0u;

    /* 写入缓冲区 */
    error_buffer_push(&record);

    /* 更新状态 */
    error_update_status(module);

    /* 输出错误信息 */
    error_port_output(&record);

    /* Critical 错误处理 */
    if (level == ERROR_LEVEL_CRITICAL)
    {
        error_has_critical_flag = true;
        /* 可在此添加复位、停机处理 */
    }
}

void error_report_ctx(error_level_t level,
                      uint8_t module,
                      uint16_t error_id,
                      const char* function,
                      uint32_t line,
                      const char* message,
                      uint32_t ctx0, uint32_t ctx1,
                      uint32_t ctx2, uint32_t ctx3)
{
    error_record_t record;

    memset(&record, 0, sizeof(record));

    record.error_code = MAKE_ERROR_CODE(module, error_id, level);

#if ERROR_USE_TIMESTAMP
    record.timestamp = error_port_get_timestamp();
#endif

#if ERROR_USE_SOURCE_INFO
    record.function = function;
    record.line = line;
#endif

    record.message = message;

#if ERROR_USE_CONTEXT
    record.context[0] = ctx0;
    record.context[1] = ctx1;
    record.context[2] = ctx2;
    record.context[3] = ctx3;
#endif

    record.task_id = error_port_get_task_id();
    record.cpu_id = 0u;
    record.reserved = 0u;

    error_buffer_push(&record);
    error_update_status(module);
    error_port_output(&record);

    if (level == ERROR_LEVEL_CRITICAL)
    {
        error_has_critical_flag = true;
    }
}

/* ================= 缓冲区操作 ================= */

static void error_buffer_push(const error_record_t* record)
{
#if ERROR_USE_THREAD_SAFE
    /* 临界区保护 */
    __disable_irq();
#endif

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

#if ERROR_USE_THREAD_SAFE
    __enable_irq();
#endif
}

static void error_update_status(uint8_t module)
{
#if ERROR_USE_THREAD_SAFE
    __disable_irq();
#endif

    if (module < 16u)
    {
        error_status.module_counts[module]++;
    }

#if ERROR_USE_THREAD_SAFE
    __enable_irq();
#endif
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

uint32_t error_get_module_count(uint8_t module)
{
    if (module >= 16u)
    {
        return 0u;
    }
    return error_status.module_counts[module];
}

void error_get_system_status(error_system_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

#if ERROR_USE_THREAD_SAFE
    __disable_irq();
#endif

    *status = error_status;

#if ERROR_USE_THREAD_SAFE
    __enable_irq();
#endif
}

void error_clear_records(void)
{
#if ERROR_USE_THREAD_SAFE
    __disable_irq();
#endif

    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;

#if ERROR_USE_THREAD_SAFE
    __enable_irq();
#endif
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
