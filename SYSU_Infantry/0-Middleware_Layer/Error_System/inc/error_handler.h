/**
  * @file    error_handler.h
  * @brief   错误处理系统核心接口
  *
  * @note    本项目固定配置：
  *          - FreeRTOS + UART 输出
  *          - 启用时间戳、函数名、行号
  *          - 环形缓冲区：32 条记录
  */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>

/* ================= 固定配置 ================= */

/* 环形缓冲区大小 - 必须是 2 的幂 */
#define ERROR_BUFFER_SIZE       32u

/* ================= 错误等级 ================= */

typedef enum {
    ERROR_LEVEL_INFO = 0u,
    ERROR_LEVEL_WARNING = 1u,
    ERROR_LEVEL_ERROR = 2u,
    ERROR_LEVEL_CRITICAL = 3u,
} error_level_t;

/* ================= 错误记录结构 ================= */

typedef struct {
    uint32_t error_code;           // 错误码 [保留 24 位][等级 8 位]
    const char* module_name;       // 模块名称字符串
    uint32_t timestamp;            // 时间戳 (ms)
    const char* function;          // 函数名
    uint32_t line;                 // 行号
    const char* message;           // 错误描述
    uint16_t task_id;
    uint8_t  cpu_id;
    uint8_t  reserved;
} error_record_t;

/* ================= 系统状态结构 ================= */

typedef struct {
    uint32_t total_count;          // 累计错误总数
    uint32_t overflow_count;       // 缓冲区溢出次数
} error_system_status_t;

/* ================= 错误码宏 ================= */

#define MAKE_ERROR_CODE(level)  ((uint32_t)((level) & 0xFFu))
#define ERROR_GET_LEVEL(code)   ((code) & 0xFFu)

/* ================= 初始化 ================= */

void error_system_init(void* uart_handle);

/* ================= 错误上报核心接口 ================= */

void error_report_core(error_level_t level,
                       const char* module_name,
                       const char* function,
                       uint32_t line,
                       const char* format, ...);


#define ERROR_INFO(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_INFO, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_WARN(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_WARNING, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_RAISE(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_ERROR, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_CRITICAL(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_CRITICAL, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

/* ================= 快捷宏 ================= */

uint32_t error_get_total_count(void);
const error_record_t* error_get_latest(void);
const error_record_t* error_get_history(uint32_t index);
void error_get_system_status(error_system_status_t* status);
void error_clear_records(void);
bool error_has_critical(void);

/* ================= 平台相关接口 ================= */

uint16_t error_port_get_task_id(void);
uint32_t error_port_get_timestamp(void);
void error_port_output(const error_record_t* record);

#endif /* __ERROR_HANDLER_H */
