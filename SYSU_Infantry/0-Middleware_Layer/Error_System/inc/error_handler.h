/**
  * @file    error_handler.h
  * @brief   错误处理系统核心接口
  *
  * @note    本项目固定配置：
  *          - FreeRTOS + UART 输出
  *          - 启用时间戳、函数名、行号
  */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include "error_config.h"
#include <stdbool.h>

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
                       const char* message);

void error_report_ctx(error_level_t level,
                      const char* module_name,
                      const char* function,
                      uint32_t line,
                      const char* message,
                      uint32_t ctx0, uint32_t ctx1,
                      uint32_t ctx2, uint32_t ctx3);

/* ================= 快捷宏 ================= */

#define ERROR_INFO(module_name, msg) \
    error_report_core(ERROR_LEVEL_INFO, module_name, __func__, __LINE__, msg)

#define ERROR_WARN(module_name, msg) \
    error_report_core(ERROR_LEVEL_WARNING, module_name, __func__, __LINE__, msg)

#define ERROR_RAISE(module_name, msg) \
    error_report_core(ERROR_LEVEL_ERROR, module_name, __func__, __LINE__, msg)

#define ERROR_CRITICAL(module_name, msg) \
    error_report_core(ERROR_LEVEL_CRITICAL, module_name, __func__, __LINE__, msg)

/* 带上下文的宏 */
#define ERROR_INFO_CTX(module_name, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_INFO, module_name, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_WARN_CTX(module_name, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_WARNING, module_name, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_RAISE_CTX(module_name, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_ERROR, module_name, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_CRITICAL_CTX(module_name, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_CRITICAL, module_name, __func__, __LINE__, msg, c0, c1, c2, c3)

/* ================= 错误查询接口 ================= */

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
