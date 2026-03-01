/**
  * @file    error_handler.h
  * @brief   错误处理系统核心接口
  */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H



#include "error_config.h"
#include "error_code.h"
#include <stdbool.h>

/* ================= 初始化 ================= */

/**
  * @brief  初始化错误处理系统
  * @param  uart_handle: UART 句柄指针（用于错误输出，可为 NULL）
  * @note   在系统启动时调用一次
  */
void error_system_init(void* uart_handle);

/* ================= 错误上报核心接口 ================= */

/**
  * @brief  上报错误（内部使用）
  * @param  level: 错误等级
  * @param  module: 模块 ID
  * @param  error_id: 错误 ID
  * @param  function: 函数名 (__func__)
  * @param  line: 行号 (__LINE__)
  * @param  message: 错误描述
  */
void error_report_core(error_level_t level,
                       uint8_t module,
                       uint16_t error_id,
                       const char* function,
                       uint32_t line,
                       const char* message);

/**
  * @brief  上报错误带上下文数据
  * @param  level: 错误等级
  * @param  module: 模块 ID
  * @param  error_id: 错误 ID
  * @param  function: 函数名
  * @param  line: 行号
  * @param  message: 错误描述
  * @param  ctx0-ctx3: 上下文数据（如寄存器值、状态码等）
  */
void error_report_ctx(error_level_t level,
                      uint8_t module,
                      uint16_t error_id,
                      const char* function,
                      uint32_t line,
                      const char* message,
                      uint32_t ctx0, uint32_t ctx1,
                      uint32_t ctx2, uint32_t ctx3);

/* ================= 快捷宏 ================= */

#if ERROR_USE_SOURCE_INFO

#define ERROR_INFO(module, id, msg) \
    error_report_core(ERROR_LEVEL_INFO, module, id, __func__, __LINE__, msg)

#define ERROR_WARN(module, id, msg) \
    error_report_core(ERROR_LEVEL_WARNING, module, id, __func__, __LINE__, msg)

#define ERROR_RAISE(module, id, msg) \
    error_report_core(ERROR_LEVEL_ERROR, module, id, __func__, __LINE__, msg)

#define ERROR_CRITICAL(module, id, msg) \
    error_report_core(ERROR_LEVEL_CRITICAL, module, id, __func__, __LINE__, msg)

/* 带上下文的宏 */
#define ERROR_INFO_CTX(module, id, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_INFO, module, id, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_WARN_CTX(module, id, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_WARNING, module, id, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_RAISE_CTX(module, id, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_ERROR, module, id, __func__, __LINE__, msg, c0, c1, c2, c3)

#define ERROR_CRITICAL_CTX(module, id, msg, c0, c1, c2, c3) \
    error_report_ctx(ERROR_LEVEL_CRITICAL, module, id, __func__, __LINE__, msg, c0, c1, c2, c3)

#else /* !ERROR_USE_SOURCE_INFO */

#define ERROR_INFO(module, id, msg) \
    error_report_core(ERROR_LEVEL_INFO, module, id, NULL, 0, msg)

#define ERROR_WARN(module, id, msg) \
    error_report_core(ERROR_LEVEL_WARNING, module, id, NULL, 0, msg)

#define ERROR_RAISE(module, id, msg) \
    error_report_core(ERROR_LEVEL_ERROR, module, id, NULL, 0, msg)

#define ERROR_CRITICAL(module, id, msg) \
    error_report_core(ERROR_LEVEL_CRITICAL, module, id, NULL, 0, msg)

#endif /* ERROR_USE_SOURCE_INFO */

/* ================= 错误查询接口 ================= */

/**
  * @brief  获取错误总数
  * @retval 累计发生的错误数量
  */
uint32_t error_get_total_count(void);

/**
  * @brief  获取最新错误记录
  * @retval error_record_t* 指向最新错误记录，无错误时返回 NULL
  */
const error_record_t* error_get_latest(void);

/**
  * @brief  获取历史错误记录
  * @param  index: 索引 (0=最新，1=次新...)
  * @retval error_record_t* 指向错误记录，越界时返回 NULL
  */
const error_record_t* error_get_history(uint32_t index);

/**
  * @brief  获取指定模块的错误计数
  * @param  module: 模块 ID
  * @retval 该模块的错误数量
  */
uint32_t error_get_module_count(uint8_t module);

/**
  * @brief  获取系统状态
  * @param  status: 输出状态结构指针
  */
void error_get_system_status(error_system_status_t* status);

/**
  * @brief  清除所有错误记录
  * @note   不清除计数器
  */
void error_clear_records(void);

/**
  * @brief  检查是否有未处理的 Critical 错误
  * @retval true 存在 Critical 错误
  */
bool error_has_critical(void);

/* ================= 平台相关接口 ================= */

/**
  * @brief  获取当前任务 ID
  * @retval 任务标识符
  */
uint16_t error_port_get_task_id(void);

/**
  * @brief  获取系统时间戳 (ms)
  * @retval 毫秒级时间戳
  */
uint32_t error_port_get_timestamp(void);

/**
  * @brief  输出错误信息
  * @param  record: 错误记录
  */
void error_port_output(const error_record_t* record);



#endif /* __ERROR_HANDLER_H */
