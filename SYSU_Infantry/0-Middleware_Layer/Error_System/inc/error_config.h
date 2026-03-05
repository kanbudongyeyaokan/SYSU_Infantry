/**
  * @file    error_config.h
  * @brief   错误处理系统配置文件
  *
  * @note    本项目固定配置：
  *          - FreeRTOS + UART 输出
  *          - 启用时间戳、函数名、行号
  *          - 不启用上下文数据
  */

#ifndef __ERROR_CONFIG_H
#define __ERROR_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* ================= 固定配置 ================= */

/* 环形缓冲区大小 - 必须是 2 的幂，设为 0 禁用 */
#define ERROR_BUFFER_SIZE       0u

/* 上下文数据 - 0 禁用，1 启用 */
#define ERROR_USE_CONTEXT       0u

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
#if ERROR_USE_CONTEXT
    uint32_t context[4];
#endif
    uint16_t task_id;
    uint8_t  cpu_id;
    uint8_t  reserved;
} error_record_t;

/* ================= 系统状态结构 ================= */

typedef struct {
    uint32_t total_count;          // 累计错误总数
    uint32_t overflow_count;       // 缓冲区溢出次数
} error_system_status_t;


#endif /* __ERROR_CONFIG_H */