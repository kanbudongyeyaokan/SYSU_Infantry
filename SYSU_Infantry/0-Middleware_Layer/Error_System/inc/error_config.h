/**
  * @file    error_config.h
  * @brief   错误处理系统配置文件
  */

#ifndef __ERROR_CONFIG_H
#define __ERROR_CONFIG_H


/* ================= 配置选项 ================= */

/* 环形缓冲区大小 - 必须是 2 的幂次方便取模 */
#define ERROR_BUFFER_SIZE       32u     // 最多保存 32 条错误记录

/* 是否启用时间戳 */
#define ERROR_USE_TIMESTAMP     1u

/* 是否启用函数名/行号记录 */
#define ERROR_USE_SOURCE_INFO   1u

/* 是否启用上下文数据 */
#define ERROR_USE_CONTEXT       0u      // 设为 1 可记录寄存器/状态等额外数据

/* 是否启用线程安全（FreeRTOS 环境） */
#define ERROR_USE_THREAD_SAFE   1u

/* 错误输出方式 */
#define ERROR_OUTPUT_RTT        0u      // SEGGER RTT 输出
#define ERROR_OUTPUT_UART     1u      // UART 输出

/* ================= 模块定义 ================= */

#define ERROR_MODULE_SYSTEM     0x00u
#define ERROR_MODULE_CAN        0x01u
#define ERROR_MODULE_MOTOR      0x02u
#define ERROR_MODULE_IMU        0x03u
#define ERROR_MODULE_GIMBAL     0x04u
#define ERROR_MODULE_CHASSIS    0x05u
#define ERROR_MODULE_SHOOT      0x06u
#define ERROR_MODULE_REFEREE    0x07u
#define ERROR_MODULE_REMOTE     0x08u
#define ERROR_MODULE_POWER      0x09u
#define ERROR_MODULE_AUDIO      0x0Au
#define ERROR_MODULE_USB        0x0Bu
#define ERROR_MODULE_FREE       0x0Cu   // 空闲模块，自定义使用

/* ================= 错误等级 ================= */

typedef enum {
    ERROR_LEVEL_INFO = 0u,       // 提示信息
    ERROR_LEVEL_WARNING = 1u,    // 警告，可恢复
    ERROR_LEVEL_ERROR = 2u,      // 错误，需关注
    ERROR_LEVEL_CRITICAL = 3u,   // 严重，系统停止
} error_level_t;

/* ================= 类型定义 ================= */

#include <stdint.h>
#include <stddef.h>

/* 错误记录结构 */
typedef struct {
    uint32_t error_code;           // 错误码 [模块 8 位][ID 16 位][等级 8 位]
    uint32_t timestamp;            // 发生时间 (ms)
#if ERROR_USE_SOURCE_INFO
    const char* function;          // 函数名
    uint32_t line;                 // 行号
#endif
    const char* message;           // 错误描述
#if ERROR_USE_CONTEXT
    uint32_t context[4];           // 上下文数据
#endif
    uint16_t task_id;              // 任务 ID/堆栈名
    uint8_t  cpu_id;               // CPU 核心
    uint8_t  reserved;             // 保留
} error_record_t;

/* 错误系统状态 */
typedef struct {
    uint32_t total_count;          // 累计错误总数
    uint32_t overflow_count;       // 缓冲区溢出次数
    uint32_t module_counts[16];    // 各模块错误计数
} error_system_status_t;


#endif /* __ERROR_CONFIG_H */
