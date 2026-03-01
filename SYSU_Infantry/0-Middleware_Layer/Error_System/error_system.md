# 错误处理系统使用文档

## 1. 快速开始

### 1.1 添加文件到项目

在 `CMakeLists.txt` 中添加：

```cmake
# 错误处理系统
0-Middleware_Layer/Error_System/inc/error_config.h
0-Middleware_Layer/Error_System/inc/error_code.h
0-Middleware_Layer/Error_System/inc/error_handler.h
0-Middleware_Layer/Error_System/src/error_handler.c
0-Middleware_Layer/Error_System/src/error_port.c
```

添加 include 路径：
```cmake
${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Error_System/inc
```

### 1.2 初始化

```c
#include "error_handler.h"
#include "bsp_usart.h"

// 假设 uart_handle 是 Uart_instance_t* 类型的 UART 句柄
error_system_init(uart_handle);  // 传入 UART 句柄，可为 NULL
```

**示例（robot_task.c）**：
```c
Uart_instance_t* test_uart = Uart_register(&huart6, NULL);
error_system_init(test_uart);  // 初始化错误系统
```

---

## 2. 基本使用

### 2.1 上报错误

```c
#include "error_handler.h"

/* Info - 信息提示 */
ERROR_INFO(ERROR_MODULE_SYSTEM, ERROR_SYSTEM_STATE_ERROR,
           "System state check passed");

/* Warning - 警告 */
ERROR_WARN(ERROR_MODULE_CAN, ERROR_CAN_TIMEOUT,
           "CAN receive timeout");

/* Error - 错误 */
ERROR_RAISE(ERROR_MODULE_MOTOR, ERROR_MOTOR_COMM_FAIL,
            "Motor communication failed");

/* Critical - 严重错误 */
ERROR_CRITICAL(ERROR_MODULE_IMU, ERROR_IMU_INIT_FAIL,
               "IMU initialization failed");
```

### 2.2 带上下文数据的错误

```c
/* 记录错误时的寄存器值或状态 */
ERROR_RAISE_CTX(ERROR_MODULE_CAN, ERROR_CAN_SEND_FAIL,
                "CAN send failed",
                hcan->Instance->ESR,  // 上下文 0: 错误状态寄存器
                hcan->Instance->TSR,  // 上下文 1: 发送状态寄存器
                0, 0);                 // 上下文 2,3
```

### 2.3 在现有代码中集成

```c
/* HAL 回调中 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (HAL_CAN_GetRxFillLevel(hcan) > RX_FIFO_DEPTH)
    {
        ERROR_WARN(ERROR_MODULE_CAN, ERROR_CAN_FIFO_OVERFLOW,
                   "CAN FIFO overflow detected");
        return;
    }
    // ...
}

/* 设备初始化 */
HAL_StatusTypeDef BMI088_Init(void)
{
    if (chip_id != BMI088_CORRECT_ID)
    {
        ERROR_CRITICAL(ERROR_MODULE_IMU, ERROR_IMU_INIT_FAIL,
                       "BMI088 chip ID mismatch");
        return HAL_ERROR;
    }
    // ...
}

/* 电机驱动 */
void motor_set_speed(motor_t* motor, float speed)
{
    if (motor == NULL)
    {
        ERROR_RAISE(ERROR_MODULE_MOTOR, ERROR_MOTOR_NULL_POINTER,
                    "motor pointer is NULL");
        return;
    }

    if (speed > MAX_SPEED || speed < -MAX_SPEED)
    {
        ERROR_WARN(ERROR_MODULE_MOTOR, ERROR_MOTOR_INVALID_SPEED,
                   "speed out of range");
        speed = fmaxf(-MAX_SPEED, fminf(speed, MAX_SPEED));
    }
    // ...
}
```

---

## 3. 蜂鸣器集成

错误系统已与现有蜂鸣器告警系统集成。

### 3.1 自动报警机制

当发生 **Critical 等级** 错误时：
- 自动发送报警码 `99` 到 `Buzzer_cmd_queue_handle`
- 蜂鸣器播放 **两声 2kHz 短促提示音**
- 带 2 秒冷却时间，防止同一错误重复触发

### 3.2 报警效果

```c
// 上报 Critical 错误
ERROR_CRITICAL(ERROR_MODULE_IMU, ERROR_IMU_INIT_FAIL, "IMU 初始化失败");
```

**触发结果**:
1. UART6 输出：`[CRIT][IMU][0x0001] bmi088.c:45 IMU 初始化失败`
2. 蜂鸣器：两声"滴 - 滴"提示音
3. LED：PG14 闪烁 5 次

### 3.3 修改报警方式

在 `buzzer_alarm.c` 的 `Alarm_handle_command()` 中修改 `cmd == 99` 的处理逻辑：

```c
if (*cmd == 99)
{
    // 自定义报警方式
    Play_Mario_Die(&buzzer);  // 播放音乐
    // 或者...
}
```

---

## 4. 配置选项

在 `error_config.h` 中修改配置：

```c
/* 环形缓冲区大小 - 必须是 2 的幂 */
#define ERROR_BUFFER_SIZE       32u     // 32/64/128...

/* 是否启用时间戳 */
#define ERROR_USE_TIMESTAMP     1u

/* 是否启用函数名/行号 */
#define ERROR_USE_SOURCE_INFO   1u

/* 是否启用上下文数据 */
#define ERROR_USE_CONTEXT       0u      // 1 启用，增加 16 字节/记录

/* 是否启用线程安全 */
#define ERROR_USE_THREAD_SAFE   1u

/* 输出方式 */
#define ERROR_OUTPUT_RTT        1u      // SEGGER RTT
#define ERROR_OUTPUT_UART     0u      // UART
```

---

## 4. 错误码定义

### 4.1 错误码格式

```
[31:24] 模块 ID (8 位)
[23:8]  错误 ID (16 位)
[7:0]   错误等级 (8 位)
```

### 4.2 预定义模块

| 模块 ID | 名称 | 说明 |
|---------|------|------|
| 0x00 | SYS | 系统 |
| 0x01 | CAN | CAN 总线 |
| 0x02 | MOTOR | 电机驱动 |
| 0x03 | IMU | 姿态传感器 |
| 0x04 | GIMBAL | 云台 |
| 0x05 | CHASSIS | 底盘 |
| 0x06 | SHOOT | 射击 |
| 0x07 | REFEREE | 裁判系统 |
| 0x08 | REMOTE | 遥控器 |
| 0x09 | POWER | 电源 |
| 0x0A | AUDIO | 蜂鸣器 |
| 0x0B | USB | USB 设备 |
| 0x0C | FREE | 自定义 |

### 4.3 添加自定义错误

在 `error_code.h` 中添加：

```c
/* 自定义模块错误 */
#define ERROR_MY_MODULE_INIT_FAIL   0x0001u
#define ERROR_MY_MODULE_TIMEOUT     0x0002u
```

使用：
```c
ERROR_RAISE(0x0D, ERROR_MY_MODULE_INIT_FAIL, "自定义错误");
```

---

## 5. 高级用法

### 5.1 替换 Error_Handler

```c
/* 在 main.c 中 */
void Error_Handler(void)
{
    ERROR_CRITICAL(ERROR_MODULE_SYSTEM, ERROR_SYSTEM_TIMEOUT,
                   "HAL initialization failed");

    /* 原有代码 */
    __disable_irq();
    while (1)
    {
    }
}
```

### 5.2 错误恢复钩子

在 `error_handler.c` 的 `error_report_core()` 中添加：

```c
if (level == ERROR_LEVEL_CRITICAL)
{
    error_has_critical_flag = true;

    /* 添加恢复逻辑 */
    // NVIC_SystemReset();  // 系统复位
    // enter_safe_mode();   // 进入安全模式
}
```

### 5.3 与裁判系统对接

```c
/* 在 error_port_output() 中添加 */
if (level == ERROR_LEVEL_CRITICAL)
{
    /* 发送到裁判系统 */
    referee_send_error(record->error_code);
}
```

---

## 6. 内存占用

| 配置 | RAM 占用 |
|------|----------|
| 基础 (32 条记录，无上下文) | 32 × 48 = 1.5KB |
| 完整 (32 条记录，有上下文) | 32 × 64 = 2KB |
| 64 条记录 (无上下文) | 64 × 48 = 3KB |

---

## 7. 注意事项

1. **环形缓冲区大小必须是 2 的幂**（32/64/128...）
2. **Critical 错误会触发蜂鸣器报警**（两声 2kHz 提示音，2 秒冷却）
3. **中断中可以使用**，已做临界区保护
4. **上下文数据会显著增加 RAM 占用**
