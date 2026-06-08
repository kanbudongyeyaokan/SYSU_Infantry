# Middleware Layer 中间件层说明

中间件层位于 `0-Middleware_Layer/`，提供不绑定具体硬件和机器人机构的通用能力。它向 BSP、设备驱动、功能模块和应用任务提供算法、通信模型、调试工具和错误处理基础。

这一层的核心原则是：不关心“是哪一个电机、哪一个任务、哪一辆车”，只提供可复用的基础模块。

## 目录总览

| 目录 | 主要文件 | 职责 |
| --- | --- | --- |
| `Pid_Controller` | `algorithm_pid.c/.h` | PID 控制器，支持自动 dt、输出限幅、梯形积分、微分先行、前馈、滤波、积分分离 |
| `Kalman_Fliter` | `algorithm_kf.*`、`algorithm_ekf.*`、`MahonyAHRS.*` | 状态估计、扩展卡尔曼、姿态融合算法 |
| `Message_Center` | `message_center.c/.h` | 发布-订阅消息中心，支持多发布者/多订阅者和互斥锁保护 |
| `Crc` | `crc_referee.c/.h` | CRC8/CRC16 计算与裁判系统协议校验封装 |
| `Error_System` | `error_handler.*`、`error_port.c` | 错误记录、日志输出、Fatal 急停标志 |
| `Filter` | `lowpass_filter.c/.h` | 一阶低通滤波器 |
| `Ramp_Controller` | `ramp_controller.c/.h` | 通用斜坡限速器 |
| `Math_lib` | `math_lib.*` | 常用数学宏和辅助函数 |
| `UI` | `ui_*.c/.h` | 图形/UI 打包能力 |
| `letter-shell-master` | `src/shell.*` | Letter Shell 命令行组件 |
| `RTT-main` | `RTT/SEGGER_RTT.*` | SEGGER RTT 调试通道 |
| `SystemView-main` | `SYSVIEW/SEGGER_SYSVIEW.*` | SEGGER SystemView 运行时分析支持 |

## PID 控制器

接口：

```c
void Pid_init(Pid_instance_t *pid, Pid_init_t *config);
float Pid_calculate(Pid_instance_t *pid, float measure, float target);
void Pid_reset(Pid_instance_t *pid);
```

当前 `algorithm_pid.c` 会通过 `DWT_GetDeltaT()` 自动计算控制周期，调用前必须保证 `DWT_Init()` 已在 `main.c` 中完成。工程里 `main.c` 当前在外设初始化后调用 `DWT_Init(168)`。

支持的优化位包括：

- `PID_OUTPUT_LIMIT`：输出和积分限幅
- `PID_DIFFERENTIAL_GO_FIRST`：微分先行，降低目标阶跃带来的冲击
- `PID_TRAPEZOID_INTERGRAL`：梯形积分
- `PID_OUTPUT_FILTER`：输出低通滤波
- `PID_FEEDFOWARD`：目标值或外部源前馈
- `PID_INTEGRAL_ANTI_WINDUP`：积分抗饱和
- `PID_INTEGRAL_SEPARATION`：积分分离

典型使用方式：

```c
Pid_init_t cfg = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_iout = 3000.0f,
    .max_out = 15000.0f,
    .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_DIFFERENTIAL_GO_FIRST,
};

Pid_instance_t pid;
Pid_init(&pid, &cfg);
float out = Pid_calculate(&pid, measure, target);
```

## 消息中心

接口：

```c
Publisher_t *Pub_register(char *name, uint8_t data_len);
Subscriber_t *Sub_register(char *name, uint8_t data_len);
uint8_t Pub_push_message(Publisher_t *pub, void *data_ptr);
uint8_t Sub_get_message(Subscriber_t *sub, void *data_ptr);
```

实现特点：

- 使用话题名组织发布者和订阅者。
- 相同话题的多个发布者会共享同一个 `Publisher_t`。
- 每个订阅者内部队列长度为 `QUEUE_SIZE`，当前为 1，因此更像“最新值邮箱”。
- 注册和推送过程使用 FreeRTOS 互斥锁保护。

适合场景：

- 测试任务和低频状态广播。
- 多模块之间需要一对多分发。
- 希望避免直接依赖具体任务句柄。

注意：当前机器人主控制链路已经大量使用 FreeRTOS Queue，如 `Chassis_cmd_queue_handle`、`Gimbal_cmd_queue_handle`。如果要把 Message Center 接入主链路，需要显式写桥接逻辑，不要假设同名话题会自动被主任务读取。

## 错误系统

接口宏：

```c
ERROR_INFO(module, fmt, ...);
ERROR_WARN(module, fmt, ...);
ERROR_RAISE(module, fmt, ...);
ERROR_CRITICAL(module, fmt, ...);
ERROR_FATAL(module, fmt, ...);
```

关键能力：

- 环形缓冲区记录错误历史。
- 记录模块名、时间戳、函数名和行号。
- `ERROR_FATAL` 会置位 Fatal 状态，决策层 `Check_fatal_estop()` 会据此强制底盘、云台、发射进入安全态。

使用建议：

- 高频中断或 1 kHz 控制循环内必须限频输出。
- 可恢复问题用 `ERROR_WARN` 或 `ERROR_RAISE`。
- 设备离线、驱动初始化失败等影响安全的问题用 `ERROR_CRITICAL`。
- 只有不可恢复、必须全车急停的问题才使用 `ERROR_FATAL`。

## CRC

`Crc/crc_referee.c` 提供基础 CRC8/CRC16 和裁判系统专用封装：

```c
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
```

裁判系统和视觉通信协议都应优先复用这里的 CRC 实现，避免各模块各写一份。

## 滤波与斜坡

`Filter/lowpass_filter.*` 提供一阶低通滤波器：

```c
void LPF_Init(Lpf_t *lpf, float dt, float cutoff_freq, float initial_val);
float LPF_Calc(Lpf_t *lpf, float input);
void LPF_Reset(Lpf_t *lpf, float reset_val);
```

`Ramp_Controller/ramp_controller.*` 提供通用斜坡控制：

```c
void Ramp_Init(Ramp_t *ramp, float initial_val, float step);
float Ramp_Calc(Ramp_t *ramp, float target_val);
```

底盘模块当前还实现了专用 `chassis_scurve`，用于底盘 `vx/vy` 平滑；通用 Ramp/LPF 可用于遥控器输入、传感器数据和目标值平滑。

## 调试组件

- `letter-shell-master`：Shell 核心，当前移植入口位于 `3-Function_Module_Layer/Shell/shell_port.c`，任务入口位于 `4-Application_Layer/Decision_Making/Shell_task.c`。
- `RTT-main`：SEGGER RTT，可用于高频调试输出。
- `SystemView-main`：用于观察 FreeRTOS 任务切换和运行时间。

## 新增中间件建议

新增中间件时请满足：

1. 不依赖具体机器人任务和物理参数。
2. 头文件只暴露必要接口。
3. 对实时性和内存占用有注释说明。
4. 如果需要 FreeRTOS 对象，明确初始化时机。
5. 在对应目录添加简短 `.md` 用法说明。
