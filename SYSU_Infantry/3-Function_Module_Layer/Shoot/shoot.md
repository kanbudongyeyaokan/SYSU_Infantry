# Shoot 发射机构模块说明

发射机构模块位于 `3-Function_Module_Layer/Shoot/`，负责摩擦轮、拨弹盘、单发/连发和堵转处理。应用层入口是 `4-Application_Layer/Execution/Shoot_Task/shoot_task.c`。

## 模块边界

发射模块做：

- 初始化 2 个 M3508 摩擦轮电机和 1 个 M2006 拨弹电机。
- 根据 `Shoot_cmd_send_t` 控制摩擦轮启停。
- 根据 `loader_mode` 控制拨弹盘停止、单发、连发和反转。
- 检测拨弹盘堵转并触发反转退弹。
- 调用 `Djimotor_Calc_Output()` 缓存输出。
- 写回 `Shoot_feedback_queue_handle`。

发射模块不做：

- 不创建 FreeRTOS 任务。
- 不直接发送 CAN 控制帧。
- 不直接读取遥控器拨轮或鼠标按键。

## 硬件配置

当前初始化的电机：

| 名称 | 类型 | CAN | 控制帧 | 反馈 ID | 控制模式 |
| --- | --- | --- | --- | --- | --- |
| `FRICTION_L` | M3508 | CAN2 | `0x200` | `0x201` | 速度环 |
| `FRICTION_R` | M3508 | CAN2 | `0x200` | `0x202` | 速度环 |
| `LOADER` | M2006 | CAN2 | `0x200` | `0x203` | 角度-速度串级环 / 速度环 |

关键常量：

```c
#define ONE_BULLET_DELTA_ANGLE    36.0f
#define REDUCTION_RATIO_LOADER    36.0f
#define SINGLE_SHOOT_INTERVAL_MS  200
#define FRICTION_WHEEL_TARGET_RPM 6700
#define JAM_DETECT_TIME_MS        250
#define JAM_CURRENT_THRESHOLD     8000
#define JAM_CURRENT_SAMPLES       10
#define REVERSE_ANGLE             108.0f
```

含义：

- 10 孔拨弹盘每发转动 `36` 度。
- M2006 减速比按 `36` 计算，因此单发目标角变化为 `36 * 36` 度电机侧角度。
- 单发最小间隔 200 ms。
- 摩擦轮目标转速当前为 6700 rpm。
- 连发堵转通过拨弹电机电流平均值判断，超过阈值并持续一定时间后反转。

## 主要接口

```c
void Shoot_motors_init(void);
void Shoot_task_init(void);
void Shoot_handle_command(Shoot_cmd_send_t *cmd);
```

### `Shoot_motors_init`

初始化 3 个 DJI 电机实例，并设置默认状态：

- 摩擦轮默认 `MOTOR_ENABLED`，但在 `SHOOT_OFF` 下会被切到 `MOTOR_STOP`。
- 拨弹电机默认 `MOTOR_STOP`。
- `shoot_cmd_recv.loader_mode = LOAD_STOP`。
- `shoot_cmd_recv.shoot_mode = SHOOT_OFF`。

### `Shoot_task_init`

当前只调用 `Shoot_motors_init()`。

### `Shoot_handle_command`

主控制入口，应用层任务每 5 ms 调用一次。

## 控制模式

### 摩擦轮

`SHOOT_OFF`：

- 两个摩擦轮电机切到 `MOTOR_STOP`。
- 目标转速置 0。

`SHOOT_ON`：

- 两个摩擦轮电机切到 `MOTOR_ENABLED`。
- 左摩擦轮目标 `+6700 rpm`。
- 右摩擦轮目标 `-6700 rpm`。

### 拨弹盘 `LOAD_STOP`

- 拨弹电机切到 `SPEED_LOOP`。
- 状态设为 `MOTOR_STOP`。
- 目标置 0。

### 拨弹盘 `LOAD_1_BULLET`

- 拨弹电机切到 `ANGLE_AND_SPEED_LOOP`。
- 首次进入单发模式时以当前角度作为起点。
- 每隔 `SINGLE_SHOOT_INTERVAL_MS` 让目标角减少 `ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER`。
- 通过绝对目标角推进实现非阻塞单发。

### 拨弹盘 `LOAD_BURSTFIRE`

- 拨弹电机切到 `ANGLE_AND_SPEED_LOOP`。
- 连发间隔由 `shoot_rate` 决定：

```c
burst_interval_ms = (cmd->shoot_rate > 0) ? (1000 / cmd->shoot_rate) : 125;
```

- 到达间隔后推进一次拨弹目标角。
- 同时采样拨弹电机电流，计算 10 个样本平均值。
- 电流平均值超过 `JAM_CURRENT_THRESHOLD` 并持续 `JAM_DETECT_TIME_MS` 后进入反转。

### 反转退弹

堵转后：

- `is_reversing = true`。
- 目标角设为当前多圈角度加 `REVERSE_ANGLE * REDUCTION_RATIO_LOADER`。
- 接近目标后退出反转，重置堵转检测时间，并延迟继续供弹。

## 应用层任务

`Shoot_control_task()` 当前：

1. 调用 `Shoot_task_init()`。
2. 默认命令为 `SHOOT_OFF + LOAD_STOP`。
3. 非阻塞读取 `Shoot_cmd_queue_handle`。
4. 调用 `Shoot_handle_command()`。
5. 使用 `vTaskDelayUntil()` 保持 5 ms 周期。

## 使用示例

```c
Shoot_cmd_send_t cmd = {0};

cmd.shoot_mode = SHOOT_ON;
cmd.loader_mode = LOAD_1_BULLET;
cmd.shoot_rate = 0;
Shoot_handle_command(&cmd);
```

连发：

```c
Shoot_cmd_send_t cmd = {0};

cmd.shoot_mode = SHOOT_ON;
cmd.loader_mode = LOAD_BURSTFIRE;
cmd.shoot_rate = 8;
Shoot_handle_command(&cmd);
```

停止：

```c
Shoot_cmd_send_t cmd = {0};

cmd.shoot_mode = SHOOT_OFF;
cmd.loader_mode = LOAD_STOP;
cmd.shoot_rate = 0;
Shoot_handle_command(&cmd);
```

## 调试建议

1. 先拆弹或空仓测试摩擦轮方向。
2. 单独测试 `LOAD_STOP` 下拨弹盘是否无力矩。
3. 用低 `shoot_rate` 测试单发目标角推进是否正确。
4. 连发时观察拨弹电机电流和堵转日志。
5. 若误判堵转，先调整 `JAM_CURRENT_THRESHOLD` 和 `JAM_DETECT_TIME_MS`。
6. 调试日志 `ERROR_INFO("SHOOT", ...)` 在高频状态下可能较密，实车前建议限频或关闭。
