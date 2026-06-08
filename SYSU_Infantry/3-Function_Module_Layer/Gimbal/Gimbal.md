# Gimbal 云台控制模块说明

云台模块位于 `3-Function_Module_Layer/Gimbal/`，负责步兵机器人 Yaw/Pitch 双轴云台电机初始化、IMU 反馈闭环、视觉目标接入和反馈回写。应用层入口是 `4-Application_Layer/Execution/Gimbal_Task/Gimbal_task.c`。

本文档按当前源码描述。历史设想中的完整无扰切换状态机尚未在当前 `gimbal.c` 中落地，调试时请以本说明和源码为准。

## 模块边界

云台模块做：

- 获取 INS 姿态数据指针。
- 初始化 Yaw/Pitch 两个 GM6020 电机。
- 初始化视觉通信。
- 根据 `Gimbal_cmd_send_t` 处理零力矩、IMU 模式和视觉模式。
- 调用 `Djimotor_Calc_Output()` 缓存电机输出。
- 把云台单圈角度和当前电机目标写回反馈队列。

云台模块不做：

- 不创建任务。
- 不直接发送电机 CAN 控制帧。
- 不决定遥控/键鼠/视觉模式切换，模式由决策层下发。

## 文件结构

| 文件 | 作用 |
| --- | --- |
| `gimbal.c/.h` | 云台电机初始化、状态查询、命令处理 |
| `../Vision_Comm/vision_comm.*` | 电控板与视觉主机通信 |
| `../Ins/ins.*` | 云台使用的姿态数据来源 |
| `../Decision_making/decision_making.*` | 云台命令和反馈结构体定义 |

## 硬件与反馈源

当前初始化的电机：

| 轴 | 电机 | CAN | 控制帧 | 反馈 ID | 角度反馈 | 速度反馈 |
| --- | --- | --- | --- | --- | --- | --- |
| Yaw | GM6020 | CAN1 | `0x1FF` | `0x205` | `gimbal_imu_data->total_yaw` | `gimbal_imu_data->gyro_body.z` |
| Pitch | GM6020 | CAN2 | `0x1FF` | `0x206` | `gimbal_imu_data->euler.roll` | `gimbal_imu_data->gyro_body.x` |

注意：源码中 Pitch 角度反馈当前使用 `euler.roll`，注释中也提示“代码中使用 euler.roll 代指 pitch”。调参和改 IMU 坐标系时要重点核对这一点。

## 主要接口

```c
void Gimbal_task_init(void);
void Gimbal_handle_command(Gimbal_cmd_send_t *cmd);
Gimbal_state_e Gimbal_get_state(void);
```

### `Gimbal_task_init`

当前流程：

1. 调用 `Ins_get_data()` 获取姿态数据指针。
2. 指针为空则上报 `ERROR_CRITICAL` 并返回。
3. 调用内部 `Gimbal_motor_init()` 初始化两个 GM6020。
4. 调用 `Vision_Comm_Init()` 初始化视觉通信。

### `Gimbal_get_state`

当前状态判断较简单：

- IMU 指针或电机指针为空：`GIMBAL_STATE_INIT`
- INS 状态为 `INS_STATE_READY`：`GIMBAL_STATE_READY`
- 其他情况：`GIMBAL_STATE_INIT`

头文件中保留了 `GIMBAL_STATE_ZEROING`，但当前实现没有单独归中流程。

## 控制模式

模式定义位于 `Robot_Definitions/robot_definitions.h`。

### `GIMBAL_ZERO_FORCE`

安全失能：

- Yaw/Pitch 电机切到 `MOTOR_STOP`。
- 目标置 0。
- 计算输出后由电机任务发送零电流。

### `GIMBAL_GYRO_MODE`

IMU 反馈模式：

- 电机使能。
- `cmd->yaw` 直接作为 Yaw 目标。
- `cmd->pitch` 直接作为 Pitch 目标。
- 角度环 + 速度环串级 PID 完成闭环。

目标量通常由决策层根据遥控器摇杆或键鼠鼠标增量累加得到。

### `GIMBAL_VISION_MODE`

视觉模式：

- 电机使能。
- 如果 `Is_Vision_Online()` 为真，从 `Get_Vision_Ctrl_Data()` 读取视觉目标。
- `target_yaw` 和 `target_pitch` 直接设置为 Yaw/Pitch 电机目标。
- 当前视觉速度前馈字段在 `vision_comm.c` 中被置 0，尚未接入云台 PID。

如果视觉离线，当前源码没有显式回退到遥控目标的执行分支，相关逻辑被注释保留。实车调试时建议在决策层或云台层明确离线降级策略。

## 视觉通信

`Gimbal_handle_command()` 每次执行都会：

1. 调用 `Vision_Comm_Parse_Task()` 非阻塞解析视觉 FIFO。
2. 使用 `DWT_GetTimeline_s()` 生成微秒时间戳。
3. 将当前 Pitch/Yaw 角度和角速度从度转换为弧度。
4. 调用 `Vision_Send_Pose()` 给视觉主机发送姿态包。

当前视觉通信默认启用：

```c
#define USE_VISION_USB
```

即通过 USB CDC 收发数据。如果改用 UART，需要调整 `vision_comm.h` 和底层注册逻辑。

## 反馈给决策层

`Gimbal_handle_command()` 末尾写入：

```c
gimbal_feedback.yaw_motor_single_round_angle = yaw_motor->motor_measure.current_angle;
gimbal_feedback.active_yaw_target = yaw_motor->motor_pid.pid_target;
gimbal_feedback.active_pitch_target = pitch_motor->motor_pid.pid_target;
xQueueOverwrite(Gimbal_feedback_queue_handle, &gimbal_feedback);
```

当前反馈中 `active_yaw_target/active_pitch_target` 实际等于当前电机 PID 目标。决策层使用它们进行模式切换时的目标同步，但云台层本身尚未实现完整的 active_ref 过渡器。

## 应用层任务

`Gimbal_control_task()` 当前周期为 `GIMBAL_TASK_PERIOD = 1` tick：

1. 调用 `Gimbal_task_init()`。
2. 默认命令初始化为 `GIMBAL_ZERO_FORCE`。
3. 循环中解析视觉数据。
4. 非阻塞读取 `Gimbal_cmd_queue_handle`。
5. 调用 `Gimbal_handle_command()`。
6. 使用 `vTaskDelayUntil()` 保持周期。

## 调试顺序

1. 先确认 INS 状态能进入 `INS_STATE_READY`。
2. 在 `GIMBAL_ZERO_FORCE` 下确认电机无输出。
3. 使用 `GIMBAL_GYRO_MODE` 小幅改变 Yaw/Pitch 目标，确认方向和限幅。
4. 校准 `YAW_CHASSIS_ALIGN_ECD`、`PITCH_HORIZON_ECD` 和 IMU 坐标轴。
5. 再开启视觉通信，检查 USB/UART 数据帧、CRC、在线状态。
6. 视觉模式下先限制目标变化幅度，再逐步调 PID。

## 后续建议

当前源码中已经有决策层目标同步相关字段，但云台层还可以继续增强：

- 视觉离线自动回退到 IMU/手动目标。
- 视觉目标一阶滤波。
- Yaw 单圈目标解缠到连续多圈角附近。
- 模式切换 active_ref 平滑过渡。
- 视觉速度前馈渐入渐出。
- Pitch 机械限位和输出斜率限制。
