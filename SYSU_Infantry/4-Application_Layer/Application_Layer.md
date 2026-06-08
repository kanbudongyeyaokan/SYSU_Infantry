# Application Layer 应用层说明

应用层位于 `4-Application_Layer/`，负责 FreeRTOS 任务组织、队列收发、控制主循环和测试任务。它是机器人运行时的调度层。

应用层的目标是薄而清楚：任务里尽量只做初始化、收发数据、调用功能模块接口和周期调度，复杂控制逻辑下沉到 `3-Function_Module_Layer`。

## 目录总览

| 目录 | 主要文件 | 职责 |
| --- | --- | --- |
| `Robot_Task` | `robot_task.c/.h` | 全车任务句柄、队列句柄、任务创建入口 |
| `Decision_Making` | `Decision_making_task.c`、`Shell_task.c` | 决策任务和 Shell 任务入口 |
| `Execution/Chassis_Task` | `Chassis_task.c/.h` | 底盘执行任务 |
| `Execution/Gimbal_Task` | `Gimbal_task.c/.h` | 云台执行任务 |
| `Execution/Shoot_Task` | `shoot_task.c/.h` | 发射机构执行任务 |
| `Execution/Motor_Task` | `motor_task.c/.h` | 电机 CAN 聚合发送任务 |
| `Execution/Watchdog_Task` | `watchdog_task.c/.h` | 软件看门狗调度任务 |
| `Perception/Ins_Task` | `Ins_task.c/.h` | INS 更新任务 |
| `Perception/Referee_Task` | `Referee_task.c/.h` | 裁判系统和 UI 任务 |
| `Buzzer_Alarm_Task` | `buzzer_alarm_task.c/.h` | 蜂鸣器报警任务 |
| `For_Testing_Task` | 多个测试任务 | 单模块或链路测试 |

## 启动入口

任务创建链路：

```text
Core/Src/main.c
  -> MX_FREERTOS_Init()
     -> Core/Src/freertos.c
        -> Robot_task_init()
           -> 创建队列
           -> 创建机器人任务
```

`Robot_task_init()` 位于 `4-Application_Layer/Robot_Task/robot_task.c`，是应用层最重要的入口。

## 队列设计

当前主链路队列在 `robot_task.c` 中创建：

```c
Chassis_cmd_queue_handle = xQueueCreate(1, sizeof(Chassis_cmd_send_t));
Gimbal_cmd_queue_handle  = xQueueCreate(1, sizeof(Gimbal_cmd_send_t));
Shoot_cmd_queue_handle   = xQueueCreate(1, sizeof(Shoot_cmd_send_t));

Gimbal_feedback_queue_handle  = xQueueCreate(1, sizeof(Gimbal_feedback_info_t));
Chassis_feedback_queue_handle = xQueueCreate(1, sizeof(Chassis_feedback_info_t));
Shoot_feedback_queue_handle   = xQueueCreate(1, sizeof(Shoot_feedback_info_t));

Buzzer_cmd_queue_handle = xQueueCreate(5, sizeof(uint8_t));
```

命令队列长度为 1，配合 `xQueueOverwrite()` 使用，表示“只关心最新控制指令”。这适合遥控器、键鼠和视觉控制这类实时目标。

维护提示：命令队列和反馈队列的结构体类型不同，读写时不要混用。新增任务时应在 `robot_task.h` 中声明对应句柄，并在初始化阶段创建。

## 当前任务列表

| 任务 | 入口函数 | 优先级 | 栈大小 | 主要周期/行为 |
| --- | --- | --- | --- | --- |
| 蜂鸣器报警 | `Buzzer_alarm_control_task` | `osPriorityNormal` | 1024 | 先于错误系统启动，用于报警 |
| INS | `Ins_task` | `osPriorityHigh` | 2048 | 约 1 tick 更新一次 |
| 看门狗 | `Watchdog_control_task` | `osPriorityHigh` | 512 | `osDelay(100)` |
| 电机发送 | `Motor_control_task` | `osPriorityNormal` | 512 | 1 ms 调用 `Djimotor_Send_All_Bus()` |
| 裁判系统 | `Referee_task` | `osPriorityNormal` | 512 | 100 ms 刷新 UI/动态数据 |
| 决策 | `Decision_making_task` | `osPriorityAboveNormal` | 1024 | 1 tick 控制决策 |
| 底盘 | `Chassis_control_task` | `osPriorityNormal` | 512 | 队列等待 100 tick，超时零力矩 |
| 云台 | `Gimbal_control_task` | `osPriorityAboveNormal` | 512 | 1 tick 处理云台和视觉 |
| 发射 | `Shoot_control_task` | `osPriorityNormal` | 512 | 5 ms |

部分测试任务已在代码中保留头文件和句柄，但默认没有创建。

## 任务职责

### Robot Task

`Robot_task_init()` 负责：

1. 创建命令队列和反馈队列。
2. 创建蜂鸣器报警任务。
3. 初始化错误系统。
4. 创建 INS、看门狗、电机、裁判、决策、底盘、云台、发射任务。

新增正式任务应在这里统一创建，方便查看整车任务表。

### Decision Task

`Decision_making_task()` 的主循环：

```text
Receive_feedback_infomation()
Robot_set_command()
Calc_offset_angle()
Check_fatal_estop()
Send_command_to_all_task()
vTaskDelayUntil(...)
```

它负责从遥控器、键鼠、视觉和反馈数据中生成底盘、云台、发射指令，并在 Fatal 错误时覆盖为安全态。

### Execution Tasks

- `Chassis_control_task()`：读取底盘命令，调用 `Chassis_Update_Control()`；超时未收到命令时进入 `CHASSIS_ZERO_FORCE`。
- `Gimbal_control_task()`：初始化云台，解析视觉数据，调用 `Gimbal_handle_command()`。
- `Shoot_control_task()`：读取发射命令，调用 `Shoot_handle_command()`。
- `Motor_control_task()`：等待系统稳定后，以 1 ms 周期统一发送所有 DJI 电机 CAN 控制帧。

### Perception Tasks

- `Ins_task()`：选择 IMU 驱动，调用 `Ins_init()` 和 `Ins_update()`，通过 `Ins_get_data()` 给上层提供姿态。
- `Referee_task()`：获取裁判系统数据，发送 HUD 静态图层心跳，动态更新电容条等 UI。

## 测试任务

`For_Testing_Task` 中保留了若干测试任务：

- `message_test_task.c`：测试 Message Center 一对一、多对多和并发访问。
- `chassis_motor_integration_test.c`：历史 Pub/Sub 形式的底盘电机链路测试任务。
- `rc_test_task.c`：当前内容实际是 DJI Motor Lib 验收测试，不是遥控器测试。
- `can_motors_test_task.c`：当前主体被注释，保留为历史测试草稿。

注意：主控制链路当前使用 FreeRTOS Queue。启用测试任务前，应确认测试任务使用的通信方式与当前主任务一致。

## 新增任务建议

1. 任务函数放入合适的 `Decision_Making`、`Execution`、`Perception` 或 `For_Testing_Task` 子目录。
2. 在 `robot_task.h` 声明任务句柄和必要队列。
3. 在 `Robot_task_init()` 中创建队列和任务。
4. 周期任务使用 `vTaskDelayUntil()` 保持固定节拍。
5. 任务栈大小从保守值开始，使用调试工具观察余量后再调整。
6. 每个任务必须有安全默认态，不能因为队列超时或外设离线继续输出旧目标。

## 调试建议

- 首次上车建议只启用最小任务集：INS、Motor、Decision、Chassis。
- 电机相关问题优先检查 `Motor_control_task` 是否运行。
- 控制量异常先看队列写入和功能层输入结构体。
- 频繁离线或抖动先看 `bsp_wdg`、CAN 错误日志和任务周期。
