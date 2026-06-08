# DJI Motor 大疆电机驱动说明

`dji_motor.c/.h` 是本仓库最核心的设备驱动之一，用于统一管理 RoboMaster 常用 DJI 电机：M3508、M2006、GM6020。它封装 CAN 反馈解析、多圈角度解算、PID 计算、软件看门狗离线保护和 CAN 控制帧聚合发送。

## 设计目标

1. 用同一套结构体描述不同 DJI 电机。
2. 把 CAN 收发细节封装在驱动层。
3. 支持速度环、角度环、电流环和串级环。
4. 让底盘、云台、发射机构只关心目标值和反馈值。
5. 实现“算发分离”：功能模块计算输出，电机任务统一发送 CAN。

## 支持电机

```c
typedef enum {
    M3508 = 0,
    M2006,
    GM6020
} Djimotor_type_e;
```

当前控制帧分组：

| 控制帧 | 适用范围 |
| --- | --- |
| `0x1FF` | GM6020 ID 1-4 |
| `0x200` | M3508/M2006 ID 1-4 |
| `0x2FF` | GM6020 ID 5-7，M3508/M2006 ID 5-8 |

驱动会根据电机所在 CAN 总线、控制帧 ID 和 `tx_id` 把 `out_current` 填入对应 8 字节控制帧。

## 核心结构体

### 反馈数据

```c
typedef struct {
    uint16_t last_ecd;
    uint16_t current_ecd;
    float current_angle;
    float angular_velocity;
    float linear_velocity;
    int16_t real_current;
    uint8_t motor_temperature;
    int32_t total_round;
    float total_angle;
} Djimotor_measure_t;
```

`Decode_djimotor()` 会解析 CAN 回传帧：

- `[0-1]`：编码器值
- `[2-3]`：转速 rpm
- `[4-5]`：实际电流
- `[6]`：温度

并根据编码器跨越 0/8191 的跳变维护 `total_angle`。

### 控制器

```c
typedef struct {
    Djimotor_closeloop_e close_loop;
    Djimotor_feedback_source_e angle_source;
    Djimotor_feedback_source_e speed_source;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    Pid_instance_t current_pid;
    Pid_instance_t angle_pid;
    Pid_instance_t speed_pid;
    float pid_target;
    float speed_feedforward;
} Djimotor_controller_t;
```

反馈源可以来自电机自身，也可以来自外部指针。例如云台 Yaw 电机角度反馈使用 INS `total_yaw`，速度反馈使用 IMU 角速度。

### 电机实例

```c
typedef struct {
    char motor_name[16];
    Djimotor_type_e motor_type;
    Djimotor_status_e motor_status;
    Djimotor_measure_t motor_measure;
    Djimotor_controller_t motor_pid;
    Can_controller_t *can_controller;
    int16_t deadzone_compensation;
    Watchdog_device_t *wdg;
    int16_t out_current;
} Djimotor_device_t;
```

注意：当前源码中 `can_controller` 是指针，旧文档中写成结构体值已经过时。

## 闭环模式

```c
typedef enum {
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
} Djimotor_closeloop_e;
```

`Djimotor_Calc_Output()` 中的计算关系：

- `OPEN_LOOP`：目标值直接作为输出。
- `CURRENT_LOOP`：电流 PID。
- `SPEED_LOOP`：速度 PID。
- `ANGLE_LOOP`：角度 PID。
- `SPEED_AND_CURRENT_LOOP`：速度环输出作为电流环目标。
- `ANGLE_AND_SPEED_LOOP`：角度环输出加速度前馈后作为速度环目标。

如果电机状态为 `MOTOR_STOP`，驱动会输出 0，并重置 PID 状态。

## 初始化流程

```c
Djimotor_init_config_t cfg = {
    .motor_name = "CHASSIS_FR",
    .motor_type = M3508,
    .motor_status = MOTOR_ENABLED,
    .deadzone_compensation = 500,
    .motor_controller_init = {
        .close_loop = SPEED_LOOP,
        .speed_source = MOTOR_FEEDBACK,
        .speed_pid = {
            .kp = 15,
            .ki = 0,
            .kd = 0,
            .max_iout = 3000,
            .max_out = 15000,
            .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL,
        },
    },
    .can_init = {
        .can_handle = &hcan1,
        .can_id = 0x200,
        .tx_id = 1,
        .rx_id = 0x201,
    },
};

Djimotor_device_t *motor = DJI_Motor_Init(&cfg);
```

`DJI_Motor_Init()` 会：

1. 分配并清零电机对象。
2. 拷贝基础配置。
3. 初始化 current/angle/speed PID。
4. 注册 CAN 接收设备，回调为 `Decode_djimotor()`。
5. 注册软件看门狗，默认 `reload_count = 3`。
6. 把电机加入全局 `motor_instances`，供统一发送遍历。

## 控制流程

功能模块每个控制周期通常这样做：

```c
Djimotor_set_status(motor, MOTOR_ENABLED);
Djimotor_set_target(motor, target);
Djimotor_Calc_Output(motor);
```

`Djimotor_Calc_Output()` 只更新：

```c
motor->out_current
```

它不发送 CAN。

应用层 `Motor_control_task()` 周期调用：

```c
Djimotor_Send_All_Bus();
```

统一把所有已注册电机的 `out_current` 打包到 CAN1/CAN2 的 `0x1FF/0x200/0x2FF` 报文发送。

## 动态切换控制器

```c
void Djimotor_change_controller(Djimotor_device_t *motor,
                                Djimotor_controller_init_t ctrl_params);
```

用于从速度环切到角度环、修改 PID 参数或切换反馈源。当前实现会临时关闭中断保护参数切换，并重新初始化三个 PID 实例。

使用示例：

```c
Djimotor_controller_init_t ctrl = {
    .close_loop = ANGLE_AND_SPEED_LOOP,
    .angle_source = MOTOR_FEEDBACK,
    .speed_source = MOTOR_FEEDBACK,
    .angle_pid = {
        .kp = 8,
        .max_out = 2000,
        .optimization = PID_OUTPUT_LIMIT,
    },
    .speed_pid = {
        .kp = 20,
        .ki = 5,
        .max_iout = 6000,
        .max_out = 20000,
        .optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL,
    },
};

Djimotor_change_controller(loader_motor, ctrl);
```

## 离线保护

每个电机注册 `bsp_wdg`：

- 收到 CAN 反馈时 `Watchdog_feed()`。
- 看门狗超时后执行 `Motor_Offline_Callback()`。
- 离线回调会清零速度/电流反馈，并把电机切到 `MOTOR_STOP`。

因此功能模块在长期未收到反馈时不应继续认为电机可控。

## 常见问题

| 现象 | 排查方向 |
| --- | --- |
| `DJI_Motor_Init()` 返回 `NULL` | 检查 `MAX_MOTOR_COUNT`、CAN 设备表是否满、内存是否不足 |
| 有反馈但不动 | 检查 `Motor_control_task` 是否运行、`Djimotor_Calc_Output()` 是否被调用 |
| 方向反了 | 检查目标正负号、电机安装方向、轮序 |
| 角度环跳变 | 检查 `total_angle` 是否连续，反馈源是否是 `MOTOR_FEEDBACK` 或外部 IMU |
| 掉线保护频繁触发 | 检查 CAN 反馈 ID、总线负载、看门狗周期 |

## 调试建议

1. 单电机先用 `SPEED_LOOP` 低速正反转测试。
2. 再测试 `ANGLE_LOOP` 或 `ANGLE_AND_SPEED_LOOP`。
3. 确认 `Djimotor_get_measure()` 返回的转速、电流、温度合理。
4. 最后接入底盘/云台/发射模块。
