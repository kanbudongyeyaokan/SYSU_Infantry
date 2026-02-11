# Shoot 发射机构模块使用说明文档
## 1. 模块简介

本模块用于控制机器人发射机构电机，通过 CAN 总线控制电机，实现：

- 摩擦轮启停
- 单发射击
- 连发射击
- PID 闭环控制
- 向决策层发布射击反馈信息
---

## 2. 初始化
### 2.1 电机初始化

```c
void Shoot_motors_init(void);
```
初始化 3 个 DJI 电机实例;
配置 PID 参数与 CAN 通信参数;
设置电机初始状态为停止;
### 2.2 任务初始化
```c
void Shoot_task_init(void);
```
调用 Shoot_motors_init();
注册射击反馈发布器

## 3.控制逻辑说明
### 3.1 摩擦轮控制
SHOOT_OFF：关闭摩擦轮，目标转速为 0
SHOOT_ON：开启摩擦轮，目标转速为 ±250
```c
Djimotor_set_target(shoot_motors[0], 2500);
Djimotor_set_target(shoot_motors[1], -2500);
```
### 3.2 拨弹盘控制
LOAD_STOP（停止）:
```c
Djimotor_set_status(shoot_motors[2], MOTOR_STOP);
```
LOAD_1_BULLET（单发）:
使用角度环控制;
每次触发转动固定角度 ONE_BULLET_DELTA_ANGLE;
通过模式切换实现边沿触发，防止重复发射

LOAD_BURSTFIRE（连发）:
使用速度环控制
拨弹盘转速与 shoot_rate 成比例
```c
Djimotor_set_target(shoot_motors[2],
    -cmd->shoot_rate * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
```

## 4. PID 运算与电机驱动
```c
Djimotor_Calc_Output(shoot_motors[i]);
```
计算 PID 输出
通过 CAN 总线发送电流指令
驱动电机转动，实现射击动作

## 5.使用示例
```c
//单发射击
Shoot_cmd_send_t cmd;
cmd.shoot_mode = SHOOT_ON;
cmd.loader_mode = LOAD_1_BULLET;
Shoot_handle_command(&cmd);

//连发射击
Shoot_cmd_send_t cmd;
cmd.shoot_mode = SHOOT_ON;
cmd.loader_mode = LOAD_BURSTFIRE;
cmd.shoot_rate = 5;
Shoot_handle_command(&cmd);

//停止射击
Shoot_cmd_send_t cmd;
cmd.shoot_mode = SHOOT_OFF;
cmd.loader_mode = LOAD_STOP;
Shoot_handle_command(&cmd);
