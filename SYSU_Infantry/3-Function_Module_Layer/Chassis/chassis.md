# Chassis 底盘控制模块使用说明

## 1. 模块简介

本模块用于实现机器人底盘控制与运动学解算，完成以下功能：

- 初始化底盘物理参数与电机设备  
- 接收决策层底盘控制指令  
- 进行运动学解算（vx, vy, wz → 各轮转速）  
- 控制 4 个 DJI 电机输出  
- 发布底盘反馈信息给决策层  

---

## 2. 模块结构

整体控制流程：
决策层指令 (Chassis_cmd_send_t)
↓
Chassis_Update_Control()
↓
Chassis_kinematics_solve()
↓
DJI Motor PID 计算
↓
CAN 总线发送
↓
发布 chassis_feedback

## 3. 接口说明
### 3.1 任务初始化
初始化底盘模块并注册底盘反馈发布者
```c
void Chassis_task_init(void);
```
### 3.2 底盘初始化
初始化内容包括：
底盘物理参数（轮半径、轮距、底盘类型）;4 个 DJI 电机实例;电机 PID 参数;CAN 通信参数
```c
void Chassis_init(void);
```
### 3.3 底盘控制更新
在控制周期中周期性调用:
```c
void Chassis_Update_Control(const Chassis_cmd_send_t *cmd);
```
进行运动学解算和发布反馈信息
