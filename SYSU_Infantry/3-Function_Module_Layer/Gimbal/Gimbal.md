## 云台模块与 INS 接口速览

- 初始化函数的other_angle_feedback_ptr这几个other feedback需补充(static attitude_t *gimbal_imu_data)
- pid初始化按需修改


# Gimbal 云台控制模块
## 1. 模块简介 
本模块为 云台控制功能模块（Yaw / Pitch），实现：
云台电机初始化
姿态闭环控制（角度环 + 速度环 PID）
接收决策层控制指令
向消息中心发布云台反馈信息
支持 Shell 在线修改 PID 参数

## 2.使用流程
### 2.1 初始化模块
初始化内容包括：获取 INS 姿态数据指针;
初始化 Yaw / Pitch 电机;
注册消息发布者 gimbal_feedback
```c
Gimbal_task_init();
```

### 2.2 接收控制指令并控制云台
```c
Gimbal_cmd_send_t cmd;
cmd.gimbal_mode = GIMBAL_GYRO_MODE;
cmd.yaw = 30.0f;
cmd.pitch = -10.0f;

Gimbal_handle_command(&cmd);
```
模块内部自动发布云台反馈数据：
```c
Pub_push_message(gimbal_pub, &gimbal_feedback);
```

## 3.Shell 在线调参
命令格式:
yaw_pid -s <kp> <ki> <kd> [max_out] [max_iout]
yaw_pid -a <kp> <ki> <kd> [max_out] [max_iout]

## 4. 控制模式说明
### 4.1 零力模式（失能）
GIMBAL_ZERO_FORCE
电机停止输出,云台自由状态

### 4.2 陀螺仪闭环模式
GIMBAL_GYRO_MODE
使能 Yaw / Pitch 电机;
目标角度来自控制指令;
反馈来自 INS 姿态;
使用双环 PID 控制.