# MahonyAHRS 姿态解算模块使用指南

## 1. Mahony 互补滤波算法简介

Mahony 算法是一种基于 **PI 控制思想的互补滤波算法**，主要特点如下：
- 使用陀螺仪进行姿态积分（短期精度高）
- 使用加速度计测量重力方向（长期稳定）
- 通过比例项（Kp）与积分项（Ki）修正陀螺仪漂移

本实现为 **6轴版本（Gyro + Acc）**，不包含磁力计。
---

## 2. 模块功能介绍

主要功能包括：

- 初始化并维护姿态四元数
- 基于 IMU 数据更新姿态状态
- 提供四元数输出接口
- 提供欧拉角输出接口（Roll、Pitch、Yaw）

---
## 3. 使用流程
1. 初始化模块：Mahony 模块内部已将四元数初始化为单位四元数，无需额外初始化函数，只需保证首次调用前系统稳定。
2. 定期更新姿态：在 IMU 数据更新任务中周期性调用
```c
    MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
```   
3. 获取姿态四元数
```c
    float q[4];
    Mahony_GetQuaternion(q);
```   
4. 获取欧拉角姿态
```c
    float roll, pitch, yaw;
    Mahony_GetEulerAngle(&pitch, &roll, &yaw);
```  