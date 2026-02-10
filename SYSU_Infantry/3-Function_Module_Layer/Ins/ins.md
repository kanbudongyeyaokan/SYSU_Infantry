# INS 驱动接口注册（BMI088 Driver Interface）模块使用说明文档
## 1. 模块简介

本模块用于将 **BMI088 IMU 驱动实现**注册为 INS（惯性导航系统）统一接口，使 INS 框架能够通过统一的函数指针接口调用具体硬件驱动，实现硬件无关的姿态解算与数据更新。

## 2. 模块功能介绍

主要功能包括：
- 将 BMI088 的各功能函数绑定到 INS 驱动接口结构体
- 实现 INS 框架与具体 IMU 驱动的解耦
- 支持多种 IMU 驱动的可替换注册
- 提供统一的数据采集与处理入口

---

## 3. 数据结构说明
### 3.1 驱动接口结构体 `Ins_driver_interface_t`

该结构体定义在 `ins.h` 中，用于描述一个 IMU 驱动应实现的统一接口：

```c
typedef struct
{
    bool (*init)(void);                         
    void (*start_read)(void);                  
    bool (*wait_data)(void);                   
    void (*process_data)(Ins_data_t *data, float dt);
} Ins_driver_interface_t;
```
## 4.使用流程
### 4.1 在相应驱动文件中定义驱动接口实例
将对应的功能函数地址赋值给接口结构体成员:
```c
static Ins_driver_interface_t bmi088_driver = {
    .init = BMI088_Init,
    .start_read = BMI088_StartRead,
    .wait_data = BMI088_WaitData,
    .process_data = BMI088_process_data
};
```
### 4.2 注册驱动接口到 INS 框架
在系统初始化阶段调用：
```c
Ins_init(&bmi088_driver);
```
注册完成后，INS 框架内部将保存该接口指针：
current_driver = &bmi088_driver;
### 4.3 周期性调度 INS 更新
在固定周期任务中调用：
```c
Ins_update();
```
INS 框架内部将自动执行：
```c
current_driver->start_read()
current_driver->wait_data()
current_driver->process_data()
```
从而完成数据采集与姿态解算。