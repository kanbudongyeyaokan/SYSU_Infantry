# 软件看门狗（Watchdog）模块使用说明文档

## 1. 模块简介

本模块实现了一个 **软件看门狗管理机制**，用于监控多个任务或设备对象的“在线/离线”状态。  

## 2. 模块功能介绍

主要功能包括：
- 注册多个看门狗实例（支持多设备监控）
- 支持自定义超时时间（reload_count）
- 支持离线回调（offline_callback）
- 支持上线回调（online_callback）
- 提供统一的周期调度接口
- 提供在线状态查询接口
---
## 3. 数据结构说明（核心思想）

每个 Watchdog 实例包含以下关键成员（逻辑含义）：

- `owner_id`：设备或任务的唯一标识
- `reload_count`：重装载值（超时阈值）
- `temp_count`：当前倒计时计数
- `is_offline`：是否离线标志
- `offline_callback`：离线时触发的回调函数
- `online_callback`：重新上线时触发的回调函数

## 4. 使用流程

### 4.1 注册看门狗实例

在系统初始化阶段，为需要监控的设备或任务注册一个看门狗对象。
```c
Watchdog_init_t wdg_cfg = {
    .owner_id = 1,
    .reload_count = 100,   // 超时阈值（单位：调度周期次数）若100 次周期内没有喂狗，则判定离线
    .callback = Device_Offline_Callback,
    .online_callback = Device_Online_Callback
};

Watchdog_device_t *device_wdg = Watchdog_register(&wdg_cfg);
```
### 4.2 周期性调度
必须在固定周期任务中调用：
```c
void Watchdog_control_all();
```
### 4.3 喂狗操作
在被监控对象“正常工作”时调用：
```c
Watchdog_feed(device_wdg);
```
### 4.4 查询在线状态
```c
uint8_t status = Watchdog_is_online(device_wdg);

if (status)
{
    // 设备在线
}
else
{
    // 设备离线
}
```
## 5.使用示例
```c
Watchdog_device_t *can_wdg;

void CAN_Init(void)
{
    Watchdog_init_t cfg = {
        .owner_id = 0x01,
        .reload_count = 50,   // 50 * 10ms = 500ms 超时
        .callback = CAN_Offline,
        .online_callback = CAN_Online
    };

    can_wdg = Watchdog_register(&cfg);
}

void CAN_Rx_Callback(void)
{
    //...
    Watchdog_feed(can_wdg);

}

void Task_10ms(void)
{
    Watchdog_control_all();
}
```