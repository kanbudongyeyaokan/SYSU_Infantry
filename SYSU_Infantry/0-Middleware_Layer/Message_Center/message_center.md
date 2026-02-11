# Message Center 消息中心模块使用指南

## 1. 发布-订阅（Publish-Subscribe）机制简介

本模块基于发布-订阅（Pub/Sub）通信模型实现任务间解耦的数据传输，其核心思想如下：

- 发布者（Publisher）只负责发布数据，不关心谁在接收
- 订阅者（Subscriber）只关心某一话题（Topic）的数据
- 通过消息中心统一管理话题与订阅关系
- 支持一个话题对应多个订阅者（广播机制）

---

## 2. 模块功能介绍

本模块主要提供以下功能：

- 注册发布者（Topic）
- 注册订阅者（Subscriber）
- 发布数据到指定话题
- 从订阅者队列中读取数据
- 自动管理话题链表与订阅者链表
- 提供线程安全的数据访问机制

---

## 3. 使用流程

### 3.1 注册发布者（Topic）

用于创建或获取一个话题对象：

```c
    Publisher_t *pub = Pub_register("imu_data", sizeof(IMU_Data_t));
```
若该话题已存在，则直接返回已有 Publisher;同一话题的数据长度必须保持一致

### 3.2 注册订阅者（Subscriber）
```c
    Subscriber_t *sub = Sub_register("imu_data", sizeof(IMU_Data_t));
```
若话题尚未创建，将自动调用 Pub_register 创建;每个订阅者内部维护一个循环消息队列;可为同一话题注册多个订阅者

### 3.3 发布消息
向某个话题推送数据，系统会广播给所有订阅该话题的订阅者：
```c
    IMU_Data_t imu;
    Pub_push_message(pub, &imu);
```
### 3.4 读取订阅者消息
从订阅者队列中取出一条数据：
```c
    IMU_Data_t recv_data;

    if (Sub_get_message(sub, &recv_data))
    {
        // 成功读取数据
    }
```
## 4.使用示例
```c
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

Subscriber_t *sub = Sub_register("attitude", sizeof(Attitude_t));
Publisher_t *pub = Pub_register("attitude", sizeof(Attitude_t));

Attitude_t att = {10.0f, 2.0f, 30.0f};
Pub_push_message(pub, &att);

Attitude_t recv;
if (Sub_get_message(sub, &recv))
{
    // 使用 recv 数据
}
```