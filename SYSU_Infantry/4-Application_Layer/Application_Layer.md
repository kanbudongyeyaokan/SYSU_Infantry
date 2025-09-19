
> 该层为应用层，主要包括机器人各个模块的任务编写，比如底盘、云台、发射机构、遥控器数据解析等。  
> 目的：只需调用 **Funciton_Module** 层的接口即可完成任务,因此代码会非常简洁，所有功能都在Funciton_Module层实现，这里只做执行。
> 所有任务均在 **Robot_Task**中的robot_task.h/.c创建，任务接口函数亦全部集中于此。
> 机器人的所有物理参数，所有模块（底盘/云台/发射机构）的状态机定义都在**Robot_Task**中的Robot_information.h中实现

---
## 任务框架思想  
> 将机器人当作人类来设计任务。

---

## 机器人任务分类  
| 类别 | 类比 | 职责 |
| --- | --- | --- |
| 决策 Decision-making | 大脑 | 决定如何运动，并向执行层下达命令 |
| 感知 Perception | 眼 / 耳 / 鼻 | 采集外部与内部数据 |
| 执行 Execution | 手 / 脚 / 内循环 | 完成所有模块的实际运动 |

---

## 任务创建  
> 所有任务均在 **Robot_Task** 里创建。

>任务框架如下：

### ① 决策 Decision-making  
| 任务 | 说明 | 频率 |
| --- | --- | --- |
| `Decision_making_task` | 控制模式设定；获取并中转遥控器/视觉/导航等控制量 | 500 Hz |

---

### ② 感知 Perception  
| 任务 | 说明 | 频率 |
| --- | --- | --- |
| `Ins_task` | 惯性导航数据（IMU 等） | 1 kHz |
| `Referee_task` | 裁判系统数据 | 1 kHz |
| `Vision_task` | 视觉通信（与视觉模块交互） | 自定义（后续可能并入决策层） |
| `Navigation_task` | 导航通信（与导航模块交互） | 自定义（后续可能并入决策层） |

---

### ③ 执行 Execution  
| 任务 | 说明 | 频率 |
| --- | --- | --- |
| `Chassis_task` | 底盘控制 + 功率限制 | 200 Hz |
| `Gimbal_task` | 云台控制 | 200 Hz |
| `Shoot_task` | 射击机构控制 | 200 Hz |
| `Motor_task` | 所有电机控制 | 1 kHz |
| `Watch_dog_task` | 看门狗，监控各模块状态 | 1 kHz |
| `Screen_task` | 屏幕，用于查看机器人当前状态 | 100 Hz |

---

