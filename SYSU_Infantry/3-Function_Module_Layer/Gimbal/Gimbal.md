# Gimbal 云台控制模块说明

## 1. 模块定位

本模块负责 RoboMaster 步兵机器人双轴云台的姿态控制，控制对象为：

- `Yaw` 轴 GM6020 电机
- `Pitch` 轴 GM6020 电机
- IMU 姿态反馈
- NUC 视觉自瞄目标

当前控制架构包含两类主要目标源：

- `GIMBAL_GYRO_MODE`
  由遥控器/键鼠给出手动目标，目标量以世界系角度累加形成。
- `GIMBAL_VISION_MODE`
  由 NUC 下发绝对目标角 `target_yaw`、`target_pitch`，并可附带角速度前馈 `target_yaw_v`、`target_pitch_v`。

底层执行仍然是同一套云台角度环 + 速度环 PID，因此模式切换时最容易出问题的不是执行器变化，而是：

- 目标角来源变化
- 视觉参考系与 IMU 世界系存在微小偏差
- 视觉单圈角与 IMU 多圈角表示方式不同
- 视觉速度前馈在切换瞬间被突然注入

本模块现已按“无扰切换（Bumpless Transfer）”思路进行了统一处理。

## 2. 当前实现的统一目标层

为避免 IMU 模式和视觉模式直接抢占 PID 目标，代码中采用了三层目标概念：

- `manual_ref`
  决策层维护的手动目标，由遥控器/键鼠增量累加。
- `vision_ref`
  视觉给出的绝对角目标，先经过偏差补偿、Yaw 解缠和一阶滤波。
- `active_ref`
  真正送给底层 PID 的统一执行目标。

核心原则只有一条：

> 底层 PID 只认 `active_ref`，不直接认遥控器目标，也不直接认视觉目标。

对应到代码中：

- 云台层在 `gimbal.c` 内维护 `gimbal_bumpless_state.active_yaw_target` 和 `active_pitch_target`
- 决策层通过 `active_yaw_target` / `active_pitch_target` 回读当前真实执行目标
- 视觉期间持续把手动目标追到 `active_ref`，退出视觉时不会跳回旧手动目标

## 3. 无扰切换设计目标

实现重点是同时解决两类问题：

1. 角度跳变
   模式切换瞬间不能让 Setpoint 直接跳到新模式目标。
2. 控制力矩突变
   模式切换瞬间不能因为积分清零、前馈突加、目标突跳而让输出电流尖峰出现。

当前策略是：

- 先保证 `sp(t0+) = sp(t0-)`
- 再通过过渡器逐步逼近新目标
- 积分不直接清零，而是短时冻结
- 视觉前馈单独渐入渐出，不与目标切换硬绑定

## 4. 需求整合与方案落地

### 4.1 目标值同步

目标值同步的核心不是“切换后立刻等于新目标”，而是“切换瞬间连续”。

当前代码实现方式：

- 模式源变化时，先锁存当前 `active_ref` 作为过渡起点
- 视觉目标先做偏差补偿，再做 `Yaw` 解缠
- 视觉目标再做一阶低通滤波，抑制抖动
- 最后由过渡器把 `active_ref` 从旧目标平滑拉向新目标

对应逻辑：

```text
RC/键鼠增量 ---> manual_ref ----\
                                 \
Vision绝对角 ---> 补差 ---> 解缠 ---> 滤波 ---> desired_ref ---> 过渡器 ---> active_ref ---> PID
                                 /
IMU姿态反馈 --------------------/
```

退出视觉时的重点是：

- 决策层在视觉工作期间持续执行 `manual_ref <- active_ref`
- 切回手动的第一拍再同步一次

这样退出视觉不会回跳到旧的遥控器目标。

### 4.2 PID 状态遗传

IMU 与 Vision 模式切换时，不建议简单清空积分项。

原因：

- 被控对象没变，电机仍在工作
- 当前积分往往已经承担了抗重力、摩擦补偿和静差消除
- 如果切换瞬间把 `Iout` 清零，输出力矩会先塌陷，再重新积累

当前代码采用的策略是：

- 保留原有 `Iout`
- 切换初期仅把 `Ki` 暂时置零
- 过渡窗口结束后恢复原 `Ki`

代码等价含义：

- `Gimbal_set_integral_hold(true)` 只冻结积分继续累加
- 不清历史积分状态
- `Gimbal_set_integral_hold(false)` 再恢复原参数

这一策略适合当前工程，因为：

- 模式切换前后仍是同一个云台闭环
- 只是目标源发生变化，不是控制对象切换

### 4.3 坐标变换补差

若视觉参考系与 IMU 世界系存在小偏差，代码中先用简单偏置补偿处理：

```text
yaw_cmd   = target_yaw   + gimbal_vision_yaw_bias_deg
pitch_cmd = target_pitch + gimbal_vision_pitch_bias_deg
```

然后对 `Yaw` 再执行“解缠到当前多圈角附近”：

```text
unwrap_to_nearest(vision_yaw, active_ref_yaw)
```

这样可以解决两个典型问题：

- 视觉零点与 IMU 零点不完全重合
- 视觉单圈角在 `-180 ~ 180` 或 `0 ~ 360` 跳变，而 IMU 是连续多圈角

如果后续实车发现 `Pitch` 变化会诱发 `Yaw` 偏差，可再升级为 2x2 小角度耦合补偿；当前实现先保留最小可用方案，便于稳定调试。

### 4.4 过渡平滑技术

当前工程已用到三层平滑：

1. 视觉目标一阶滤波
   作用：抑制视觉解算抖动，避免刚切入视觉时目标抖动直接传到底层。
2. `S` 曲线目标过渡
   作用：保证 `active_ref` 的位置连续，并减轻开始/结束时的速度突变。
3. 视觉前馈渐入渐出
   作用：防止 `target_yaw_v`、`target_pitch_v` 在切换瞬间全量打到输出。

对应时间参数：

- `GIMBAL_MODE_BLEND_TIME_S`
  模式切换目标过渡时间
- `GIMBAL_VISION_REF_FILTER_TAU_S`
  视觉目标滤波时间常数
- `GIMBAL_VISION_FF_BLEND_TIME_S`
  视觉前馈渐入渐出时间
- `GIMBAL_I_HOLD_TIME_S`
  积分冻结时间

## 5. 流程图描述

虽然代码里没有额外拆成多个显式枚举态，但当前逻辑等价于下面的状态流：

```text
[ACTIVE_IMU]
  |
  | 请求进入视觉 && vision在线
  v
[PREPARE_TO_VISION]
  锁存 q0 = active_ref
  视觉目标做补差、解缠、滤波
  ff_blend 置为渐入起点
  Ki 进入短时冻结窗口
  v
[TRANSITION_TO_VISION]
  active_ref = S曲线(q0 -> q_target)
  ff_blend   = ramp(0 -> 1)
  v
[ACTIVE_VISION]
  持续刷新视觉目标
  manual_ref <- active_ref
  |
  | 视觉离线 或 请求退出视觉
  v
[PREPARE_TO_IMU]
  manual_ref = active_ref
  ff_blend 准备衰减
  Ki 短时冻结
  v
[TRANSITION_TO_IMU]
  active_ref 保持连续
  ff_blend = ramp(1 -> 0)
  v
[ACTIVE_IMU]
```

另外，`GIMBAL_ZERO_FORCE` 作为失能路径单独处理：

- 进入失能时，`active_ref` 收回当前实测姿态
- 退出失能时，从当前姿态重新起步
- 避免恢复使能后追历史旧目标

## 6. 当前代码落点

### 6.1 云台层

文件：`3-Function_Module_Layer/Gimbal/gimbal.c`

关键实现：

- `Gimbal_bumpless_state_t`
  统一保存 `active_ref`、过渡器、视觉滤波、前馈渐入和积分冻结状态
- `Gimbal_unwrap_to_nearest`
  视觉 `Yaw` 单圈角对齐到当前多圈角附近
- `Gimbal_lpf_step`
  视觉绝对角一阶滤波
- `Gimbal_start_transition`
  锁存切换前 `active_ref`，启动平滑过渡
- `Gimbal_set_integral_hold`
  冻结 `Ki`、保留 `Iout`
- `vision_ff_blend`
  视觉速度前馈渐入渐出

### 6.2 决策层

文件：`3-Function_Module_Layer/Decision_making/decision_making.c`

关键实现：

- `Decision_sync_gimbal_manual_target()`
  视觉期间持续同步手动目标，退出视觉首拍再同步一次
- `Keyboard_ctrl_set()`
  键鼠控制固定切回手动 IMU 模式，并先同步手动目标
- READY 首拍同步
  云台从未就绪到就绪时，先继承 `active_ref`
- `ZERO_FORCE` 保护
  急停/失能期间不继续把旧手动目标往后积累

文件：`3-Function_Module_Layer/Decision_making/decision_making.h`

新增反馈字段：

- `imu_pitch_angle`
- `active_yaw_target`
- `active_pitch_target`

## 7. 调参与实战建议

建议按下面顺序调试：

1. 先确认 `Yaw` 解缠正常
   视觉目标在 `-180/180` 附近切换时，不应出现大步跳变。
2. 再调 `gimbal_vision_yaw_bias_deg` / `gimbal_vision_pitch_bias_deg`
   先消掉静态偏差。
3. 再调 `GIMBAL_MODE_BLEND_TIME_S`
   如果切换仍然“抽”，适当增大；如果跟手太慢，再适当减小。
4. 再调 `GIMBAL_VISION_FF_BLEND_TIME_S`
   如果电流尖峰明显，优先放慢前馈渐入。
5. 最后微调 `GIMBAL_I_HOLD_TIME_S`
   让切换初期积分不乱冲，但也不要冻结过久。

## 8. 后续可扩展项

如果后续需要进一步提升鲁棒性，可继续增加：

- `Pitch` 视觉目标限幅
- 电流输出 `slew rate limit`
- 在线偏差自校正
- 相机到 IMU 的更完整外参补偿

当前版本已经覆盖本次需求中的四个核心点：

- 目标值同步
- PID 状态遗传
- 坐标变换补差
- 过渡平滑技术
