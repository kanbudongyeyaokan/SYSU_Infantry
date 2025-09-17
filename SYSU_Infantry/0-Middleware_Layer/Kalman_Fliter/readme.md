# 卡尔曼滤波器模块使用指南

本模块提供两种卡尔曼滤波器实现：标准卡尔曼滤波器(KF)和扩展卡尔曼滤波器(EKF)。这些滤波器可以应用于不同的场景，提供稳定可靠的数据融合和状态估计。

## 1. 扩展卡尔曼滤波器(EKF)

扩展卡尔曼滤波器适用于非线性系统的状态估计，在我们的项目中主要用于姿态估计。

### 1.1 EKF 模块介绍

EKF 模块位于 `algorithm_ekf.h/c` 中，提供以下主要功能：

- 初始化 EKF 参数和状态
- 基于IMU数据进行状态预测
- 使用传感器数据进行状态更新
- 静态检测和零偏校准
- 姿态表示转换(四元数与欧拉角互转)

### 1.2 在 BMI088 中的应用

BMI088 驱动模块中使用 EKF 实现了姿态估计功能。通过解耦设计，BMI088 驱动只负责传感器数据获取，而将姿态估计算法放在独立的 EKF 模块中。这样的设计有利于代码的维护和复用。

#### 使用流程

1. 初始化 EKF 参数

```c
// 在BMI088初始化函数中
Ekf_config_t ekf_config = {
    .process_noise_q = 0.001f,       // 过程噪声协方差
    .measurement_noise_r = 0.05f,     // 测量噪声协方差
    .gyro_bias_noise = 0.0001f,       // 陀螺仪零偏噪声
    .dt = 0.001f,                     // 采样周期(s)
    .enable_bias_correction = true,   // 启用零偏校正
    .static_threshold = 0.2f          // 静态检测阈值
};

// 初始化EKF
Bmi088_ekf_init(bmi088, &ekf_config);
```

2. 定期更新 EKF 状态

```c
// 在姿态估计任务中定期调用
Bmi088_ekf_update(bmi088);
```

3. 获取姿态信息

```c
// 获取四元数表示的姿态
Quaternion_t* q = Bmi088_get_quaternion(bmi088);

// 获取欧拉角表示的姿态(单位：度)
Euler_angles_t* euler = Bmi088_get_euler_angles(bmi088);
```

### 1.3 EKF 在其他模块中的使用

您可以直接调用 EKF 模块提供的 API 在其他应用中使用：

```c
#include "algorithm_ekf.h"

// 创建EKF状态结构体
Ekf_state_t ekf_state;

// 配置EKF参数
Ekf_config_t ekf_config = {
    .process_noise_q = 0.001f,
    .measurement_noise_r = 0.05f,
    .gyro_bias_noise = 0.0001f,
    .dt = 0.001f,
    .enable_bias_correction = true,
    .static_threshold = 0.2f
};

// 初始化EKF
Ekf_init(&ekf_state, &ekf_config);

// 在传感器数据更新时调用
void process_imu_data(float acc_x, float acc_y, float acc_z, 
                      float gyro_x, float gyro_y, float gyro_z) {
    // 构造数据数组
    float acc[3] = {acc_x, acc_y, acc_z};
    float gyro[3] = {gyro_x, gyro_y, gyro_z};
    
    // 更新EKF
    Ekf_update(&ekf_state, acc, gyro);
    
    // 使用更新后的姿态数据
    float roll = ekf_state.euler.roll;
    float pitch = ekf_state.euler.pitch;
    float yaw = ekf_state.euler.yaw;
}
```

## 2. 标准卡尔曼滤波器(KF)

标准卡尔曼滤波器适用于线性系统，可用于传感器数据滤波、目标跟踪等场景。

### 2.1 KF 模块介绍

KF 模块位于 `algorithm_kf.h/c` 中，提供两种实现：

- **标量卡尔曼滤波器**：单变量系统，如传感器数据滤波
- **二阶卡尔曼滤波器**：状态包含位置和速度的系统，如运动目标跟踪

### 2.2 标量卡尔曼滤波器使用示例

适用于简单传感器数据滤波，如温度、距离等单一变量：

```c
#include "algorithm_kf.h"

// 创建滤波器实例
KF_scalar_t kf;

// 初始化滤波器
void init_filter(void) {
    KF_config_t config = {
        .Q = 0.01f,  // 过程噪声
        .R = 0.1f,   // 测量噪声
        .dt = 0.01f  // 采样周期
    };
    // 初始值为0，初始方差为1
    KF_scalar_init(&kf, &config, 0.0f, 1.0f);
}

// 过滤传感器数据
float filter_sensor_data(float raw_data) {
    // 无控制输入的情况下，第三个参数为0
    return KF_scalar_step(&kf, raw_data, 0.0f);
}
```

### 2.3 二阶卡尔曼滤波器使用示例

适用于需要同时估计位置和速度的系统，如物体运动跟踪：

```c
#include "algorithm_kf.h"

// 创建滤波器实例
KF_2order_t kf;

// 初始化滤波器
void init_tracker(void) {
    KF_config_t config = {
        .Q = 0.1f,   // 加速度噪声
        .R = 0.5f,   // 位置测量噪声
        .dt = 0.01f  // 采样周期
    };
    // 初始位置、速度及其不确定性
    KF_2order_init(&kf, &config, 0.0f, 0.0f, 1.0f, 0.1f);
}

// 更新目标位置并获取预测值
void update_target(float measured_position) {
    // 更新滤波器
    float filtered_position = KF_2order_step(&kf, measured_position, 0.0f);
    
    // 获取估计的速度
    float velocity = KF_2order_get_velocity(&kf);
    
    // 使用滤波后的位置和速度进行控制
    // ...
}
```

## 3. 应用场景举例

### 3.1 陀螺仪零偏补偿

使用 EKF 可以在静态检测的基础上进行陀螺仪零偏估计和补偿，提高姿态估计精度。

### 3.2 传感器数据平滑

使用标量 KF 可以对噪声较大的传感器数据进行平滑处理：

```c
// 假设有一个距离传感器，数据带有噪声
float raw_distance = read_distance_sensor();
float filtered_distance = filter_sensor_data(raw_distance);

// 使用滤波后的数据进行决策
if (filtered_distance < THRESHOLD) {
    // ...
}
```

### 3.3 目标追踪

使用二阶 KF 可以实现运动目标位置和速度的平滑估计：

```c
// 通过视觉或其他传感器获取目标位置
float target_position = detect_target_position();

// 使用KF更新目标状态估计
update_target(target_position);

// 获取滤波后的位置和速度用于自动瞄准
float aim_position = kf.x[0] + lead_time * KF_2order_get_velocity(&kf);
```

## 4. 总结

卡尔曼滤波器模块为我们的系统提供了强大的状态估计和数据滤波能力。根据应用需求，可以选择：

- **EKF**：用于非线性系统的状态估计，如姿态估计
- **标量KF**：用于单变量系统的滤波，如传感器数据平滑
- **二阶KF**：用于位置-速度系统的状态估计，如目标追踪

合理应用这些工具，可以显著提高系统的稳定性和精确度。

