# Bmi088_Driver 文档

## 代办

需要更新 HAL_Delay 函数为 dwt 延时函数

## 概述

本模块在原有BMI088驱动基础上集成了扩展卡尔曼滤波器(EKF)，用于实现高精度姿态估计。通过融合加速度计和陀螺仪数据，能够输出稳定的四元数和欧拉角姿态信息。

## 主要特性

1. **多实例支持**: 支持多个BMI088设备同时运行EKF
2. **四元数估计**: 提供无奇点的姿态表示
3. **欧拉角输出**: 直观的Roll、Pitch、Yaw角度输出（单位：度）
4. **零偏校准**: 自动检测静态状态并校准陀螺仪零偏
5. **可配置参数**: 支持调整过程噪声和测量噪声参数
6. **实时更新**: 适配STM32环境，优化资源占用

## 数据结构

### EKF配置参数
```c
typedef struct {
    float process_noise_q;      // 过程噪声协方差 (推荐: 0.01)
    float measurement_noise_r;  // 测量噪声协方差 (推荐: 0.1)
    float gyro_bias_noise;      // 陀螺仪零偏噪声 (推荐: 0.001)
    float dt;                   // 采样周期(s) (推荐: 0.001)
    bool enable_bias_correction;// 启用零偏校正
    float static_threshold;     // 静态检测阈值 (推荐: 0.2)
} Ekf_config_t;
```

### 姿态输出结构
```c
// 四元数表示
typedef struct {
    float q0; // 实部
    float q1; // 虚部i
    float q2; // 虚部j  
    float q3; // 虚部k
} Quaternion_t;

// 欧拉角表示（单位：度）
typedef struct {
    float roll;  // 横滚角
    float pitch; // 俯仰角
    float yaw;   // 偏航角
} Euler_angles_t;
```

## 使用方法

### 1. 初始化BMI088设备
```c
// 创建BMI088配置
Bmi088_config_t bmi088_config = {
    .spi_handle = &hspi1,
    .accel_cs_gpio_port = GPIOA,
    .accel_cs_gpio_pin = GPIO_PIN_4,
    .gyro_cs_gpio_port = GPIOB,
    .gyro_cs_gpio_pin = GPIO_PIN_0,
    .enable_accel_self_test = true,
    .enable_gyro_self_test = true
};

// 初始化BMI088设备
Bmi088_device_t* bmi088 = Bmi088_device_init(&bmi088_config);
```

### 2. 初始化EKF
```c
// 创建EKF配置
Ekf_config_t ekf_config = {
    .process_noise_q = 0.01f,
    .measurement_noise_r = 0.1f,
    .gyro_bias_noise = 0.001f,
    .dt = 0.001f, // 1ms采样周期
    .enable_bias_correction = true,
    .static_threshold = 0.2f
};

// 初始化EKF
Bmi088_error_e error = Bmi088_ekf_init(bmi088, &ekf_config);
if (error != NO_ERROR) {
    printf("EKF初始化失败\n");
}
```

### 3. 实时姿态估计
```c
// 在主循环中以固定频率调用
void attitude_estimation_loop() {
    // 更新EKF（自动读取传感器数据并估计姿态）
    Bmi088_error_e error = Bmi088_ekf_update(bmi088);
    
    if (error == NO_ERROR) {
        // 获取四元数
        Quaternion_t* quaternion = Bmi088_get_quaternion(bmi088);
        
        // 获取欧拉角
        Euler_angles_t* euler = Bmi088_get_euler_angles(bmi088);
        
        // 使用姿态数据
        printf("姿态: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
               euler->roll, euler->pitch, euler->yaw);
    }
}
```

## API接口说明

### Bmi088_ekf_init()
- **功能**: 初始化EKF参数和状态
- **参数**: 
  - `bmi088`: BMI088设备指针
  - `ekf_config`: EKF配置参数指针
- **返回**: 错误代码

### Bmi088_ekf_update()
- **功能**: 更新EKF状态估计
- **参数**: `bmi088`: BMI088设备指针
- **返回**: 错误代码
- **说明**: 自动读取传感器数据并执行预测和更新步骤

### Bmi088_get_quaternion()
- **功能**: 获取当前姿态四元数
- **参数**: `bmi088`: BMI088设备指针
- **返回**: 四元数指针（指向内部存储）

### Bmi088_get_euler_angles()
- **功能**: 获取当前欧拉角
- **参数**: `bmi088`: BMI088设备指针
- **返回**: 欧拉角指针（指向内部存储）

## 算法原理

### EKF状态向量
状态向量包含7个元素：
- q0, q1, q2, q3: 姿态四元数
- bx, by, bz: 陀螺仪零偏

### 预测步骤
使用陀螺仪角速度积分更新四元数：
```
q_k+1 = q_k + 0.5 * dt * Ω * q_k
```
其中Ω是角速度反对称矩阵。

### 更新步骤
使用归一化的加速度计数据作为重力方向观测，校正姿态估计。

### 零偏校准
当检测到静态状态时（加速度接近重力值且角速度很小），自动校准陀螺仪零偏。

## 参数调优建议

1. **process_noise_q**: 控制对陀螺仪的信任度
   - 较大值: 更信任加速度计，姿态变化更平滑
   - 较小值: 更信任陀螺仪，响应更快速

2. **measurement_noise_r**: 控制对加速度计的信任度
   - 较大值: 更信任陀螺仪，减少加速度计噪声影响
   - 较小值: 更信任加速度计，姿态校正更强

3. **采样频率**: 推荐100Hz-1000Hz
   - 频率越高，估计精度越好，但计算负荷增加

## 性能特点

- **收敛时间**: 约1-2秒
- **姿态精度**: Roll/Pitch < 1°, Yaw取决于初始值
- **计算负荷**: 每次更新约0.1ms@72MHz STM32
- **内存占用**: 每实例约1KB RAM

## 注意事项

1. 确保传感器正确校准
2. 避免强烈的线性加速度干扰
3. 定期让设备处于静态状态以校准零偏
4. Yaw角需要外部磁力计或GPS辅助才能获得绝对方向

## 测试验证

参考 `bmi088_test_task.c` 中的完整测试示例，包含：
- 传感器初始化验证
- EKF参数配置
- 实时姿态输出
- 零偏校准状态监控