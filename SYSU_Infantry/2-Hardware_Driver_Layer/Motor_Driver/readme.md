
# 电机控制场景切换

在机器人控制系统中，对于同样的四个底盘电机，有一些不同的使用场景需要去更改不同的pid参数和控制模式去适应他们，比如
- 系统使用键盘和鼠标进行控制时，底盘电机有一套 pid 参数 和pid 控制模式（xx环）
- 系统使用遥控器进行控制时，底盘电机有一套 pid 参数 和pid 控制模式（xx环）
- 机器人进行小陀螺之类的机动动作时，底盘电机也有一套 pid 参数 和pid 控制模式（xx环）
- 机器人的底盘跟随云台时，底盘电机也有一套 pid 参数 和pid 控制模式（xx环）

请你分析当前我们的电机驱动库 dji_motor 和 alogorithm_pid 的代码结构，帮我修改成可以支持上述功能的代码结构，方便在决策层切换不同的模式时，自适应的去更改不同场景的底盘电机的 pid 参数 和pid 控制模式（xx环）

## 关键实现的现状

### 数据结构设计与优化分析

当前实现采用了场景管理机制，每个电机支持多个场景配置，每个场景有一套完整的PID参数和控制模式。对于数据结构设计和内存使用，有以下关键分析：

1. **当前数据结构**
   ```c
   typedef struct {
       Djimotor_closeloop_e close_loop;  // 电机控制模式
       Pid_init_t current_pid;           // 电流PID参数
       Pid_init_t angle_pid;             // 角度PID参数
       Pid_init_t speed_pid;             // 速度PID参数
   } Djimotor_scene_config_t;
   ```

2. **内存占用分析**
   - 每个电机有5个场景配置
   - 每个场景有3个PID参数结构体
   - 对于4个底盘电机，总共需要4×5×3=60个`Pid_init_t`实例
   - 每个`Pid_init_t`结构体包含约9个float变量（36字节），总共约2.16KB

3. **使用union优化潜力**
   不同控制模式下，并非所有PID参数都会被使用。例如：
   - 开环控制(OPEN_LOOP)：不使用任何PID
   - 速度环(SPEED_LOOP)：只使用速度PID
   - 电流环(CURRENT_LOOP)：只使用电流PID
   - 角度环(ANGLE_LOOP)：只使用角度PID
   - 复合环路：才会使用多个PID

   使用union可以共享内存，根据控制模式只保留需要的PID参数：

   ```c
   typedef struct {
       Djimotor_closeloop_e close_loop;  // 电机控制模式
       union {
           // 单环控制
           struct { Pid_init_t speed_pid; } speed_only;     
           struct { Pid_init_t current_pid; } current_only; 
           struct { Pid_init_t angle_pid; } angle_only;     
           
           // 双环控制
           struct {
               Pid_init_t speed_pid;
               Pid_init_t current_pid;
           } speed_current;
           
           struct {
               Pid_init_t angle_pid;
               Pid_init_t speed_pid;
           } angle_speed;
           
           // 完整参数 (备用)
           struct {
               Pid_init_t current_pid;
               Pid_init_t angle_pid;
               Pid_init_t speed_pid;
           } all_pid;
       } pid_params;
   } Optimized_scene_config_t;
   ```

4. **内存优化分析**
   - 理论最佳情况：内存占用可减少约1/3（只在使用单环控制时）
   - 最坏情况：无内存节省（所有场景都使用三环控制）
   - 实际优化：按我们的使用场景，约可节省25-30%内存

5. **优化的成本与收益**
   - **收益**：节省约0.5-0.7KB内存
   - **成本**：
     - 代码复杂度增加
     - 访问PID参数需要根据控制模式选择不同union成员
     - 场景切换逻辑更复杂
     - 维护难度增加

### 结论与建议

虽然使用union可以节省一定内存，但在当前应用场景下（只有4个电机、5个场景），节省的内存总量不足1KB，而代码复杂度会显著增加。对于STM32F4系列这样有足够RAM的平台，这种优化可能是"过早优化"。

建议在当前阶段保持简单清晰的数据结构，只有在内存确实紧张或者电机和场景数量大幅增加时，再考虑使用union进行优化。


## 如何在RTOS任务中切换电机控制场景

以下是在RTOS任务中切换电机控制场景的标准流程与数据链路分析：

### 1. 任务中切换场景示例代码

```c
/* 底盘控制任务示例 */
void chassis_control_task(void *pvParameters)
{
    // 任务初始化
    Djimotor_device_t *chassis_motors[4]; // 假设已经初始化了4个底盘电机
    
    // 获取电机实例（示例）
    chassis_motors[0] = get_motor_instance("chassis_fr");
    chassis_motors[1] = get_motor_instance("chassis_fl");
    chassis_motors[2] = get_motor_instance("chassis_bl");
    chassis_motors[3] = get_motor_instance("chassis_br");
    
    // 任务主循环
    while (1)
    {
        // 1. 检测控制模式（从决策层或其他任务获取）
        control_mode_e current_mode = get_current_control_mode(); // 假设函数
        
        // 2. 根据控制模式选择合适的电机场景
        Djimotor_scene_e motor_scene;
        switch (current_mode)
        {
            case KEYBOARD_MOUSE_MODE:
                motor_scene = SCENE_KEYBOARD_MOUSE;
                break;
                
            case REMOTE_CONTROL_MODE:
                motor_scene = SCENE_REMOTE_CONTROL;
                break;
                
            case SPINNING_TOP_MODE:
                motor_scene = SCENE_SPINNING_TOP;
                break;
                
            case FOLLOW_GIMBAL_MODE:
                motor_scene = SCENE_FOLLOW_GIMBAL;
                break;
                
            default:
                motor_scene = SCENE_DEFAULT;
                break;
        }
        
        // 3. 为所有底盘电机切换场景
        for (int i = 0; i < 4; i++)
        {
            if (Djimotor_get_current_scene(chassis_motors[i]) != motor_scene)
            {
                Djimotor_switch_scene(chassis_motors[i], motor_scene);
            }
        }
        
        // 4. 计算目标速度/角度等（根据控制模式不同而不同）
        float target_values[4] = {0};
        calculate_motor_targets(target_values, current_mode); // 假设函数
        
        // 5. 更新电机目标值
        for (int i = 0; i < 4; i++)
        {
            Djimotor_set_target(chassis_motors[i], target_values[i]);
        }
        
        // 6. 发送电机控制命令
        Djimotor_control_all(); // 这会让所有电机使用新的PID参数和控制模式
        
        // 7. 任务延时
        osDelay(2); // 假设2ms控制周期
    }
}
```

### 2. 场景切换数据链路分析

当调用`Djimotor_switch_scene(motor, scene)`时，数据链路如下：

1. **触发点：决策层控制模式变更**
   - 决策层确定新的控制模式（如键盘鼠标控制、遥控器控制等）
   - 通过共享变量或消息队列传递给底盘控制任务

2. **底盘控制任务检测控制模式变更**
   - 任务循环中检测到控制模式变更
   - 确定对应的电机场景（SCENE_KEYBOARD_MOUSE、SCENE_REMOTE_CONTROL等）

3. **调用场景切换函数**
   - `Djimotor_switch_scene(motor, scene)`被调用
   - 函数首先验证电机指针和场景索引是否有效
   - 检查是否已经是目标场景，若是则直接返回
   - 记录新的场景索引：`motor->motor_pid.current_scene = scene`

4. **获取场景PID配置**
   - 从电机的场景配置数组中获取目标场景的配置
   - `scene_config = &motor->motor_pid.scene_configs[scene]`

5. **更新电机控制模式**
   - 将场景配置中的控制模式应用到电机控制器
   - `motor->motor_pid.close_loop = scene_config->close_loop`
   - 这决定了后续在`Calculate_Motor_Output`函数中使用的控制策略（如角度环+速度环、速度环+电流环等）

6. **更新PID参数**
   - 调用`Pid_init`函数更新三个PID环的参数和状态：
     - `Pid_init(&motor->motor_pid.current_pid, &scene_config->current_pid)`
     - `Pid_init(&motor->motor_pid.speed_pid, &scene_config->speed_pid)`
     - `Pid_init(&motor->motor_pid.angle_pid, &scene_config->angle_pid)`
   
7. **PID参数重置过程**
   - `Pid_init`函数内部执行：
     - `memset(pid, 0, sizeof(Pid_instance_t))`：清零整个PID实例，包括累积的积分项、上次误差等
     - `memcpy(pid, config, sizeof(Pid_init_t))`：将新的PID参数（Kp、Ki、Kd等）复制到PID实例
     - `DWT_GetDeltaT(&pid->dwt_counter)`：初始化DWT计数器用于计算控制周期

8. **参数生效时机**
   - 场景切换后，新的控制模式和PID参数在**下一个控制周期**生效
   - 当调用`Djimotor_control_all()`时，`Calculate_Motor_Output`函数会使用新的控制模式和PID参数
   - 根据控制模式的不同，`Calculate_Motor_Output`会执行不同的控制策略（角度环、速度环、电流环或组合）

9. **控制输出计算**
   - 在控制策略中，`Pid_calculate`函数会使用新的PID参数计算控制输出
   - 由于PID状态已经被重置，控制输出可能会出现暂时的不连续

10. **电机实际动作**
    - 计算得到的控制输出通过CAN总线发送到电机
    - 电机执行相应的动作（速度变化、位置调整等）

### 3. 注意事项与优化建议

1. **场景切换时的平滑性**
   - 当前实现会完全重置PID状态，可能导致控制不连续
   - 在高速运动或精确控制场景下，可能需要考虑平滑过渡策略

2. **场景切换频率**
   - 避免过于频繁地切换场景，这可能导致控制不稳定
   - 可以添加切换冷却时间或滞后机制

3. **场景参数调优**
   - 各场景的PID参数需要单独调优以获得最佳性能
   - 可以考虑使用自适应参数或在线调整功能

4. **内存优化**
   - 如果内存受限，可以考虑使用更紧凑的数据结构或动态分配场景配置

通过这种场景管理机制，机器人可以在不同的操作模式下（键盘鼠标控制、遥控器控制、小陀螺模式、底盘跟随云台等）展现出最佳的控制性能，提高整体的响应速度和稳定性。