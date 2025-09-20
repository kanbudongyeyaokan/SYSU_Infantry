motor_task 这个任务不需要接收从CHASSIS_TASK发送的target值，只需要调用DJI_motor_control_all就可以了，因为我的所有电机的target更新都放在djimotor.c里面的静态缓冲区了，你可以看下这个文件
```c
// dji_motor.c
/**对应不同电机发送CAN-ID的8字节数组,4个电机的共享发送缓冲区**/
static uint8_t djimotor_can1_0x1ff_tx[8]={0};
static uint8_t djimotor_can1_0x200_tx[8]={0};
static uint8_t djimotor_can1_0x2ff_tx[8]={0};
static uint8_t djimotor_can2_0x1ff_tx[8]={0};
static uint8_t djimotor_can2_0x200_tx[8]={0};
static uint8_t djimotor_can2_0x2ff_tx[8]={0};
```
所以motor task 中的
```c
        // 处理底盘电机控制（从Chassis Task获取目标值）
        Motor_control_handle_chassis_motors();
        
        // 处理云台控制指令（从决策层获取）
        Motor_control_handle_gimbal_cmd();
        
        // 处理发射机构控制指令（从决策层获取）
        Motor_control_handle_shoot_cmd();
```
是不需要的，你可以直接删掉，然后在motor task的循环里面直接调用 DJI_motor_control_all() 就行了


然后这里的电机初始化都是放在各自的应用层的初始化里面做的，Function_Module层不用加这个MOTOR_CONTROL,
比如GIMBAL电机就在gimbal.c里面初始化这样

所以原来的 motor_control 模块除去电机初始化的功能之后可以考虑合并到 motor_task 里面去

这个motor_task就是一个发送CAN数据的后台任务，数据都在djimotor.c的静态缓冲区实时更新的，然后各个应用层的电机实际上只需要修改他们的target值就OK，CAN发送都放在motor_task