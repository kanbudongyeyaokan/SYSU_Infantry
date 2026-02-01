使用方法
场景 A：调试 PID，看 3 个变量
你只需要像下面这样写，不用管格式：

C
#include "vofa.h" // 引入头文件

void Gimbal_control_task(void const *argument) {
// ... 初始化 ...

    // 用于分频打印
    uint8_t debug_cnt = 0;

    for (;;) {
        xQueueReceive(Gimbal_cmd_queue_handle, &cmd_recv, 0);
        Gimbal_handle_command(&cmd_recv);

        // 每 10ms 或 20ms 打印一次，防止堵塞串口
        if (++debug_cnt >= 20) {
            // [直接调用] 自动识别这是3个变量
            // 不管 yaw 是 float 还是 int，都会转成 float 发送
            VOFA_Send(test_uart, 
                      cmd_recv.yaw,                      // 目标值
                      yaw_motor->motor_measure.total_angle, // 实际值
                      yaw_motor->out_current);           // PID输出
                      
            debug_cnt = 0;
        }

        vTaskDelayUntil(&PreviousWakeTime, 1);
    }
}
发送出去的数据格式： 10.50,10.45,1500.00\n VOFA+ 显示： 会自动生成 3 条曲线，通道名为 0, 1, 2。

场景 B：我想同时看 Yaw 和 Pitch，一共 5 个变量
直接加参数即可：

C
VOFA_Send(test_uart,
cmd_recv.yaw,        // Yaw目标
yaw_motor->angle,    // Yaw实际
cmd_recv.pitch,      // Pitch目标
pitch_motor->angle,  // Pitch实际
pitch_motor->out);   // Pitch电流
发送出去的数据格式： 10.5,10.4,5.0,5.1,1200\n

场景 C：我有多个波形分组，想加个名字区分
如果你想利用 FireWater 的 image:1.1,2.2\n 这种标签功能：

C
// 发送格式： "yaw_pid:10.50,10.45,1500.00\n"
VOFA_Send_Label(test_uart, "yaw_pid", cmd_recv.yaw, yaw_real, yaw_out);