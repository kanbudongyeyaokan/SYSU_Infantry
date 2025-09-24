## 底层驱动（hardware层与hardware_driver层）：
**未完成**：                                            ***todo_list***:（集思广益一下！！！！）
            bsp_usb：USB串口                         1.math_lib:完善数学库
                                   
                                                    3.底盘的功率控制模型，这个要和当前接口适配
**待测试**：                                         4.将BMI088里面的SPI传输，接收全部改为DMA形式，减小通信时延
            bsp_dwt,bsp_dwg                         5.云台部分没有加上初始化电机对齐，就是上电后云台电机固定的动作
                                                    6.
**已完成**：bsp_can,bsp_usart,motor_driver,          7.
           bmi088_driver                            8.
                                                    9.
## 模块（Function_Module层）：                       10.
**未完成**：                                         11.
            功率控制                                 12.
            Gimbal                                  13.
            Ins
            Shoot
**待测试**：
            

**已完成**：Chassis,Remote_Control

## 任务（Application层）
**未完成**：
            Gimbal
            Ins
            Shoot
**待测试**：
        
    
**已完成**：Chassis

## 目前更新的优化环节（后续根据内容重要度和数量进行删减）
>1.可以使用以下接口切换电机的闭环控制类型，PID参数，用于解决不同模式下（比如底盘跟随云台和视觉模式）电机的PID自适应问题
>void Djimotor_change_controller(Djimotor_device_t *motor,Djimotor_controller_init_t ctrl_params);
>2.写了初步的数学库，放在第0层的MATH_LIB中