电机控制说明：
1.每一个电机使用一个结构体涵括，结构体如下：
（目的：该结构体描绘了DJI电机的一切信息，包含了电机的控制状态以及控制参数）
/**DJI电机实例**/
#pragma pack(1)
typedef struct
{
    char motor_name[16];                //电机名
    Djimotor_type_e motor_type;         //电机类型
    Djimotor_status_e motor_status;     //电机运动状态
    Djimotor_measure_t motor_measure;   //电机自身运动信息
    Djimotor_controller_t   motor_pid;  //电机自身的PID控制器
    Can_controller_t can_controller;    //电机自身的CAN管理者
}Djimotor_device_t;
#pragma pack()

解释：
1.【motor_name】其中每一个电机有一个名字motor_name（可选），这个可有可无，目的是明确该电机属于哪一个部分，但似乎可以通过变量名来表示。
2.【motor_type】电机类型，需要选择该电机是大疆电机的哪一个款式，因为G3508,M2006,GM6020电机它们的CAN发送ID是不一样的，接收ID也是不一样的，本意是根据不同电机类型，需要进行不同的处理。
3.【motor_status】电机运动状态，里面只有ENABLED和STOPPED,目的是如果手动输入电机状态或者用一个函数来实现，就可以控制电机它能否运动。
4.【motor_measure】电机自身运动信息，存储电机CAN回传的数据，解析出来的电机自身转速，编码器值等数据
5.【motor_pid】电机自身的PID控制器，该结构体包含了电机的控制模式，电机的PID控制器一切参数
6.【can_controller】电机自身的CAN管理者，这个非常重要，包含了电机CAN的发送/接收ID， 发送/接收缓冲区，在调用电机控制函数的时候，需要用到里面的信息，然后通过CAN发送出去

test
