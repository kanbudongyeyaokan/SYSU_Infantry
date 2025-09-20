
- 我似乎没在这个任务看到你启用 chassis task和motor task，我希望的是你使用这个两个任务进行串口测试
- 我想要你的测试代码能够同时启动这两个任务，并且在程序中模拟系统中的决策层。发布对应的话题去验证 chassis_task 和 motor_task 之间的数据链路


我的决策层控制接口是
```c
void Decision_making_task_init()
{
    //接收遥控器数据
    rc_data = RC_Data_Get(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    /***********************************初始化决策层的发布者和订阅者***************************************/
    //底盘
    chassis_cmd_pub = Pub_register("chassis_cmd",sizeof(Chassis_cmd_send_t));
    chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));//底盘反馈数据订阅者
    //云台
    gimbal_cmd_pub = Pub_register("gimbal_cmd", sizeof(Gimbal_cmd_send_t));//云台注册的话题是gimbal_cmd
    gimbal_feedback_sub = Sub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));
    //发射机构
    shoot_cmd_pub = Pub_register("shoot_cmd", sizeof(Shoot_cmd_send_t));
    shoot_feedback_sub = Sub_register("shoot_feedback", sizeof(Shoot_feedback_info_t));

    //机器人开始工作
    robot_state = ROBOT_ON;

}
```

我的 chassis task 的控制接口是
```c
static void Chassis_task_init(void)
{
    // 初始化底盘功能模块
    Chassis_params_t chassis_params = {
        .wheel_radius = 0.076f,         // 轮子半径76mm
        .chassis_radius = 0.2f,         // 底盘半径200mm  
        .wheel_base = 0.4f,             // 轮距400mm
        .track_width = 0.3f,            // 轮宽300mm
        .chassis_type = CHASSIS_TYPE_OMNI // 全向轮底盘
    };
    Chassis_init(&chassis_params);
    
    // 初始化底盘电机
    Chassis_motors_init();
    
    // 订阅决策层发来的底盘控制指令
    chassis_cmd_sub = Sub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
    
    // 注册底盘反馈信息发布者
    chassis_feedback_pub = Pub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));
}

我需要在底盘和电机的测试任务里