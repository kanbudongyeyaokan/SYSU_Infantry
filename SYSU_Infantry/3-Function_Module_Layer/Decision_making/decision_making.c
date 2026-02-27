/**
 * @file    decision_making.c
 * @brief   决策任务
 * @author  SYSU电控组
 * @date    2025-09-17
 * @version 1.0
 * 
 * @note    决策控制模式，控制量，并传送到对应的任务
 */
#include "decision_making.h"
//通信
#include "message_center.h"
//状态机获取
#include "robot_definitions.h"
//遥控器控制
#include <stdio.h>
#include "remote_control.h"
#include "main.h"
#include <stdbool.h>
#include <math.h>
#include "SBUS.h"
#include "gimbal.h"  // 用于获取云台状态
#include "robot_task.h"
#include "bsp_usart.h"
#include "video_link.h"
/**********************发出决策信息***************************/
//存储遥控器数据，CURRENT-当前数据,LAST-上一次数据
#if USE_SBUS_RECEIVER == 1
static SBUS_ctrl_t *sbus_data; //SBUS遥控器数据
#elif USE_SBUS_RECEIVER == 2
static Video_RC_ctrl_t *vrc_data;  //图传链路数据
#else
static RC_ctrl_t *rc_data;      //DJI遥控器数据
#endif
//底盘控制模式/控制量发布
static Chassis_cmd_send_t chassis_cmd_send;//存储决策层给底盘应用层的控制信息

//云台控制模式/控制量发布
static Gimbal_cmd_send_t  gimbal_cmd_send;  //存储决策层给云台应用层的控制信息
static float gimbal_virtual_target = 0.0f;

//发射机构控制模式/控制量发布
static Shoot_cmd_send_t   shoot_cmd_send;   //存储决策层给发射机构应用层的控制信息

//机器人整体工作状态（二元）：----ON：在线 OFF：离线
static Robot_status_e robot_state = ROBOT_OFF;

// static bool rc_cailibrated = true; // 遥控器校准标志位
#define PITCH_UP_MAX 20.0f
#define PITCH_DOWN_MAX -40.0f
#define KEY_SENSITIVITY 0.5f // 键盘控制灵敏度，数值越大响应越快，但可能不够平滑，建议从0.1开始调试
#define KEYCTL__SPEED 3000.0f // 键盘控制的最大速度，单位可以根据实际情况调整
/************************************************************/

/**********************接收反馈信息***************************/
//底盘反馈数据读取
static Chassis_feedback_info_t chassis_feedback_recv;   //存储底盘应用层发给决策层的信息

//云台反馈数据读取

extern QueueHandle_t Shoot_feedback_queue_handle;   //发射机构控制信息队列句柄
extern QueueHandle_t Chassis_feedback_queue_handle; // 声明外部底盘命令队列句柄
extern QueueHandle_t Gimbal_feedback_queue_handle; // 新增：声明外部队列句柄
static Gimbal_feedback_info_t  gimbal_feedback_recv;    //存储云台应用层发给决策层的信息
static bool gimbal_yaw_initialized = false;


//发射机构反馈数据读取
static Subscriber_t *shoot_feedback_sub;                 //发射反馈信息订阅者
static Shoot_feedback_info_t   shoot_feedback_recv;     //存储发射应用层发给决策层的信息
/************************************************************/

// 定义灵敏度系数
// 之前是 0.0018 (200Hz)，现在是 1000Hz，理论上应该除以 5
// 建议改小到 0.0003 ~ 0.0005 之间，手感会比较细腻
#define GIMBAL_RC_MOVE_RATIO_YAW   0.0002f
#define GIMBAL_RC_MOVE_RATIO_PITCH 0.0005f
// 定义死区大小 (根据你的遥控器老化程度，建议设大一点，比如 10 到 20)
#define RC_DEADBAND 5
static float PITCH_RC_CENTER_OFFSET = 0.0f;  // 摇杆中位偏移量，上电自动校准



/**
 * @brief 任务初始化函数，初始化决策层的发布者和订阅者,获取遥控器数据
 *
*/
void Decision_making_task_init()
{
    //接收遥控器数据
#if USE_SBUS_RECEIVER == 1
    sbus_data = SBUS_Data_Get(&huart3);
#elif USE_SBUS_RECEIVER == 2
    vrc_data = Video_RC_Data_Get(&huart1);  // 图传串口，按实际修改
#else
    rc_data = RC_Data_Get(&huart3);
    // 上电后等待0.5s，读取pitch摇杆原点值作为偏移量
    osDelay(500);
    PITCH_RC_CENTER_OFFSET = (float)rc_data[CURRENT].rc.Rrocker_y;
#endif

    /***********************************初始化决策层的发布者和订阅者***************************************/
    //底盘
    // chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));//底盘反馈数据订阅者
    //云台
    // gimbal_feedback_sub = Sub_register("gimbal_feedback", sizeof(Gimbal_feedback_info_t));
    
    //发射机构
    // shoot_feedback_sub = Sub_register("shoot_feedback", sizeof(Shoot_feedback_info_t));

    //机器人开始工作 - 关键！缺少此初始化会导致控制无响应
    robot_state = ROBOT_ON;

    // 初始化默认模式
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  // 默认使能云台控制
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;  // 默认不跟随模式
}

//获取各个模块的反馈信息
void Receive_feedback_infomation()
{
    //获取底盘反馈信息
    xQueuePeek(Chassis_cmd_queue_handle, &chassis_feedback_recv, 0);
    //获取云台反馈信息
    xQueuePeek(Gimbal_feedback_queue_handle, &gimbal_feedback_recv, 0);
    //获取发射机构反馈信息
    xQueuePeek(Shoot_cmd_queue_handle, &shoot_feedback_recv, 0);
}

void Send_command_to_all_task()
{
    //printf("HELLO\r\n");
    // Uart_printf(test_uart,"Sending command to all tasks\r\n");
    //发送底盘控制信息
    xQueueOverwrite(Chassis_cmd_queue_handle, &chassis_cmd_send);

    //发送云台控制信息
    xQueueOverwrite(Gimbal_cmd_queue_handle, &gimbal_cmd_send);

    //发送发射机构控制信息
    xQueueOverwrite(Shoot_cmd_queue_handle, &shoot_cmd_send);
}

/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command()
{
#if USE_SBUS_RECEIVER == 1 || USE_SBUS_RECEIVER == 2

    // SBUS遥控器调试输出
    // printf("SBUS Ch1:%d Ch2:%d Ch3:%d Ch4:%d S1:%d S2:%d\r\n",
    //        sbus_data[CURRENT].rc.Ch1, sbus_data[CURRENT].rc.Ch2,
    //        sbus_data[CURRENT].rc.Ch3, sbus_data[CURRENT].rc.Ch4,
    //        sbus_data[CURRENT].S1, sbus_data[CURRENT].S2);


    //图传链路遥控调试输出
    // 调试输出: 遥控器摇杆数据
    // Uart_printf(test_uart,"lx:%d,ly:%d,rx:%d,ry:%d,dial:%d\r\n",
    //     vrc_data[CURRENT].rc.Lrocker_x,  // 左摇杆X轴
    //     vrc_data[CURRENT].rc.Lrocker_y,// 左摇杆Y轴
    //     vrc_data[CURRENT].rc.Rrocker_x,  // 右摇杆X轴
    //     vrc_data[CURRENT].rc.Rrocker_y,  // 右摇杆Y轴
    //     vrc_data[CURRENT].rc.dial);       // 拨轮值    
    // 调试输出: 遥控器按键状态
    // Uart_printf(test_uart,"mode:%d,pause:%d,bl:%d,br:%d,trig:%d\r\n",
    //     vrc_data[CURRENT].rc.mode_switch, // 模式开关
    //     vrc_data[CURRENT].rc.pause,       // 暂停键
    //     vrc_data[CURRENT].rc.btn_left,    // 左侧按键
    //     vrc_data[CURRENT].rc.btn_right,   // 右侧按键
    //     vrc_data[CURRENT].rc.trigger);    // 扳机键
    // SBUS开关映射: S1左开关, S2右开关 (1=下, 2=上, 3=中)
    // if (sbus_data[CURRENT].S1 == SBUS_SWITCH_DOWN)
    // {
    //     RC_ctrl_set();
    // }
    // else if (sbus_data[CURRENT].S1 == SBUS_SWITCH_UP)
    // {
    //     Keyboard_ctrl_set();
    // }
    static bool ctrl_mode = 0; // 0=遥控器控制, 1=键鼠控制
    if (ctrl_mode == 0){
        //遥控器检测切换（自定义左键控制切换）
        if(vrc_data[CURRENT].rc.btn_left){

            ctrl_mode = 1;

            return;
        }   
        RC_ctrl_set();
    }else{
        //键鼠检测切换（ctrl键控制切换）
        if(vrc_data[CURRENT].keyboard & 0x0020){

            ctrl_mode = 0;

            return;
        }
        Keyboard_ctrl_set();
    } 
    // RC_ctrl_set();
#else
    // printf("rc_data[CURRENT].rc.Rrocker_x:%d\r\n",rc_data[CURRENT].rc.Rrocker_x);
    // printf("rc_data[CURRENT].rc.Rrocker_y:%d\r\n",rc_data[CURRENT].rc.Rrocker_y);
    // printf("rc_data[CURRENT].rc.Lrocker_x:%d\r\n",rc_data[CURRENT].rc.Lrocker_x);

    // Uart_printf(test_uart,"lx:%d,ly:%d,rx:%d,ry:%d\r\n",rc_data[CURRENT].rc.Lrocker_x,
    //     rc_data[CURRENT].rc.Lrocker_y,rc_data[CURRENT].rc.Rrocker_x,rc_data[CURRENT].rc.Rrocker_y);


    //左边开关打下，进入遥控器控制模式
    if (rc_data[CURRENT].rc.Lswitch == SWITCH_IS_DOWN)
    {
        RC_ctrl_set();
    }
    //左边开关打上，进入键鼠模式
    else if (rc_data[CURRENT].rc.Lswitch == SWITCH_IS_UP)
    {
        Keyboard_ctrl_set();
    }
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
*/
void RC_ctrl_set()
{
#if USE_SBUS_RECEIVER == 1
    /**根据SBUS Ch8三档拨杆设定模式**/
    // Ch8三档拨杆实际值: 下=-660, 中=0, 上=+660
    // 使用SBUS.h中定义的通用阈值宏
    
    if (sbus_data[CURRENT].rc.Ch8 < SBUS_3POS_THRESHOLD_DOWN) // Ch8 下 (约-660) → 跟随模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    else if (sbus_data[CURRENT].rc.Ch8 > SBUS_3POS_THRESHOLD_UP) // Ch8 上 (约+660) → 小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else // Ch8 中 (约0) → 不跟随模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  
    }
    
    /**射击模式设定 - 使用Ch15(左拨轮)**/
    // Ch15左拨轮范围: -660 ~ +660
    // 向上拨(正值)开启摩擦轮, 继续向上拨开始发射
    if (sbus_data[CURRENT].rc.Ch15 > 0)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;  // 开启摩擦轮
    }
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    
    if (sbus_data[CURRENT].rc.Ch15 > 600)
    {
        shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;  // 连发
        shoot_cmd_send.shoot_rate = 8;
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    
    //急停模式
    Emergency_stop();
    
    /****************控制量设定*****************/
    // SBUS通道映射: Ch2=前后, Ch4=左右, Ch1=YAW, Ch3=PITCH
    // 死区设置: 遥控器中心可能有偏移(约±50)，死区需要覆盖这个偏移
    #define SBUS_DEADZONE 60
    
    // 底盘控制量
    if (sbus_data[CURRENT].rc.Ch2 >= -SBUS_DEADZONE && sbus_data[CURRENT].rc.Ch2 <= SBUS_DEADZONE)
        chassis_cmd_send.vy = 0;
    else
        chassis_cmd_send.vy = 2.0f * (float)sbus_data[CURRENT].rc.Ch2;

    
    chassis_cmd_send.vx = -2.0f * (float)sbus_data[CURRENT].rc.Ch4;
    
    // ==================== 云台目标值同步逻辑 ====================
    // 当云台从归中状态切换到就绪状态时，需要同步目标值
    Gimbal_state_e gimbal_state = Gimbal_get_state();
    if (gimbal_state == GIMBAL_STATE_READY && !gimbal_yaw_initialized) {
        // 首次进入 READY 状态，同步目标值为当前 IMU 读数
        gimbal_cmd_send.yaw = gimbal_feedback_recv.imu_yaw_total_angle;
        gimbal_cmd_send.pitch = 0;  // Pitch 从 0 开始
        gimbal_yaw_initialized = true;
    }
    
    // 云台控制量（只有在就绪状态才累加）
    if (gimbal_state == GIMBAL_STATE_READY) {
        if (sbus_data[CURRENT].rc.Ch1 > SBUS_DEADZONE || sbus_data[CURRENT].rc.Ch1 < -SBUS_DEADZONE)
            gimbal_cmd_send.yaw += 0.0018f * (float)sbus_data[CURRENT].rc.Ch1;
        
        if (sbus_data[CURRENT].rc.Ch3 > SBUS_DEADZONE || sbus_data[CURRENT].rc.Ch3 < -SBUS_DEADZONE)
            gimbal_cmd_send.pitch -= 0.0018f * (float)sbus_data[CURRENT].rc.Ch3;
    }
    
    // Pitch限幅
    if (gimbal_cmd_send.pitch > 50)
        gimbal_cmd_send.pitch = 50;
    else if (gimbal_cmd_send.pitch < -50)
        gimbal_cmd_send.pitch = -50;
#elif USE_SBUS_RECEIVER == 2
    /**根据图传遥控 mode_switch 设定模式 (C=0,N=1,S=2)**/
    if (vrc_data[CURRENT].rc.mode_switch == 0)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (vrc_data[CURRENT].rc.mode_switch == 2)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    shoot_cmd_send.shoot_mode = vrc_data[CURRENT].rc.trigger ? SHOOT_ON : SHOOT_OFF;
    shoot_cmd_send.loader_mode = vrc_data[CURRENT].rc.trigger ? LOAD_1_BULLET : LOAD_STOP;

    Emergency_stop();

    if (fabsf((float)vrc_data[CURRENT].rc.Lrocker_y) > RC_DEADBAND)
        chassis_cmd_send.vy = -2.0f * (float)vrc_data[CURRENT].rc.Lrocker_y;
    else
        chassis_cmd_send.vy = 0;
    chassis_cmd_send.vx = -2.0f * (float)vrc_data[CURRENT].rc.Lrocker_x;

    if (fabsf((float)vrc_data[CURRENT].rc.Rrocker_x) > RC_DEADBAND)
        gimbal_cmd_send.yaw -= GIMBAL_RC_MOVE_RATIO_YAW * (float)vrc_data[CURRENT].rc.Rrocker_x;



    if (fabsf((float)vrc_data[CURRENT].rc.Rrocker_y) > RC_DEADBAND)
        gimbal_cmd_send.pitch += GIMBAL_RC_MOVE_RATIO_PITCH * (float)vrc_data[CURRENT].rc.Rrocker_y;
    if (gimbal_cmd_send.pitch > PITCH_UP_MAX) gimbal_cmd_send.pitch = PITCH_UP_MAX;
    else if (gimbal_cmd_send.pitch < PITCH_DOWN_MAX) gimbal_cmd_send.pitch = PITCH_DOWN_MAX;
    
    
    // Uart_printf(test_uart, "Vx:%.2f,Vy:%.2f,Wz:%.2f,offset,chassis:%d\r\n",
    // chassis_cmd_send.vx, 
    // chassis_cmd_send.vy, 
    // chassis_cmd_send.wz,

    // chassis_cmd_send.chassis_mode);

#else
    /**根据遥控器开关状态设定模式**/
    /**底盘/云台模式设定**/
    //如果右边开关打下，则进入底盘跟随云台模式,云台进入陀螺仪反馈模式
    // printf("vx: %f,vy:%f\r\n", chassis_cmd_send.vx,chassis_cmd_send.vy);
    if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_DOWN)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  
    }
    //如果右边开关打中间，则底盘进入小陀螺模式，云台进入自由模式
    else if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_UP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮,进入准备射击模式
    if (rc_data[CURRENT].rc.dial > 150)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    //正常情况下不打开摩擦轮
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
     //左边拨轮往上打到底，开始发射子弹
 
     if (rc_data[CURRENT].rc.dial > 450)
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;//连发
            shoot_cmd_send.shoot_rate = 8;//每分钟80发
        }
        //正常情况下不发射子弹
        else
        {
            shoot_cmd_send.loader_mode = LOAD_STOP;     //
        }
        
    //急停模式
    Emergency_stop();


    /****************控制量设定*****************/
    //底盘控制量
     /*后续可增加死区限制，解决遥控器通道值因老化而造成的零漂问题*/
     if (rc_data[CURRENT].rc.Lrocker_y>=-32&&rc_data[CURRENT].rc.Lrocker_y<=0)
         chassis_cmd_send.vy=0;
     else {
         chassis_cmd_send.vy = 2.0f * (float)rc_data[CURRENT].rc.Lrocker_y; //竖直方向
     }
     chassis_cmd_send.vx = 2.0f * (float)rc_data[CURRENT].rc.Lrocker_x; //水平方向

    // //云台控制量
    // 1. 获取原始数据
    float yaw_input = (float)rc_data[CURRENT].rc.Rrocker_x;
    float pitch_input = (float)rc_data[CURRENT].rc.Rrocker_y - PITCH_RC_CENTER_OFFSET; // 减去实测的中心偏移量;

    // 2. YAW 轴处理 (死区 + 降速)
    if (fabsf(yaw_input) > RC_DEADBAND)
    {
        // 只有超过死区才累加
        gimbal_cmd_send.yaw -= GIMBAL_RC_MOVE_RATIO_YAW * yaw_input;
    }

    // 3. PITCH 轴处理 (死区 + 累加: 初始0, 上拨+, 回中保持)
       if (fabsf(pitch_input) > RC_DEADBAND)
    {
        gimbal_cmd_send.pitch += GIMBAL_RC_MOVE_RATIO_PITCH * pitch_input;
    }
    
    // Uart_printf(test_uart, "yaw_input:%.2f,pitch_input:%.2f\r\n", yaw_input, pitch_input); 
    // 4. 限幅保持不变
    if (gimbal_cmd_send.pitch > 20)
        gimbal_cmd_send.pitch = 20;
    else if (gimbal_cmd_send.pitch < -40)
        gimbal_cmd_send.pitch = -40;

#endif
}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
void Keyboard_ctrl_set()
{
#if USE_SBUS_RECEIVER == 1
    // Ch8三档拨杆: 下=-660, 中=0, 上=+660
    if (sbus_data[CURRENT].rc.Ch8 < SBUS_3POS_THRESHOLD_DOWN) // Ch8 下 → 跟随模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
    }
    else if (sbus_data[CURRENT].rc.Ch8 > SBUS_3POS_THRESHOLD_UP) // Ch8 上 → 小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else // Ch8 中 → 跟随模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  
    }
    
    /**射击模式设定 - Ch15左拨轮**/
    if (sbus_data[CURRENT].rc.Ch15 > 150)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    
    if (sbus_data[CURRENT].rc.Ch15 > 300)
    {
        shoot_cmd_send.loader_mode = LOAD_1_BULLET;  // 单发
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    
    //急停模式
    Emergency_stop();
#elif USE_SBUS_RECEIVER == 2
    Key_t kb  = {.keys = vrc_data[CURRENT].keyboard};
    Key_t kbl = {.keys = vrc_data[LAST].keyboard};
    // Uart_printf(test_uart, "kb: w%d s%d a%d d%d shift%d ctrl%d q%d e%d r%d f%d g%d z%d x%d c%d v%d b%d\r\n",
    //     kb.w, kb.s, kb.a, kb.d, kb.shift, kb.ctrl,
    //     kb.q, kb.e, kb.r, kb.f, kb.g,
    //     kb.z, kb.x, kb.c, kb.v, kb.b);
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;//设置底盘的控制模式为跟随云台模式
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;//设置云台的控制模式为陀螺仪模式。
    Emergency_stop();//调用急停函数

    //三种底盘模式，用于 C 键循环切换
    static const chassis_mode_e vrc_modes[] = {CHASSIS_FOLLOW_GIMBAL, CHASSIS_NO_FOLLOW, CHASSIS_ROTATE};
    
    static uint8_t 
    vrc_mode_idx = 0,   //底盘模式索引
    vrc_friction = 0,   //摩擦轮开关状态（0=关/1=开）
    vrc_burst = 0;      //射击模式

    if (kb.c && !kbl.c) vrc_mode_idx = (vrc_mode_idx + 1) % 3;//C键边沿检测 ，底盘模式循环切换
    if (kb.f && !kbl.f) vrc_friction ^= 1;//F键边沿检测 ，摩擦轮状态翻转
    if (kb.e && !kbl.e) vrc_burst ^= 1;//E键边沿检测 ，射击模式切换翻转
    
    chassis_cmd_send.chassis_mode = vrc_modes[vrc_mode_idx];//取出对应的底盘模式并赋值
    chassis_cmd_send.vx = kb.w ? KEYCTL__SPEED: kb.s ? -KEYCTL__SPEED: 0;//设置底盘前后速度 W按下 Vx = 3000 S按下Vx = -3000
    chassis_cmd_send.vy = kb.d ? KEYCTL__SPEED : kb.a ? -KEYCTL__SPEED: 0;//设置底盘左右速度
/***
 * 鼠标水平移动量累加到云台 Yaw 目标角度
 * mouse.x — 鼠标本帧水平位移（向右为正）
 * 0.005f — 灵敏度系数，控制转动速率
 * -= — 鼠标右移时Yaw 目标值减小（云台向右转）
***/

    gimbal_cmd_send.yaw   -= KEY_SENSITIVITY * vrc_data[CURRENT].mouse.x;
    gimbal_cmd_send.pitch += KEY_SENSITIVITY * vrc_data[CURRENT].mouse.y;

 /***
  * 对Pitch 目标角度做限幅
  * -超过 30° → 强制钳位到 30°（最大仰角）
  * 低于 -30° → 强制钳位到 -30°（最大俯角
****/
    if (gimbal_cmd_send.pitch > 30)       gimbal_cmd_send.pitch = 30;
    else if (gimbal_cmd_send.pitch < -30) gimbal_cmd_send.pitch = -30;


    /***
     * 根据摩擦轮状态变量设置发射模式,由 F 键 toggle 控制
     * vrc_friction == 1 → SHOOT_ON（开启摩擦轮）
     * vrc_friction == 0 → SHOOT_OFF（关闭摩擦轮)
     ***/
    shoot_cmd_send.shoot_mode  = vrc_friction ? SHOOT_ON : SHOOT_OFF;

    /*
    *根据鼠标左键状态设置子弹发射模式
    *mouse.press_l == 1（左键按下）→根据 vrc_burst 决定单发还是连发
    *mouse.press_l == 0（左键未按）→ LOAD_STOP（停止发射）
    */
    shoot_cmd_send.loader_mode = vrc_data[CURRENT].mouse.press_l ?
        (vrc_burst ? LOAD_BURSTFIRE : LOAD_1_BULLET) : LOAD_STOP;
    if (vrc_burst) shoot_cmd_send.shoot_rate = 8;

    Uart_printf(test_uart, "Vx:%.2f,Vy:%.2f,Wz:%.2f,offset,chassis:%d\r\n",
    chassis_cmd_send.vx, 
    chassis_cmd_send.vy, 
    chassis_cmd_send.wz,

    chassis_cmd_send.chassis_mode);    
#else
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    Emergency_stop();

    // C: 切换底盘速度档位
    static const chassis_mode_e kb_modes[] = {CHASSIS_FOLLOW_GIMBAL, CHASSIS_NO_FOLLOW, CHASSIS_ROTATE};
    static uint8_t kb_mode_idx = 0, kb_friction = 0, kb_burst = 0;
    if (rc_data[CURRENT].keyboard.c && !rc_data[LAST].keyboard.c) kb_mode_idx = (kb_mode_idx + 1) % 3;
    if (rc_data[CURRENT].keyboard.f && !rc_data[LAST].keyboard.f) kb_friction ^= 1;
    if (rc_data[CURRENT].keyboard.e && !rc_data[LAST].keyboard.e) kb_burst ^= 1;
    chassis_cmd_send.chassis_mode = kb_modes[kb_mode_idx];

    // W/S/A/D: 底盘移动
    chassis_cmd_send.vx = rc_data[CURRENT].keyboard.w ? 3000.0f : rc_data[CURRENT].keyboard.s ? -3000.0f : 0;
    chassis_cmd_send.vy = rc_data[CURRENT].keyboard.d ? 3000.0f : rc_data[CURRENT].keyboard.a ? -3000.0f : 0;

    // 鼠标: 云台 yaw/pitch
    gimbal_cmd_send.yaw   -= 0.005f * rc_data[CURRENT].mouse.x;
    gimbal_cmd_send.pitch += 0.005f * rc_data[CURRENT].mouse.y;
    if (gimbal_cmd_send.pitch > 40)       gimbal_cmd_send.pitch = 40;
    else if (gimbal_cmd_send.pitch < -30) gimbal_cmd_send.pitch = -30;

    // F: 摩擦轮开关, E: 切换射击模式, 鼠标左键: 发射
    shoot_cmd_send.shoot_mode  = kb_friction ? SHOOT_ON : SHOOT_OFF;
    shoot_cmd_send.loader_mode = rc_data[CURRENT].mouse.press_l ?
        (kb_burst ? LOAD_BURSTFIRE : LOAD_1_BULLET) : LOAD_STOP;
    if (kb_burst) shoot_cmd_send.shoot_rate = 8;
#endif
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线等
 *
 */
void Emergency_stop()
{
#if USE_SBUS_RECEIVER == 1
    // Ch5(SF)两档拨杆: 向下打到底(-660)进入急停模式
    // 使用 SBUS_3POS_THRESHOLD_DOWN 作为急停阈值 (< -300)
    if (sbus_data[CURRENT].rc.Ch5 < SBUS_3POS_THRESHOLD_DOWN)
    {
        robot_state = ROBOT_OFF;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    // Ch5不在急停位置时，恢复正常运行
    else
    {
        robot_state = ROBOT_ON;
    }
#elif USE_SBUS_RECEIVER == 2
    // 图传遥控: pause键触发急停
    if (vrc_data[CURRENT].rc.pause)
    {
        robot_state = ROBOT_OFF;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    else
    {
        robot_state = ROBOT_ON;
    }    
#else
    // 拨轮的向下打到底则进入急停模式
    if (rc_data[CURRENT].rc.dial < -300 || robot_state == ROBOT_OFF)
    {
        robot_state = ROBOT_OFF;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    // 遥控器右侧开关为[中],恢复正常运行
    if (rc_data[CURRENT].rc.Rswitch == SWITCH_IS_MID)
    {
        robot_state = ROBOT_ON;
    }
#endif
}
/**
 * @brief 根据gimbal传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360
 *
 */
void Calc_offset_angle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_feedback_recv.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    
    float temp_offset_angle;
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        temp_offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        temp_offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
    //计算出最终的偏差角
    chassis_cmd_send.offset_angle = temp_offset_angle;

#endif
    chassis_cmd_send.gimbal_yaw_total_angle = gimbal_feedback_recv.imu_yaw_total_angle;
    chassis_cmd_send.gimbal_yaw_rate = gimbal_feedback_recv.imu_yaw_rate;
}
