#ifndef _ALGORITHM_PID_H
#define _ALGORITHM_PID_H

#include "main.h"

/*PID优化环节标志*/
typedef enum
{
    PID_OPTIMIZE_NONE           = 0b00000000,                  // 0000 0000 无优化
    PID_OUTPUT_LIMIT            = 0b00000001,                  // 0000 0001 输出限幅，包括积分限幅
    PID_DIFFERENTIAL_GO_FIRST   = 0b00000010,                  // 0000 0010 微分先行
    PID_TRAPEZOID_INTERGRAL     = 0b00000100,                  // 0000 0100 梯形积分
    PID_OUTPUT_FILTER           = 0b00001000,                  // 0000 1000 输出滤波
    PID_FEEDFOWARD              = 0b00010000                   // 0001 0000 前馈计算
} Pid_optimization_e;

/* PID结构体 */
typedef struct
{
    //PID基础参数
    float kp;
    float ki;
    float kd;
    float max_iout;  //最大积分输出
    float max_out;   //最大输出
    float deadband; //PID计算死区

    //PID计算参数
    float measure;          //当前测量值
    float last_measure;     //上一次测量值
    float target;           //当前目标值
    float last_target;      //上一次的目标值
    float error;            //当前误差
    float last_error;       //上一次误差
    float ITerm;            //当前积分计算值
    float last_ITerm;       //上一次积分计算值

    //PID输出值
    float Pout;
    float Iout;
    float Dout;
    float Output;
    float Last_Output;
    float Last_Dout;

    //PID优化选项
    Pid_optimization_e optimization;//优化选项
    float feedfoward_coefficient;   //前馈系数
    float LPF_coefficient;          //低通滤波器系数

    //PID控制周期计算
    uint32_t dwt_counter;   //当前计数器值
    uint16_t dt;            //控制周期

} Pid_instance_t;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    //PID基础参数
    float kp;
    float ki;
    float kd;
    float max_iout;  //最大积分输出
    float max_out;   //最大输出
    float deadband; //PID计算死区

    Pid_optimization_e optimization;//优化选项
    float feedfoward_coefficient;   //前馈系数
    float LPF_coefficient;          //低通滤波器系数
} Pid_init_t;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void Pid_init(Pid_instance_t *pid, Pid_init_t *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param target  设定值
 * @return float  PID计算输出
 */
float Pid_calculate(Pid_instance_t *pid, float measure, float target);


#endif //_ALGORITHM_PID_H