#include "algorithm_pid.h"
#include "string.h"
#include "bsp_dwt.h"

#define abs(x) ((x > 0) ? x : -x)

#define LimitMax(input, max)            \
{                                       \
    if (input > max) {                  \
    input = max;                        \
    } else if (input < -max) {          \
    input = -max;                       \
    }                                   \
}

//输出限幅选项
static void Limit_ouput(Pid_instance_t* pid)
{
    float Iout_temp = pid->Iout + pid->ITerm;
    float output = pid->Output;
    //当输出超过限幅时停止积分
    if (abs(output)>pid->max_out)
    {
        //仍然朝着正方向积分
        if (pid->error * pid->Iout >0)
            pid->ITerm = 0;
    }
    //输出限幅和积分限幅
    LimitMax(pid->Iout,pid->max_iout);
    LimitMax(pid->Output,pid->max_out);
}

//梯形积分选项
static void Trapezoid_intergral(Pid_instance_t *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->ki * ((pid->error + pid->last_error) / 2) * pid->dt;
}

//微分先行选项
static void Differential_on_measurement(Pid_instance_t *pid)
{
    pid->Dout = pid->kd * (pid->last_measure - pid->measure) / pid->dt;
}

// 输出低通滤波
static void Output_filter(Pid_instance_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->LPF_coefficient + pid->dt) +
                  pid->Last_Output * pid->LPF_coefficient / (pid->LPF_coefficient + pid->dt);
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void Pid_init(Pid_instance_t *pid, Pid_init_t *config)
{
    memset(pid, 0, sizeof(Pid_instance_t));
    memcpy(pid, config, sizeof(Pid_init_t));
    DWT_GetDeltaT(&pid->dwt_counter);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Pid_calculate(Pid_instance_t *pid, float measure, float target)
{
    // 保存上次的测量值和误差,计算当前error
    pid->measure = measure;
    pid->target = target;
    pid->error = pid->target - pid->measure;

    // 如果在死区外,则计算PID
    if (abs(pid->error) > pid->deadband)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->kp * pid->error;
        pid->ITerm = pid->ki * pid->error;
        pid->Dout = pid->kd * (pid->error - pid->last_error);

        // 梯形积分
        if (pid->optimization & PID_TRAPEZOID_INTERGRAL)
            Trapezoid_intergral(pid);
        // 微分先行
        if (pid->optimization & PID_DIFFERENTIAL_GO_FIRST)
            Differential_on_measurement(pid);
        // 输出/积分限幅
        if (pid->optimization & PID_OUTPUT_LIMIT)
            Limit_ouput(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->optimization & PID_OUTPUT_FILTER)
            Output_filter(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->last_measure = pid->measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->last_error = pid->error;
    pid->last_ITerm = pid->ITerm;
    return pid->Output;
}
