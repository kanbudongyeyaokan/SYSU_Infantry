#include "algorithm_pid.h"
#include "bsp_dwt.h"  // 必须包含DWT库
#include <string.h>
#include <math.h>     // 用于 fabsf

// 内部辅助宏
#define LIMIT_MAX(input, max) do { \
    if ((input) > (max)) (input) = (max); \
    else if ((input) < -(max)) (input) = -(max); \
} while(0)

/**
 * @brief 初始化PID
 */
void Pid_init(Pid_instance_t *pid, Pid_init_t *config)
{
    if (pid == NULL || config == NULL) return;

    memset(pid, 0, sizeof(Pid_instance_t));

    // 拷贝参数
    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->max_iout = config->max_iout;
    pid->max_out = config->max_out;
    pid->deadband = config->deadband;
    pid->optimization = config->optimization;
    pid->feedfoward_coefficient = config->feedfoward_coefficient;
    pid->LPF_coefficient = config->LPF_coefficient;
    pid->integral_separation_threshold = config->integral_separation_threshold;

    // 初始化时间戳，避免第一次计算dt过大
    DWT_GetDeltaT(&pid->dwt_counter);
    pid->dt = 0.001f; // 默认给一个安全值
}

/**
 * @brief 重置PID状态 (积分清零)
 */
void Pid_reset(Pid_instance_t *pid)
{
    if (pid == NULL) return;
    pid->Iout = 0.0f;
    pid->Pout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->last_error = 0.0f;
    pid->last_measure = 0.0f;
    pid->ITerm = 0.0f;
    // 更新一下时间戳，防止下次启动时dt突变
    DWT_GetDeltaT(&pid->dwt_counter);
}

/**
 * @brief PID计算核心
 */
float Pid_calculate(Pid_instance_t *pid, float measure, float target)
{
    if (pid == NULL) return 0.0f;

    // 1. 更新时间间隔 dt (关键修复)
    float dt = DWT_GetDeltaT(&pid->dwt_counter);

    // 保护: 防止 dt 异常 (如断点调试时 dt 变得巨大，或 dt 为 0)
    if (dt > 0.1f) dt = 0.1f;       // 最大允许 100ms 间隔
    if (dt < 0.0001f) dt = 0.0001f; // 最小允许 100us (防止除零)
    pid->dt = dt;

    // 2. 更新状态变量
    pid->measure = measure;
    pid->target = target;
    pid->error = pid->target - pid->measure;

    // 3. 死区判断
    /*
    if (fabsf(pid->error) < pid->deadband)
    {
        pid->error = 0.0f;
        // 进入死区可选策略：清空输出，或者保持 Iout 不变
        // 这里选择不更新 Output，直接返回上一次的值，或者输出0
        // 步兵底盘通常希望死区内无力
        pid->Pout = 0.0f;
        // 积分项是否清零视需求而定，通常不清零以保持姿态，但长时间死区应防饱和
       // return 0.0f;
    }
*/
    // 4. P项计算
    pid->Pout = pid->kp * pid->error;

    // 5. I项计算 (积分)
    // 梯形积分优化
    if (pid->optimization & PID_TRAPEZOID_INTERGRAL) {
        pid->ITerm = pid->ki * (pid->error + pid->last_error) / 2.0f * pid->dt;
    } else {
        // 标准积分: Ki * error * dt
        pid->ITerm = pid->ki * pid->error * pid->dt;
    }

    // 6. D项计算 (微分)
    // 微分先行优化 (对测量值微分，减少设定值突变带来的冲击)
    if (pid->optimization & PID_DIFFERENTIAL_GO_FIRST) {
        pid->Dout = pid->kd * (pid->last_measure - pid->measure) / pid->dt;
    } else {
        // 标准微分: Kd * d(err) / dt
        pid->Dout = pid->kd * (pid->error - pid->last_error) / pid->dt;
    }

    // 7. 计算 Iout (积分累加) 并进行抗饱和处理
        // 积分分离: 误差过大时关闭积分，防止超调
    if ((pid->optimization & PID_INTEGRAL_SEPARATION) &&
        (pid->integral_separation_threshold > 0.0f) &&
        (fabsf(pid->error) > pid->integral_separation_threshold))
    {
        pid->ITerm = 0.0f; // 误差超出阈值，暂停积分累加
    }

    // 智能积分抗饱和: 如果输出已经饱和，且积分项试图让饱和更严重，则停止积分
    float temp_output_prediction = pid->Pout + pid->Iout + pid->ITerm + pid->Dout;

    if (pid->optimization & PID_OUTPUT_LIMIT) {
        // 检查正向饱和
        if (temp_output_prediction > pid->max_out && pid->ITerm > 0) {
            pid->ITerm = 0; // 停止积分增加
        }
        // 检查负向饱和
        else if (temp_output_prediction < -pid->max_out && pid->ITerm < 0) {
            pid->ITerm = 0; // 停止积分减小
        }
    }

    pid->Iout += pid->ITerm;

    // 积分单独限幅
    if (pid->optimization & PID_OUTPUT_LIMIT) {
        LIMIT_MAX(pid->Iout, pid->max_iout);
    }

    // 8. 计算总输出
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    // 9. 前馈控制
    if (pid->optimization & PID_FEEDFOWARD) {
        pid->Output += pid->feedfoward_coefficient * pid->target;
    }

    // 10. 输出限幅
    if (pid->optimization & PID_OUTPUT_LIMIT) {
        LIMIT_MAX(pid->Output, pid->max_out);
    }

    // 11. 输出滤波 (低通)
    if (pid->optimization & PID_OUTPUT_FILTER) {
        // LPF公式: Y(n) = a * Y(n-1) + (1-a) * X(n)
        // 系数越大，滤波越强（越滞后）
        // 这里沿用你的公式逻辑，但建议用标准的 alpha 混合
        // 你的公式: out = out * dt / (Tc + dt) + last * Tc / (Tc + dt)
        // 这是一个标准的一阶低通滤波器
        float alpha = pid->dt / (pid->LPF_coefficient + pid->dt);
        pid->Output = pid->Output * alpha + pid->Last_Output * (1.0f - alpha);
    }

    // 12. 更新历史数据
    pid->last_measure = pid->measure;
    pid->last_error = pid->error;
    pid->Last_Output = pid->Output;

    return pid->Output;
}