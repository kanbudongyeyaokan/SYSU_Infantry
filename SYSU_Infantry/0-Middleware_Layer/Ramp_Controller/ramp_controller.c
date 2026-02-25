#include "ramp_controller.h"
#include <math.h>

/**
 * @brief 初始化斜坡控制器
 * @param ramp 结构体指针
 * @param initial_val 初始值（通常是电机的当前实际角度，防止上电疯转）
 * @param step 每次调用的最大步进量
 */
void Ramp_Init(Ramp_t *ramp, float initial_val, float step) {
    ramp->current_val = initial_val;
    ramp->target_val = initial_val;
    ramp->step = step;
}

/**
 * @brief 计算斜坡输出
 * @param ramp 结构体指针
 * @param target_val 最新的目标值（比如从底盘传过来的遥控器指令）
 * @return 平滑后的当前应发给电机的指令
 * @note 此函数需要以固定的频率调用（比如在你的 1000Hz Gimbal_task 中）
 */
float Ramp_Calc(Ramp_t *ramp, float target_val) {
    ramp->target_val = target_val;
    
    // 计算当前值和目标的差值
    float error = ramp->target_val - ramp->current_val;

    // 如果差值大于允许的最大步长，则只走一个步长
    if (fabsf(error) > ramp->step) {
        if (error > 0) {
            ramp->current_val += ramp->step;
        } else {
            ramp->current_val -= ramp->step;
        }
    } else {
        // 如果差值已经小于步长，直接等于目标值，防止在目标值附近震荡
        ramp->current_val = ramp->target_val;
    }

    return ramp->current_val;
}