#ifndef RAMP_CONTROLLER_H
#define RAMP_CONTROLLER_H

#include <stdint.h>

// 斜坡控制器结构体
typedef struct {
    float current_val; // 当前平滑后的输出值
    float target_val;  // 最终的目标值
    float step;        // 每次执行允许的最大步进量 (决定了斜坡的坡度)
} Ramp_t;

// 函数声明
void Ramp_Init(Ramp_t *ramp, float initial_val, float step);
float Ramp_Calc(Ramp_t *ramp, float target_val);

#endif