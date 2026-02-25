/**
 * @file    lowpass_filter.c
 * @brief   一阶低通滤波器实现
 */

#include "lowpass_filter.h"
#include <math.h>

#define PI 3.14159265358979f

void LPF_Init(Lpf_t *lpf, float dt, float cutoff_freq, float initial_val) {
    if (lpf == NULL) return;

    lpf->dt = dt;
    lpf->out = initial_val;
    
    LPF_Set_Cutoff_Freq(lpf, cutoff_freq);
}

void LPF_Set_Cutoff_Freq(Lpf_t *lpf, float cutoff_freq) {
    if (lpf == NULL) return;
    
    lpf->cutoff_freq = cutoff_freq;
    
    // 如果截止频率设为0或非法值，直接放行（alpha = 1）
    if (lpf->cutoff_freq <= 0.0f) {
        lpf->alpha = 1.0f;
    } else {
        // 计算 RC 常数
        // RC = 1 / (2 * PI * fc)
        float RC = 1.0f / (2.0f * PI * lpf->cutoff_freq);
        
        // 计算离散化 alpha 系数
        // alpha = dt / (RC + dt)
        lpf->alpha = lpf->dt / (RC + lpf->dt);
    }
}

float LPF_Calc(Lpf_t *lpf, float input) {
    if (lpf == NULL) return 0.0f;

    // 一阶惯性滤波公式: Y_n = alpha * X_n + (1 - alpha) * Y_{n-1}
    lpf->out = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->out;
    
    return lpf->out;
}

void LPF_Reset(Lpf_t *lpf, float reset_val) {
    if (lpf == NULL) return;
    lpf->out = reset_val;
}