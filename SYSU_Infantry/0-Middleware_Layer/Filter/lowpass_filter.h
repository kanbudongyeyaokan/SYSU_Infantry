/**
 * @file    lowpass_filter.h
 * @brief   一阶低通滤波器库
 * @note    采用截止频率计算 alpha，与运行频率解耦，适用于遥控器平滑、传感器滤波
 */

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

// 一阶低通滤波器结构体
typedef struct {
    float dt;           // 控制周期 (单位: s)，例如 1000Hz 对应 0.001f
    float cutoff_freq;  // 截止频率 (单位: Hz)，越小越平滑，但也越滞后
    float alpha;        // 内部计算的滤波系数 (0 ~ 1)
    float out;          // 滤波器当前的输出值
} Lpf_t;

/**
 * @brief  初始化一阶低通滤波器
 * @param  lpf: 滤波器实例指针
 * @param  dt: 任务运行周期 (秒)
 * @param  cutoff_freq: 截止频率 (Hz)
 * @param  initial_val: 初始值 (极其重要！防止上电第一帧出现剧烈阶跃)
 */
void LPF_Init(Lpf_t *lpf, float dt, float cutoff_freq, float initial_val);

/**
 * @brief  修改截止频率 (支持在线动态调参)
 * @param  lpf: 滤波器实例指针
 * @param  cutoff_freq: 新的截止频率
 */
void LPF_Set_Cutoff_Freq(Lpf_t *lpf, float cutoff_freq);

/**
 * @brief  低通滤波器计算
 * @param  lpf: 滤波器实例指针
 * @param  input: 当前最新输入值 (如：遥控器传来的阶跃目标角度)
 * @retval 平滑后的输出值
 */
float LPF_Calc(Lpf_t *lpf, float input);

/**
 * @brief  重置滤波器状态
 * @param  lpf: 滤波器实例指针
 * @param  reset_val: 重置后的当前值
 */
void LPF_Reset(Lpf_t *lpf, float reset_val);

#endif // LOWPASS_FILTER_H