#ifndef SYSU_INFANTRY_SHOOT_HEAT_CONTROL_H
#define SYSU_INFANTRY_SHOOT_HEAT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* 每颗 17mm 子弹产生的热量（RM 规则固定值） */
#define SHOOT_HEAT_PER_BULLET_DEFAULT    10.0f

/* 距热量上限多少单位时开始按比例降速（默认 4 颗子弹余量） */
#define SHOOT_HEAT_DANGER_MARGIN_DEFAULT 40.0f

/* 距热量上限多少单位时硬封锁射击（默认 1.5 颗子弹余量） */
#define SHOOT_HEAT_BLOCK_MARGIN_DEFAULT  20.0f

typedef struct {
    float current_heat;    /* 当前枪管热量 */
    float heat_limit;      /* 裁判系统热量上限 */
    float heat_per_bullet; /* 单颗子弹热量 */
} shooter_heat_ctrl_input_t;

typedef struct {
    float danger_margin; /* 开始降速时距上限的热量余量 */
    float block_margin;  /* 硬封锁时距上限的热量余量 */
} shooter_heat_ctrl_param_t;

typedef struct {
    float heat_remaining; /* 当前剩余热量余量 */
    float rate_scale;     /* 射速缩放系数 0~1，1=全速，0=封锁 */
    bool  allow_shoot;    /* 是否允许触发射击 */
} shooter_heat_ctrl_output_t;

/* 纯计算函数，不依赖全局状态，便于测试 */
void Shooter_Heat_CalcScale(const shooter_heat_ctrl_input_t *input,
                             const shooter_heat_ctrl_param_t *param,
                             shooter_heat_ctrl_output_t *output);

/* 初始化（重置参数为默认值） */
void Shooter_Heat_Control_Init(void);

/* 包装函数：从裁判系统读取热量，填充 output，直接在 shoot.c 里调用 */
void Shooter_Heat_Control(shooter_heat_ctrl_output_t *output);

#endif /* SYSU_INFANTRY_SHOOT_HEAT_CONTROL_H */
