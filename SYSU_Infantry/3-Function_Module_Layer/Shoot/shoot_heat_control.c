#include "shoot_heat_control.h"
#include "referee.h"
#include "error_handler.h"

#define SHOOT_HEAT_MODULE "SHOOT_HEAT"
#define SHOOT_HEAT_ERR_INTERVAL_TICK 200U

static shooter_heat_ctrl_param_t g_heat_param = {
    .danger_margin = SHOOT_HEAT_DANGER_MARGIN_DEFAULT,
    .block_margin  = SHOOT_HEAT_BLOCK_MARGIN_DEFAULT,
};

/* ---- 节流计数器 ---- */
static uint16_t g_warn_offline_cd  = 0U;
static uint16_t g_warn_limit_cd    = 0U;
static uint16_t g_warn_block_cd    = 0U;

static bool g_blocking_active = false;

static bool heat_should_report(uint16_t *cd)
{
    if (*cd == 0U) {
        *cd = SHOOT_HEAT_ERR_INTERVAL_TICK;
        return true;
    }
    (*cd)--;
    return false;
}

void Shooter_Heat_Control_Init(void)
{
    g_heat_param.danger_margin = SHOOT_HEAT_DANGER_MARGIN_DEFAULT;
    g_heat_param.block_margin  = SHOOT_HEAT_BLOCK_MARGIN_DEFAULT;
    ERROR_INFO(SHOOT_HEAT_MODULE,
               "init danger_margin=%.1f block_margin=%.1f heat_per_bullet=%.1f",
               g_heat_param.danger_margin,
               g_heat_param.block_margin,
               SHOOT_HEAT_PER_BULLET_DEFAULT);
}

void Shooter_Heat_CalcScale(const shooter_heat_ctrl_input_t *input,
                             const shooter_heat_ctrl_param_t *param,
                             shooter_heat_ctrl_output_t *output)
{
    float heat_remaining;
    float scaled_block;
    float scaled_danger;

    if (input == NULL || param == NULL || output == NULL) {
        return;
    }

    heat_remaining = input->heat_limit - input->current_heat;
    if (heat_remaining < 0.0f) heat_remaining = 0.0f;

    output->heat_remaining = heat_remaining;

    scaled_block  = param->block_margin;
    scaled_danger = param->danger_margin;

    /* 确保 danger > block，避免除零 */
    if (scaled_danger <= scaled_block) {
        scaled_danger = scaled_block + input->heat_per_bullet;
    }

    if (heat_remaining <= scaled_block) {
        /* 硬封锁：剩余热量不够再打一颗 */
        output->rate_scale  = 0.0f;
        output->allow_shoot = false;
    } else if (heat_remaining >= scaled_danger) {
        /* 安全区：全速 */
        output->rate_scale  = 1.0f;
        output->allow_shoot = true;
    } else {
        /* 危险区：线性降速 */
        output->rate_scale = (heat_remaining - scaled_block)
                           / (scaled_danger - scaled_block);
        output->allow_shoot = true;
    }
}

void Shooter_Heat_Control(shooter_heat_ctrl_output_t *output)
{
    shooter_heat_ctrl_input_t input;

    if (output == NULL) {
        return;
    }

    /* 裁判系统离线：放行但打警告，避免误封锁 */
    if (!Referee_Is_Online()) {
        if (heat_should_report(&g_warn_offline_cd)) {
            ERROR_WARN(SHOOT_HEAT_MODULE, "referee offline, heat control bypassed");
        }
        output->heat_remaining = 0.0f;
        output->rate_scale     = 1.0f;
        output->allow_shoot    = true;
        return;
    }

    input.current_heat    = (float)Shooter_GetHeat17mm();
    input.heat_limit      = (float)Shooter_GetHeatLimit();
    input.heat_per_bullet = SHOOT_HEAT_PER_BULLET_DEFAULT;

    /* 热量上限为 0 说明裁判数据还没来 */
    if (input.heat_limit <= 0.0f) {
        if (heat_should_report(&g_warn_limit_cd)) {
            ERROR_WARN(SHOOT_HEAT_MODULE, "heat_limit=0, heat control bypassed");
        }
        output->heat_remaining = 0.0f;
        output->rate_scale     = 1.0f;
        output->allow_shoot    = true;
        return;
    }

    Shooter_Heat_CalcScale(&input, &g_heat_param, output);

    if (!output->allow_shoot) {
        if (!g_blocking_active) {
            ERROR_WARN(SHOOT_HEAT_MODULE,
                       "heat blocked! heat=%.0f limit=%.0f remaining=%.0f",
                       input.current_heat, input.heat_limit, output->heat_remaining);
            g_blocking_active = true;
        }
    } else if (g_blocking_active) {
        ERROR_INFO(SHOOT_HEAT_MODULE,
                   "heat unblocked, remaining=%.0f scale=%.2f",
                   output->heat_remaining, output->rate_scale);
        g_blocking_active = false;
    }
}
