#include "chassis_power_control.h"

#include <stddef.h>
#include <stdbool.h>
#include "error_handler.h"
#include "referee.h"
#include "supercap_comm.h"
#define CHASSIS_POWER_WHEEL_NUM          4U
#define CHASSIS_POWER_EPSILON            1e-6f
#define CHASSIS_MOTOR_CURRENT_MAX        16000.0f
#define CHASSIS_POWER_ERR_INTERVAL_TICK  200U

#define CHASSIS_PWR_MODULE               "CHASSIS_PWR"

#define CHASSIS_POWER_LIMIT_DEFAULT      40.0f
#define CHASSIS_POWER_BUFFER_DEFAULT     60.0f

#define CHASSIS_POWER_K_T_DEFAULT        1.0e-4f
#define CHASSIS_POWER_STATIC_DEFAULT     2.0f
#define CHASSIS_POWER_DANGER_LINE_DEFAULT 30.0f
#define CHASSIS_POWER_BUFFER_KP_DEFAULT  1.0f
#define CHASSIS_POWER_MIN_ALLOW_DEFAULT  1.0f

static chassis_power_ctrl_param_t g_chassis_power_param =
{
    .k_t = CHASSIS_POWER_K_T_DEFAULT,
    .p_static = CHASSIS_POWER_STATIC_DEFAULT,
    .danger_energy_line = CHASSIS_POWER_DANGER_LINE_DEFAULT,
    .k_p_buffer = CHASSIS_POWER_BUFFER_KP_DEFAULT,
    .p_min_allow = CHASSIS_POWER_MIN_ALLOW_DEFAULT,
};

/* 错误节流与恢复状态，防止 1kHz 日志刷屏 */
static uint16_t g_null_input_err_cd = 0U;
static uint16_t g_null_motors_err_cd = 0U;
static uint16_t g_ref_limit_err_cd = 0U;
static uint16_t g_ref_buffer_err_cd = 0U;
static uint16_t g_param_pmin_err_cd = 0U;
static uint16_t g_pmax_floor_warn_cd = 0U;

static uint16_t g_motor_null_warn_cd[CHASSIS_POWER_WHEEL_NUM] = {0U};
static bool g_motor_null_active[CHASSIS_POWER_WHEEL_NUM] = {false};
static bool g_ref_limit_abnormal_active = false;
static bool g_ref_buffer_abnormal_active = false;
static bool g_pmax_floor_active = false;

static bool chassis_power_should_report(uint16_t *cooldown)
{
    if (cooldown == NULL)
    {
        return false;
    }

    if (*cooldown == 0U)
    {
        *cooldown = CHASSIS_POWER_ERR_INTERVAL_TICK;
        return true;
    }

    (*cooldown)--;
    return false;
}

static float chassis_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float chassis_clampf(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

static int16_t chassis_float_to_i16_clamped(float x)
{
    float bounded = chassis_clampf(x, -CHASSIS_MOTOR_CURRENT_MAX, CHASSIS_MOTOR_CURRENT_MAX);
    if (bounded >= 0.0f)
    {
        return (int16_t)(bounded + 0.5f);
    }
    return (int16_t)(bounded - 0.5f);
}

void Chassis_Power_Control_Init(void)
{
    g_chassis_power_param.k_t = CHASSIS_POWER_K_T_DEFAULT;
    g_chassis_power_param.p_static = CHASSIS_POWER_STATIC_DEFAULT;
    g_chassis_power_param.danger_energy_line = CHASSIS_POWER_DANGER_LINE_DEFAULT;
    g_chassis_power_param.k_p_buffer = CHASSIS_POWER_BUFFER_KP_DEFAULT;
    g_chassis_power_param.p_min_allow = CHASSIS_POWER_MIN_ALLOW_DEFAULT;

    ERROR_INFO(CHASSIS_PWR_MODULE,
               "init k_t=%.6f p_static=%.2f danger=%.2f kp=%.2f pmin=%.2f",
               g_chassis_power_param.k_t,
               g_chassis_power_param.p_static,
               g_chassis_power_param.danger_energy_line,
               g_chassis_power_param.k_p_buffer,
               g_chassis_power_param.p_min_allow);
}

void Chassis_Power_CalcAndScale(const chassis_power_ctrl_input_t *input,
                                const chassis_power_ctrl_param_t *param,
                                chassis_power_ctrl_output_t *output)
{
    uint8_t i;
    float p_total = 0.0f;
    float p_max_allow;
    float p_limit;
    float e_buffer;
    float p_min_allow;
    float p_max_allow_raw;
    float alpha = 1.0f;

    if ((input == NULL) || (param == NULL) || (output == NULL))
    {
        if (chassis_power_should_report(&g_null_input_err_cd))
        {
            ERROR_RAISE(CHASSIS_PWR_MODULE,
                        "calc null ptr input=0x%lx param=0x%lx output=0x%lx",
                        (uint32_t)(uintptr_t)input,
                        (uint32_t)(uintptr_t)param,
                        (uint32_t)(uintptr_t)output);
        }
        return;
    }

    p_limit = input->p_limit;
    e_buffer = input->e_buffer;
    p_min_allow = param->p_min_allow;

    if (p_min_allow <= CHASSIS_POWER_EPSILON)
    {
        if (chassis_power_should_report(&g_param_pmin_err_cd))
        {
            ERROR_WARN(CHASSIS_PWR_MODULE,
                       "param p_min_allow invalid=%.3f, fallback=%.3f",
                       p_min_allow,
                       CHASSIS_POWER_MIN_ALLOW_DEFAULT);
        }
        p_min_allow = CHASSIS_POWER_MIN_ALLOW_DEFAULT;
    }

    if (p_limit <= 0.0f)
    {
        if (chassis_power_should_report(&g_ref_limit_err_cd))
        {
            ERROR_WARN(CHASSIS_PWR_MODULE,
                       "ref power limit invalid=%.2f, clamp to p_min_allow=%.2f",
                       p_limit,
                       p_min_allow);
        }
        p_limit = p_min_allow;
        g_ref_limit_abnormal_active = true;
    }
    else if (g_ref_limit_abnormal_active)
    {
        ERROR_INFO(CHASSIS_PWR_MODULE, "ref power limit recovered=%.2f", p_limit);
        g_ref_limit_abnormal_active = false;
    }

    if (e_buffer < 0.0f)
    {
        if (chassis_power_should_report(&g_ref_buffer_err_cd))
        {
            ERROR_WARN(CHASSIS_PWR_MODULE,
                       "buffer energy invalid=%.2f, clamp to 0", e_buffer);
        }
        e_buffer = 0.0f;
        g_ref_buffer_abnormal_active = true;
    }
    else if (g_ref_buffer_abnormal_active)
    {
        ERROR_INFO(CHASSIS_PWR_MODULE, "buffer energy recovered=%.2f", e_buffer);
        g_ref_buffer_abnormal_active = false;
    }

    for (i = 0U; i < CHASSIS_POWER_WHEEL_NUM; i++)
    {
        /* 步骤1：单轮功率估算 P_esti = K_t * abs(I_cmd * w_fdb) + P_static */
        float i_mul_w = input->i_cmd[i] * input->w_fdb[i];
        float p_wheel = param->k_t * chassis_absf(i_mul_w) + param->p_static;
        output->p_wheel_esti[i] = p_wheel;

        /* 步骤2：四轮功率求和 */
        p_total += p_wheel;
    }

    output->p_total = p_total;

    /* 步骤3：缓冲能量防线动态限功 */
    if (e_buffer > param->danger_energy_line)
    {
        p_max_allow_raw = p_limit;
    }
    else
    {
        p_max_allow_raw = p_limit -
                          param->k_p_buffer * (param->danger_energy_line - e_buffer);
    }

    p_max_allow = p_max_allow_raw;

    /* 底层保底，避免功率上限 <= 0 */
    if (p_max_allow < p_min_allow)
    {
        p_max_allow = p_min_allow;

        if (chassis_power_should_report(&g_pmax_floor_warn_cd))
        {
            ERROR_WARN(CHASSIS_PWR_MODULE,
                       "p_max_allow floor raw=%.2f floor=%.2f p_limit=%.2f e_buffer=%.2f",
                       p_max_allow_raw,
                       p_min_allow,
                       p_limit,
                       e_buffer);
        }
        g_pmax_floor_active = true;
    }
    else if (g_pmax_floor_active)
    {
        ERROR_INFO(CHASSIS_PWR_MODULE,
                   "p_max_allow recovered raw=%.2f p_limit=%.2f e_buffer=%.2f",
                   p_max_allow_raw,
                   p_limit,
                   e_buffer);
        g_pmax_floor_active = false;
    }
    output->p_max_allow = p_max_allow;

    /* 步骤4：超功率时做等比例电流缩放 */
    if ((p_total > p_max_allow) && (p_total > CHASSIS_POWER_EPSILON))
    {
        /* 除零保护：仅在 p_total 足够大时进行除法 */
        alpha = p_max_allow / p_total;
        if (alpha < 0.0f)
        {
            alpha = 0.0f;
        }
        if (alpha > 1.0f)
        {
            alpha = 1.0f;
        }
    }
    else
    {
        alpha = 1.0f;
    }

    output->alpha = alpha;

    for (i = 0U; i < CHASSIS_POWER_WHEEL_NUM; i++)
    {
        output->i_out[i] = input->i_cmd[i] * alpha;
    }
}

void Chassis_Power_Control(Djimotor_device_t *motors[4])
{
    uint8_t i;
    chassis_power_ctrl_input_t input;
    chassis_power_ctrl_output_t output;

    if (motors == NULL)
    {
        if (chassis_power_should_report(&g_null_motors_err_cd))
        {
            ERROR_RAISE(CHASSIS_PWR_MODULE, "motors array is NULL");
        }
        return;
    }

    for (i = 0U; i < CHASSIS_POWER_WHEEL_NUM; i++)
    {
        if ((motors[i] == NULL) || (motors[i]->motor_status == MOTOR_STOP))
        {
            input.i_cmd[i] = 0.0f;
            input.w_fdb[i] = 0.0f;

            if (motors[i] == NULL)
            {
                if (chassis_power_should_report(&g_motor_null_warn_cd[i]))
                {
                    ERROR_WARN(CHASSIS_PWR_MODULE, "motor[%lu] is NULL", (uint32_t)i);
                }
                g_motor_null_active[i] = true;
            }
            continue;
        }

        if (g_motor_null_active[i])
        {
            ERROR_INFO(CHASSIS_PWR_MODULE, "motor[%lu] pointer recovered", (uint32_t)i);
            g_motor_null_active[i] = false;
        }

        input.i_cmd[i] = (float)motors[i]->out_current;
        input.w_fdb[i] = motors[i]->motor_measure.angular_velocity;
    }

    input.p_limit = (float)SuperCap_Get_Power_Limit();
    input.e_buffer = (float)ChassisPower_GetBuffer();

    if (input.p_limit < CHASSIS_POWER_LIMIT_DEFAULT)
    {
        input.p_limit = (float)ChassisPower_GetMaxLimit();
        if (input.p_limit < CHASSIS_POWER_LIMIT_DEFAULT)
        {
            input.p_limit = CHASSIS_POWER_LIMIT_DEFAULT;
        }
    }

    if (input.e_buffer < 0.0f)
    {
        input.e_buffer = CHASSIS_POWER_BUFFER_DEFAULT;
    }
    ERROR_INFO(CHASSIS_PWR_MODULE ,"got power limit: %.2fW buffer: %.2f%%", input.p_limit, input.e_buffer);
    Chassis_Power_CalcAndScale(&input, &g_chassis_power_param, &output);

    for (i = 0U; i < CHASSIS_POWER_WHEEL_NUM; i++)
    {
        if ((motors[i] == NULL) || (motors[i]->motor_status == MOTOR_STOP))
        {
            continue;
        }
        motors[i]->out_current = chassis_float_to_i16_clamped(output.i_out[i]);
    }
}

