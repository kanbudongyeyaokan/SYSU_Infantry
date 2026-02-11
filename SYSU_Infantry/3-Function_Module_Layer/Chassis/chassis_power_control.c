#include "chassis_power_control.h"
#include "algorithm_pid.h"
#include <math.h>
#include <stdbool.h>
#include "referee.h"
#include "remote_control.h"
#include "supercap_comm.h"

static Pid_instance_t chassis_buffer_pid;
static uint8_t cap_state = 0;



static bool ChassisPower_IsKeyPressedE(void)
{
    // TODO: 是否按下e

    return false;
}

static bool ChassisPower_IsKeyPressedQ(void)
{
    // TODO:是否按下Q
    return false;
}

static uint8_t Is_Enable_DCDC()
{
    static uint8_t last_state = 1;//默认上电启用超点
    //不使用超点
    if (ChassisPower_IsKeyPressedE())
    {
        last_state = 0;
        return 0;
    }
    //使用超点
    if (ChassisPower_IsKeyPressedQ())
    {
        last_state = 1;
        return 1;
    }
    return last_state;
}

//控制缓冲能量pid
void Chassis_Power_Control_Init(void)
{
    Pid_init_t pid_cfg = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_iout = 1000.0f,
        .max_out = 1000.0f,
        .deadband = 0.0f,
        .optimization = PID_OUTPUT_LIMIT,
        .feedfoward_coefficient = 0.0f,
        .LPF_coefficient = 0.0f,
    };

    Pid_init(&chassis_buffer_pid, &pid_cfg);
}

void Chassis_Power_Control(Chassis_output_t *output, Djimotor_device_t *motors[4])
{
    if (output == NULL || motors == NULL)
    {
        return;
    }

    uint16_t max_power_limit = 40;//最低为40
    float chassis_max_power = 0.0f;
    float input_power = 0.0f;
    float initial_give_power[4] = { 0 };
    float initial_total_power = 0.0f;

    float chassis_power_buffer = 0.0f;

    const float torque_coefficient = 1.99688994e-6f;
    const float a = 1.453e-07f;
    const float k2 = 1.23e-07f;

    const float constant = 4.081f;

    chassis_power_buffer = ChassisPower_GetBuffer();

    //将缓冲能量控制在50，充分利用能量
    float buffer_pid_out = Pid_calculate(&chassis_buffer_pid, chassis_power_buffer, 50.0f);

    max_power_limit = ChassisPower_GetMaxLimit();//根据等级增长最大规律变化

    //加上缓冲能量可用的功率，得到当前底盘可用的最大功率，加或减实际测试
    input_power = (float) max_power_limit - buffer_pid_out;

    cap_state = Is_Enable_DCDC();

    //超点容量大于5%（255*0.05≈13）
    if (SuperCap_Get_Cap_Energy() > 13)
    {
        if (cap_state == 0)
        {
            chassis_max_power = input_power + 5.0f;
        }
        else
        {
            chassis_max_power = input_power + 200.0f;
        }
    }
    else
    {
        chassis_max_power = input_power;
    }

    
    for (uint8_t i = 0; i < 4; i++)
    {
        float speed_rpm = motors[i]->motor_measure.angular_velocity;
        float torque_cmd_proxy = output->motor_speed[i];

        initial_give_power[i] = torque_cmd_proxy * torque_coefficient * speed_rpm +
            k2 * speed_rpm * speed_rpm +
            a * torque_cmd_proxy * torque_cmd_proxy +
            constant;

        if (initial_give_power[i] < 0.0f)
        {
            continue;
        }
        initial_total_power += initial_give_power[i];
    }

    if (initial_total_power > chassis_max_power && initial_total_power > 0.0f)
    {
        float power_scale = chassis_max_power / initial_total_power;
        float scaled_give_power[4] = { 0 };

        for (uint8_t i = 0; i < 4; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale;
            if (scaled_give_power[i] < 0.0f)
            {
                continue;
            }

            float speed_rpm = motors[i]->motor_measure.angular_velocity;
            float b = torque_coefficient * speed_rpm;
            float c = k2 * speed_rpm * speed_rpm - scaled_give_power[i] + constant;
            float inside = b * b - 4.0f * a * c;

            if (inside < 0.0f)
            {
                continue;
            }

            if (output->motor_speed[i] > 0.0f)
            {
                float temp = (-b + sqrtf(inside)) / (2.0f * a);
                if (temp > 16000.0f)
                {
                    output->motor_speed[i] = 16000.0f;
                }
                else
                {
                    output->motor_speed[i] = temp;
                }
            }
            else
            {
                float temp = (-b - sqrtf(inside)) / (2.0f * a);
                if (temp < -16000.0f)
                {
                    output->motor_speed[i] = -16000.0f;
                }
                else
                {
                    output->motor_speed[i] = temp;
                }
            }
        }
    }
}


void Send2SuperCap(void)
{
    SuperCap_TxData tx_data = {
        .enable_dcdc = Is_Enable_DCDC(),
        .system_restart = 0,
        .feedback_referee_power_limit = ChassisPower_GetMaxLimit(),
        .feedback_referee_energy_buffer = ChassisPower_GetBuffer(),
    };

    SuperCap_Comm_Send(&tx_data);
}