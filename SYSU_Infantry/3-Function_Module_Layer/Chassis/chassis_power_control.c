#include "chassis_power_control.h"
#include <math.h>
#include "referee.h" // 确保包含你的裁判系统获取接口
#include "error_handler.h"

// ==================  3508 物理常数 ==================
#define TORQUE_COEF 0.0003662109375f        // (20/16384)*(0.3), 电机转矩系数
#define POWER_COEF (187.0f / 3591.0f / 9.55f) // 机械功率系数，适配 rpm
static const float K1[4] = {1.23e-07f, 1.23e-07f, 1.23e-07f, 1.23e-07f}; // 电流平方项系数
static const float K2[4] = {1.453e-07f, 1.453e-07f, 1.453e-07f, 1.453e-07f}; // 转速平方项系数
static const float constant[4] = {4.081f, 4.081f, 4.081f, 4.081f};       // 静态功耗

void Chassis_Power_Control(Djimotor_device_t *motors[4])
{
    if (motors == NULL) return;

    // 获取裁判系统状态
    float buffer_energy = ChassisPower_GetBuffer(); // 当前缓冲能量 (满管一般 60J)
    float ref_power_limit = ChassisPower_GetMaxLimit(); // 裁判系统给定的功率上限 (如 45W, 60W, 80W)

    if (ref_power_limit < 40.0f) ref_power_limit = 40.0f; // 容错机制

    // 缓冲能量动态限幅 
    float chassis_max_power = ref_power_limit;
    
    // 假设满缓冲是 60J。当跌破 30J 时，开始强制线性压低功率上限
    if (buffer_energy < 30.0f) {
        // 留 10J 作为绝对死线，低于 10J 功率直接降到 10W 以下保命
        float scale = (buffer_energy - 10.0f) / 20.0f; 
        if (scale < 0.0f) scale = 0.0f;
        chassis_max_power = 10.0f + (ref_power_limit - 10.0f) * scale;
    }

    // 统计 4 个轮子的基础预测功率 
    float initial_total_power = 0.0f;
    float initial_give_power[4] = {0};

    for (uint8_t i = 0; i < 4; i++) {
        if (motors[i] == NULL || motors[i]->motor_status == MOTOR_STOP) continue;

        float speed_rpm = motors[i]->motor_measure.angular_velocity;
        float current_cmd = (float)motors[i]->out_current; // PID算出的原始需求电流

        float A = K1[i] * current_cmd * current_cmd;
        float B = K2[i] * speed_rpm * speed_rpm;
        float C = POWER_COEF * speed_rpm * current_cmd * TORQUE_COEF;

        initial_give_power[i] = A + B + C + constant[i];

        // 仅将做功状态（消耗功率 > 0）的功率计入总和，发电刹车不计入
        if (initial_give_power[i] > 0.0f) {
            initial_total_power += initial_give_power[i];
        }
    }

    // 判断是否超功率，执行等比例功率缩放与二次方程逆解
    if (initial_total_power > chassis_max_power) {
        float ratio = chassis_max_power / initial_total_power;

        for (uint8_t i = 0; i < 4; i++) {
            if (motors[i] == NULL || motors[i]->motor_status == MOTOR_STOP) continue;

            // 发电状态（功率 < 0）直接跳过限制，保证刹车性能
            if (initial_give_power[i] <= 0.0f) continue;

            // 计算该轮子被分配到的目标功率
            float target_power = initial_give_power[i] * ratio;
            float speed_rpm = motors[i]->motor_measure.angular_velocity;

            // 构建二次方程: a*I^2 + b*I + c = 0
            float a = K1[i];
            float b = TORQUE_COEF * POWER_COEF * speed_rpm;
            float c = K2[i] * speed_rpm * speed_rpm - target_power + constant[i];

            float discriminant = b * b - 4.0f * a * c;

            if (discriminant >= 0.0f) {
                // 根据原始 PID 期望电流的方向，决定取正根还是负根
                if (motors[i]->out_current > 0) {
                    motors[i]->out_current = (int16_t)((-b + sqrtf(discriminant)) / (2.0f * a));
                    if (motors[i]->out_current > 16000) motors[i]->out_current = 16000;
                } else {
                    motors[i]->out_current = (int16_t)((-b - sqrtf(discriminant)) / (2.0f * a));
                    if (motors[i]->out_current < -16000) motors[i]->out_current = -16000;
                }
            } else {
                // 无解极端情况：纯摩擦和静态损耗已超过分配功率，强制锁死电流保命
                motors[i]->out_current = 0;
            }
        }
    }
}