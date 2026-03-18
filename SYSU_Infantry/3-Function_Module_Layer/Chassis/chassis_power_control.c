#include "chassis_power_control.h"
#include "algorithm_pid.h"
#include "referee.h"         // 替换为你实际获取裁判系统数据的头文件
#include <math.h>

// ================== 物理常数与超参定义 ==================
#define K_TORQUE 0.0003662109375f             // 3508力矩转换系数
#define POWER_COEF (187.0f / 3591.0f / 9.55f) // 机械功率系数(适配rpm)
#define ERROR_UPPER_BOUND 200.0f              // RPM总误差上限 (完全大P分配)
#define ERROR_LOWER_BOUND 100.0f              // RPM总误差下限 (完全等比缩放)

// ================== RLS 动态模型参数 ==================
// 即使不运行 RLS 更新函数，这套初始参数也足够优秀，能直接作为前馈模型使用
static float k1_dynamic = 0.22f;       // 转速绝对值损耗系数
static float k2_dynamic = 1.23e-07f;   // 电流平方损耗系数
static float k3_static  = 2.78f;       // 底盘静态功耗 (W)

// RLS 矩阵变量 (2x2)
static float rls_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; 
static const float rls_lambda = 0.999f; 

// ================== 能量环控制器 ==================
static Pid_instance_t energy_pd;
static uint8_t is_energy_pd_init = 0;

void Chassis_Power_Control_Init(void)
{
    // 初始化能量闭环 PD 控制器 (根据缓冲能量动态压低功率上限)
    Pid_init_t pid_cfg = {
        .kp = 1.5f,
        .ki = 0.0f,
        .kd = 0.2f, // 引入D项，对缓冲能量骤降做出瞬间压制反应
        .max_out = 40.0f,
        .max_iout = 0.0f,
        .optimization = PID_OUTPUT_LIMIT,
    };
    Pid_init(&energy_pd, &pid_cfg);
    is_energy_pd_init = 1;
}

void Chassis_Power_RLS_Update(Djimotor_device_t *motors[4])
{
    // 【注意】最新赛季裁判系统不再提供实时功率反馈！
    // 只有当你的电路板上有 INA226 等电流计，能读到真实电功率时，再传入此变量
    float real_measured_power = 0.0f; // 替换为你的真实硬件功率读取函数
    
    // 如果没有真实功率反馈，强制退出 RLS，避免模型发散
    if (real_measured_power <= 0.0f) return; 

    float sum_abs_rpm = 0.0f;
    float sum_current_sq = 0.0f;
    float effective_power = 0.0f;

    for (int i = 0; i < 4; i++) {
        if (motors[i] == NULL || motors[i]->motor_status == MOTOR_STOP) continue;
        float rpm = motors[i]->motor_measure.angular_velocity;
        float current = motors[i]->motor_measure.real_current; 

        sum_abs_rpm += fabsf(rpm);
        sum_current_sq += current * current;
        effective_power += current * K_TORQUE * POWER_COEF * rpm; 
    }

    float y = real_measured_power - effective_power - k3_static;
    float x[2] = {sum_abs_rpm, sum_current_sq};
    
    // RLS 矩阵更新运算 (展开版)
    float Px[2] = {
        rls_P[0][0] * x[0] + rls_P[0][1] * x[1],
        rls_P[1][0] * x[0] + rls_P[1][1] * x[1]
    };
    float denominator = rls_lambda + (x[0] * Px[0] + x[1] * Px[1]);
    float K[2] = {Px[0] / denominator, Px[1] / denominator};
    float error = y - (k1_dynamic * x[0] + k2_dynamic * x[1]);

    k1_dynamic += K[0] * error;
    k2_dynamic += K[1] * error;

    if (k1_dynamic < 1e-5f) k1_dynamic = 1e-5f;
    if (k2_dynamic < 1e-7f) k2_dynamic = 1e-7f;

    float new_P[2][2];
    new_P[0][0] = (rls_P[0][0] - K[0] * Px[0]) / rls_lambda;
    new_P[0][1] = (rls_P[0][1] - K[0] * Px[1]) / rls_lambda;
    new_P[1][0] = (rls_P[1][0] - K[1] * Px[0]) / rls_lambda;
    new_P[1][1] = (rls_P[1][1] - K[1] * Px[1]) / rls_lambda;

    rls_P[0][0] = new_P[0][0]; rls_P[0][1] = new_P[0][1];
    rls_P[1][0] = new_P[1][0]; rls_P[1][1] = new_P[1][1];
}

void Chassis_Power_Control(Djimotor_device_t *motors[4])
{
    if (!is_energy_pd_init || motors == NULL) return;

    // 1. 获取裁判系统基础数据
    float buffer_energy = ChassisPower_GetBuffer();      // 实时缓冲能量,上限为60J
    float referee_max_power = ChassisPower_GetMaxLimit(); // 裁判系统上限
    if (referee_max_power < 40.0f) referee_max_power = 40.0f;

    // 2. 能量闭环：计算动态允许功率 P_max
    // 假设满缓冲 60J，设定维持目标在 45J 左右
    float target_buffer = 45.0f; 
    float pd_out = Pid_calculate(&energy_pd, target_buffer, buffer_energy);
    
    float P_max_limit = referee_max_power - pd_out; 
    
    // 安全底线：最惨情况(电量耗尽)也得给够 80% 的功率维持基本机动
    float min_power_limit = referee_max_power * 0.8f; 
    if (P_max_limit < min_power_limit) P_max_limit = min_power_limit;

    // 3. 全向轮核心：功率预测与负功回收
    float allocatable_power = P_max_limit;
    float cmd_power[4] = {0};
    float error_rpm[4] = {0};
    float sum_cmd_power = 0.0f;
    float sum_error_rpm = 0.0f;
    float sum_positive_power_req = 0.0f;

    for (int i = 0; i < 4; i++) {
        if (motors[i] == NULL || motors[i]->motor_status == MOTOR_STOP) continue;

        float rpm = motors[i]->motor_measure.angular_velocity;
        float target_rpm = motors[i]->motor_pid.pid_target;
        float pid_torque_current = motors[i]->out_current; // PID 原始电流

        error_rpm[i] = fabsf(target_rpm - rpm);

        // 预测命令功率模型
        cmd_power[i] = (pid_torque_current * K_TORQUE * POWER_COEF * rpm) + 
                       (k1_dynamic * fabsf(rpm)) + 
                       (k2_dynamic * pid_torque_current * pid_torque_current) + 
                       (k3_static / 4.0f);

        sum_cmd_power += cmd_power[i];

        // 负功回收：全向轮拖拽发电的轮子，把能量还给功率池
        if (cmd_power[i] <= 0.0f) {
            allocatable_power += -cmd_power[i]; 
        } else {
            sum_error_rpm += error_rpm[i];
            sum_positive_power_req += cmd_power[i];
        }
    }

    // 4. 超功率处理：大 P 误差分配 + 二次方程解算
    if (sum_cmd_power > P_max_limit && sum_positive_power_req > 0.0f) {
        
        // 计算误差置信度 K_coe
        float k_coe = 0.0f;
        if (sum_error_rpm > ERROR_UPPER_BOUND) {
            k_coe = 1.0f;
        } else if (sum_error_rpm > ERROR_LOWER_BOUND) {
            k_coe = (sum_error_rpm - ERROR_LOWER_BOUND) / (ERROR_UPPER_BOUND - ERROR_LOWER_BOUND);
        }

        for (int i = 0; i < 4; i++) {
            if (motors[i] == NULL || motors[i]->motor_status == MOTOR_STOP) continue;
            
            // 发电刹车的轮子直接放行
            if (cmd_power[i] <= 0.0f) continue;

            // 混合权重分配：平滑兼顾等比例缩放与误差突变补偿
            float weight_error = error_rpm[i] / (sum_error_rpm + 1e-6f);
            float weight_prop  = cmd_power[i] / sum_positive_power_req;
            float final_weight = (k_coe * weight_error) + ((1.0f - k_coe) * weight_prop);
            
            float target_P_i = allocatable_power * final_weight;

            // 二次方程逆解：A*I^2 + B*I + C = 0
            float rpm = motors[i]->motor_measure.angular_velocity;
            float A = k2_dynamic;
            float B = K_TORQUE * POWER_COEF * rpm;
            float C = (k1_dynamic * fabsf(rpm)) + (k3_static / 4.0f) - target_P_i;

            float delta = B * B - 4.0f * A * C;
            int16_t limited_current = motors[i]->out_current;

            if (delta <= 0.0f) {
                // 无解情况取极点
                limited_current = (int16_t)(-B / (2.0f * A));
            } else {
                // 有解：根据原始PID电流方向取根
                if (motors[i]->out_current > 0) {
                    limited_current = (int16_t)((-B + sqrtf(delta)) / (2.0f * A));
                } else {
                    limited_current = (int16_t)((-B - sqrtf(delta)) / (2.0f * A));
                }
            }

            // 硬件电流限幅与覆盖
            if (limited_current > 16000) limited_current = 16000;
            if (limited_current < -16000) limited_current = -16000;
            motors[i]->out_current = limited_current;
        }
    }
}