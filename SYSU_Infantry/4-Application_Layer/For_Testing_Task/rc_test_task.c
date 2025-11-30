/**
 * @file    rc_test_task.c
 * @brief   DJI Motor Lib 全功能验收测试 (Final Acceptance Test)
 * @author  SYSU电控组
 * @note    这是一个自动化测试脚本，将依次测试电机库的各项功能。
 * 请连接好 GM6020 (ID 1) 并确保电源开启。
 */

#include "rc_test_task.h"
#include "bsp_dwt.h"
#include "main.h"
#include "dji_motor.h"
#include "bsp_can.h"
#include <stdio.h>
#include <math.h>

extern CAN_HandleTypeDef hcan1;

// 测试电机指针
static Djimotor_device_t *test_motor = NULL;

/* 测试阶段枚举 */
typedef enum {
    STAGE_INIT = 0,
    STAGE_SPEED_SINE,   // 1. 速度环正弦波 (测试 PID 和基本控制)
    STAGE_POS_STEP,     // 2. 位置环阶跃 (测试多圈角度和串级 PID)
    STAGE_TIMEOUT,      // 3. 超时保护 (测试看门狗)
    STAGE_RECOVERY,     // 4. 恢复控制 (测试重连)
    STAGE_END
} Test_Stage_e;

static char *Stage_Names[] = {
    "INIT",
    "SPEED SINE WAVE",
    "POSITION STEP (Multi-Turn)",
    "WATCHDOG TIMEOUT SIMULATION",
    "RECOVERY & END"
};

void Rc_test_task(void const *argument)
{
    osDelay(1000); // 等待系统稳定

    // 如果 main.c 没调 Can_init，这里调一次
    Can_init();

    printf("\r\n=============================================\r\n");
    printf("    DJI MOTOR LIB ACCEPTANCE TEST START\r\n");
    printf("=============================================\r\n");

    // --- 1. 初始化 ---
    Djimotor_init_config_t init_conf = {0};
    init_conf.motor_type = GM6020;
    init_conf.can_init.can_handle = &hcan1;
    init_conf.can_init.can_id = 0x1FF; // TX Group
    init_conf.can_init.tx_id = 1;      // ID 1
    init_conf.can_init.rx_id = 0x205;  // RX ID
    strcpy(init_conf.motor_name, "Test_GM6020");

    // 初始 PID (速度环)
    // 使用我们之前调好的参数
    init_conf.motor_controller_init.close_loop = SPEED_LOOP;
    init_conf.motor_controller_init.speed_pid.kp = 1.2f;
    init_conf.motor_controller_init.speed_pid.ki = 800.0f;
    init_conf.motor_controller_init.speed_pid.max_out = 28000.0f;
    init_conf.motor_controller_init.speed_pid.max_iout = 10000.0f;
    init_conf.motor_controller_init.speed_pid.optimization = PID_OUTPUT_LIMIT | PID_TRAPEZOID_INTERGRAL | PID_FEEDFOWARD;
    init_conf.motor_controller_init.speed_pid.feedfoward_coefficient = 12.0f;

    test_motor = DJI_Motor_Init(&init_conf);

    if (test_motor == NULL) {
        printf("[FAIL] Motor Init Failed!\r\n");
        for(;;) osDelay(1000);
    }
    printf("[PASS] Motor Init Success. Ptr: %p\r\n", test_motor);

    uint32_t tick = 0;
    uint32_t stage_timer = 0;
    Test_Stage_e current_stage = STAGE_SPEED_SINE;

    // 预备位置环参数 (供后面切换用)
    Djimotor_controller_init_t pos_conf = init_conf.motor_controller_init;
    pos_conf.close_loop = ANGLE_LOOP;
    pos_conf.angle_pid.kp = 12.0f;
    pos_conf.angle_pid.max_out = 500.0f; // 限制最大转速
    pos_conf.angle_pid.deadband = 0.2f;

    printf("\r\n>>> Stage 1: %s <<<\r\n", Stage_Names[current_stage]);

    for(;;)
    {
        // --- 状态机切换逻辑 (每 5 秒切一次) ---
        if (stage_timer > 5000) {
            stage_timer = 0;
            current_stage++;

            if (current_stage < STAGE_END) {
                printf("\r\n>>> Switch to Stage %d: %s <<<\r\n", current_stage, Stage_Names[current_stage]);

                // 阶段初始化
                if (current_stage == STAGE_POS_STEP) {
                    // 切到位置环
                    Djimotor_change_controller(test_motor, pos_conf);
                    // 将当前角度设为目标，防止突变
                    Djimotor_measure_t m = Djimotor_get_measure(test_motor);
                    Djimotor_set_target(test_motor, m.total_angle);
                }
                else if (current_stage == STAGE_TIMEOUT) {
                    printf("   -> Simulating app freeze (stop calling set_target)...\r\n");
                }
                else if (current_stage == STAGE_RECOVERY) {
                    // 恢复速度环
                    Djimotor_change_controller(test_motor, init_conf.motor_controller_init);
                    printf("   -> Recovering control...\r\n");
                }
            } else {
                // 测试结束，从头开始
                current_stage = STAGE_SPEED_SINE;
                Djimotor_change_controller(test_motor, init_conf.motor_controller_init);
                printf("\r\n>>> Loop Restart <<<\r\n");
            }
        }

        // --- 业务逻辑 ---

        if (current_stage == STAGE_SPEED_SINE) {
            // 速度环：正弦波 +/- 50rpm
            float target = 50.0f * sinf(tick * 0.002f);
            Djimotor_set_target(test_motor, target);
        }
        else if (current_stage == STAGE_POS_STEP) {
            // 位置环：0 -> 360 -> 720 -> 0 (多圈测试)
            // 每 1.25s 变一次
            float target = 0;
            if (stage_timer < 1250) target = 0;
            else if (stage_timer < 2500) target = 360;
            else if (stage_timer < 3750) target = 720;
            else target = 0;

            Djimotor_set_target(test_motor, target);
        }
        else if (current_stage == STAGE_TIMEOUT) {
            // 模拟死机：**故意不调用** Djimotor_set_target
            // 预期：在 100ms 后，电机应自动停止 (Current=0)
        }
        else if (current_stage == STAGE_RECOVERY) {
            // 恢复控制
            Djimotor_set_target(test_motor, 30.0f); // 恒定低速转
        }

        // --- 底层发送 (模拟 1kHz 任务) ---
        // 在你的实际工程中，这行代码在 motor_task.c 里
      //  Djimotor_control_all();

        // --- 打印反馈 (每 100ms) ---
        if (tick % 100 == 0) {
            Djimotor_measure_t m = Djimotor_get_measure(test_motor);
            Djimotor_status_e status = Djimotor_get_status(test_motor);

            // 打印: 阶段 | 状态(1=En, 0=Stop) | 目标 | 实际角度 | 实际速度 | 输出电流
            printf("S:%d | St:%d | Tgt:%.1f | Ang:%.1f | Spd:%.1f | Out:%d\r\n",
                   current_stage,
                   status == MOTOR_ENABLED,
                   test_motor->motor_pid.pid_target,
                   m.total_angle,
                   m.angular_velocity,
                   test_motor->motor_measure.real_current);
        }

        tick++;
        stage_timer++;
        osDelay(1); // 1000Hz
    }
}