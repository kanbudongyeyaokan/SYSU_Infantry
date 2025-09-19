/**
 * @file    chassis_motor_integration_test.c
 * @brief   底盘电机集成测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-19
 * @version 1.0
 * 
 * @note    用于测试底盘->话题数据->电机驱动的完整数据流框架
 */

#include "chassis_motor_integration_test.h"
#include "motor_task.h"
#include "Chassis_task.h"
#include "message_center.h"
#include "decision_making.h"
#include "motor_chassis_interface.h"
#include "chassis.h"
#include "dji_motor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <math.h>

// 测试阶段枚举
typedef enum {
    TEST_INIT = 0,              // 初始化测试
    TEST_STATIC_VALUES,         // 静态值测试
    TEST_DYNAMIC_MOVEMENT,      // 动态运动测试
    TEST_MODE_SWITCHING,        // 模式切换测试
    TEST_KINEMATICS,           // 运动学解算测试
    TEST_COMPLETED             // 测试完成
} test_phase_e;

// 测试状态结构体
typedef struct {
    test_phase_e current_phase;
    uint32_t phase_counter;
    uint32_t test_round;
    uint8_t all_tests_passed;
    uint8_t current_test_passed;
} test_state_t;

// 全局测试状态
static test_state_t g_test_state = {0};

// 测试用的发布者（模拟决策层）
static Publisher_t *chassis_cmd_pub = NULL;
static Subscriber_t *chassis_feedback_sub = NULL;

// 测试任务句柄
static osThreadId chassis_test_task_handle = NULL;
static osThreadId motor_test_task_handle = NULL;

// UART实例用于日志输出
static Uart_instance_t *test_uart = NULL;

/**
 * @brief 打印测试日志
 */
#define TEST_LOG(fmt, ...) \
    do { \
        if (test_uart) { \
            Uart_printf(test_uart, "[CHASSIS-MOTOR-TEST] " fmt "\r\n", ##__VA_ARGS__); \
        } \
        printf("[CHASSIS-MOTOR-TEST] " fmt "\r\n", ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 发送底盘控制指令
 */
static void Send_chassis_command(float vx, float vy, float wz, chassis_mode_e mode)
{
    if (chassis_cmd_pub == NULL) {
        TEST_LOG("ERROR: chassis_cmd_pub is NULL");
        return;
    }
    
    Chassis_cmd_send_t cmd = {
        .vx = vx,
        .vy = vy,
        .wz = wz,
        .offset_angle = 0.0f,
        .chassis_mode = mode
    };
    
    uint8_t result = Pub_push_message(chassis_cmd_pub, &cmd);
    TEST_LOG("Sent chassis command: vx=%.2f, vy=%.2f, wz=%.2f, mode=%d -> %d subscribers", 
             vx, vy, wz, mode, result);
}

/**
 * @brief 检查底盘反馈
 */
static uint8_t Check_chassis_feedback(float expected_wz)
{
    if (chassis_feedback_sub == NULL) {
        TEST_LOG("ERROR: chassis_feedback_sub is NULL");
        return 0;
    }
    
    Chassis_feedback_info_t feedback;
    if (Sub_get_message(chassis_feedback_sub, &feedback)) {
        TEST_LOG("Received chassis feedback: wz=%.2f (expected: %.2f)", feedback.chassis_wz, expected_wz);
        
        // 检查反馈值是否符合预期（允许小的误差）
        float error = fabsf(feedback.chassis_wz - expected_wz);
        if (error < 0.1f) {
            TEST_LOG("✓ Chassis feedback validation passed");
            return 1;
        } else {
            TEST_LOG("✗ Chassis feedback validation failed, error=%.3f", error);
            return 0;
        }
    } else {
        TEST_LOG("✗ No chassis feedback received");
        return 0;
    }
}

/**
 * @brief 检查电机速度设置
 */
static uint8_t Check_motor_speeds(void)
{
    float motor_speeds[4];
    if (Motor_get_chassis_speeds(motor_speeds)) {
        TEST_LOG("Current motor speeds: [%.1f, %.1f, %.1f, %.1f]", 
                 motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
        
        // 检查是否有电机速度设置（不为全零）
        uint8_t has_movement = 0;
        for (int i = 0; i < 4; i++) {
            if (fabsf(motor_speeds[i]) > 0.1f) {
                has_movement = 1;
                break;
            }
        }
        
        if (has_movement) {
            TEST_LOG("✓ Motor speeds validation passed");
            return 1;
        } else {
            TEST_LOG("✗ All motor speeds are zero");
            return 0;
        }
    } else {
        TEST_LOG("✗ No motor speed update available");
        return 0;
    }
}

/**
 * @brief 检查电机模式设置
 */
static uint8_t Check_motor_mode(chassis_mode_e expected_mode)
{
    chassis_mode_e current_mode;
    if (Motor_get_chassis_config(&current_mode)) {
        TEST_LOG("Current motor mode: %d (expected: %d)", current_mode, expected_mode);
        
        if (current_mode == expected_mode) {
            TEST_LOG("✓ Motor mode validation passed");
            return 1;
        } else {
            TEST_LOG("✗ Motor mode validation failed");
            return 0;
        }
    } else {
        TEST_LOG("✗ No motor mode update available");
        return 0;
    }
}

/**
 * @brief 测试阶段1：初始化测试
 */
static void Test_phase_init(void)
{
    TEST_LOG("=== Phase 1: Initialization Test ===");
    
    // 检查消息中心注册
    if (chassis_cmd_pub && chassis_feedback_sub) {
        TEST_LOG("✓ Message center registration successful");
        g_test_state.current_test_passed = 1;
    } else {
        TEST_LOG("✗ Message center registration failed");
        g_test_state.current_test_passed = 0;
    }
    
    // 发送停止指令确保初始状态
    Send_chassis_command(0.0f, 0.0f, 0.0f, CHASSIS_ZERO_FORCE);
}

/**
 * @brief 测试阶段2：静态值测试
 */
static void Test_phase_static_values(void)
{
    TEST_LOG("=== Phase 2: Static Values Test ===");
    
    uint8_t test_passed = 1;
    
    // 测试不同的静态指令
    static const struct {
        float vx, vy, wz;
        chassis_mode_e mode;
        const char* description;
    } test_cases[] = {
        {1.0f, 0.0f, 0.0f, CHASSIS_NO_FOLLOW, "Forward movement"},
        {0.0f, 1.0f, 0.0f, CHASSIS_NO_FOLLOW, "Left movement"},
        {0.0f, 0.0f, 1.0f, CHASSIS_ROTATE, "Rotation"},
        {0.5f, 0.5f, 0.5f, CHASSIS_FOLLOW_GIMBAL, "Combined movement"}
    };
    
    int test_case = g_test_state.phase_counter % (sizeof(test_cases) / sizeof(test_cases[0]));
    
    TEST_LOG("Testing: %s", test_cases[test_case].description);
    Send_chassis_command(test_cases[test_case].vx, test_cases[test_case].vy, 
                        test_cases[test_case].wz, test_cases[test_case].mode);
    
    // 等待数据传播
    osDelay(50);
    
    // 检查反馈
    if (!Check_chassis_feedback(test_cases[test_case].wz)) {
        test_passed = 0;
    }
    
    // 检查电机速度
    if (!Check_motor_speeds()) {
        test_passed = 0;
    }
    
    // 检查电机模式
    if (!Check_motor_mode(test_cases[test_case].mode)) {
        test_passed = 0;
    }
    
    g_test_state.current_test_passed = test_passed;
}

/**
 * @brief 测试阶段3：动态运动测试
 */
static void Test_phase_dynamic_movement(void)
{
    TEST_LOG("=== Phase 3: Dynamic Movement Test ===");
    
    // 生成正弦波运动
    float time = g_test_state.phase_counter * 0.1f;
    float vx = sinf(time) * 0.5f;
    float vy = cosf(time) * 0.5f;
    float wz = sinf(time * 0.5f) * 0.3f;
    
    TEST_LOG("Dynamic movement: vx=%.2f, vy=%.2f, wz=%.2f", vx, vy, wz);
    Send_chassis_command(vx, vy, wz, CHASSIS_NO_FOLLOW);
    
    // 等待数据传播
    osDelay(50);
    
    uint8_t test_passed = 1;
    
    // 检查反馈
    if (!Check_chassis_feedback(wz)) {
        test_passed = 0;
    }
    
    // 检查电机速度
    if (!Check_motor_speeds()) {
        test_passed = 0;
    }
    
    g_test_state.current_test_passed = test_passed;
}

/**
 * @brief 测试阶段4：模式切换测试
 */
static void Test_phase_mode_switching(void)
{
    TEST_LOG("=== Phase 4: Mode Switching Test ===");
    
    static const chassis_mode_e modes[] = {
        CHASSIS_ZERO_FORCE,
        CHASSIS_NO_FOLLOW,
        CHASSIS_FOLLOW_GIMBAL,
        CHASSIS_ROTATE
    };
    
    static const char* mode_names[] = {
        "ZERO_FORCE",
        "NO_FOLLOW",
        "FOLLOW_GIMBAL",
        "ROTATE"
    };
    
    int mode_index = g_test_state.phase_counter % (sizeof(modes) / sizeof(modes[0]));
    chassis_mode_e current_mode = modes[mode_index];
    
    TEST_LOG("Testing mode: %s", mode_names[mode_index]);
    Send_chassis_command(0.5f, 0.5f, 0.2f, current_mode);
    
    // 等待数据传播
    osDelay(50);
    
    uint8_t test_passed = 1;
    
    // 检查模式设置
    if (!Check_motor_mode(current_mode)) {
        test_passed = 0;
    }
    
    // 除了零力矩模式，其他模式都应该有电机运动
    if (current_mode != CHASSIS_ZERO_FORCE) {
        if (!Check_motor_speeds()) {
            test_passed = 0;
        }
    }
    
    g_test_state.current_test_passed = test_passed;
}

/**
 * @brief 测试阶段5：运动学解算验证
 */
static void Test_phase_kinematics(void)
{
    TEST_LOG("=== Phase 5: Kinematics Verification Test ===");
    
    // 测试特定的运动学场景
    static const struct {
        float vx, vy, wz;
        const char* description;
        float expected_speeds[4]; // 预期的四个电机速度（简化验证）
    } kinematics_tests[] = {
        {1.0f, 0.0f, 0.0f, "Pure forward", {0, 1, 0, -1}},  // 简化的预期值
        {0.0f, 1.0f, 0.0f, "Pure left", {1, 0, -1, 0}},
        {0.0f, 0.0f, 1.0f, "Pure rotation", {-1, -1, -1, -1}}
    };
    
    int test_case = g_test_state.phase_counter % (sizeof(kinematics_tests) / sizeof(kinematics_tests[0]));
    
    TEST_LOG("Testing kinematics: %s", kinematics_tests[test_case].description);
    Send_chassis_command(kinematics_tests[test_case].vx, kinematics_tests[test_case].vy, 
                        kinematics_tests[test_case].wz, CHASSIS_NO_FOLLOW);
    
    // 等待数据传播
    osDelay(50);
    
    uint8_t test_passed = 1;
    
    // 检查电机速度方向性（简化检查）
    float motor_speeds[4];
    if (Motor_get_chassis_speeds(motor_speeds)) {
        TEST_LOG("Actual motor speeds: [%.2f, %.2f, %.2f, %.2f]", 
                 motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
        
        // 简化验证：检查运动方向的一致性
        uint8_t has_expected_pattern = 1;
        for (int i = 0; i < 4; i++) {
            // 检查符号是否一致（允许零值）
            if (kinematics_tests[test_case].expected_speeds[i] != 0) {
                float expected_sign = kinematics_tests[test_case].expected_speeds[i] > 0 ? 1.0f : -1.0f;
                float actual_sign = motor_speeds[i] > 0 ? 1.0f : -1.0f;
                
                if (fabsf(motor_speeds[i]) > 0.1f && expected_sign != actual_sign) {
                    has_expected_pattern = 0;
                    break;
                }
            }
        }
        
        if (has_expected_pattern) {
            TEST_LOG("✓ Kinematics pattern validation passed");
        } else {
            TEST_LOG("✗ Kinematics pattern validation failed");
            test_passed = 0;
        }
    } else {
        test_passed = 0;
    }
    
    g_test_state.current_test_passed = test_passed;
}

/**
 * @brief 执行当前测试阶段
 */
static void Execute_current_test_phase(void)
{
    g_test_state.current_test_passed = 0; // 重置测试结果
    
    switch (g_test_state.current_phase) {
        case TEST_INIT:
            Test_phase_init();
            break;
        case TEST_STATIC_VALUES:
            Test_phase_static_values();
            break;
        case TEST_DYNAMIC_MOVEMENT:
            Test_phase_dynamic_movement();
            break;
        case TEST_MODE_SWITCHING:
            Test_phase_mode_switching();
            break;
        case TEST_KINEMATICS:
            Test_phase_kinematics();
            break;
        case TEST_COMPLETED:
            TEST_LOG("=== All Tests Completed ===");
            break;
    }
    
    // 更新测试状态
    if (g_test_state.current_test_passed) {
        TEST_LOG("✓ Phase %d test passed", g_test_state.current_phase);
    } else {
        TEST_LOG("✗ Phase %d test failed", g_test_state.current_phase);
        g_test_state.all_tests_passed = 0;
    }
}

/**
 * @brief 进入下一个测试阶段
 */
static void Advance_to_next_phase(void)
{
    g_test_state.phase_counter++;
    
    // 每个阶段运行一定次数后进入下一阶段
    const uint32_t phase_iterations[] = {1, 4, 20, 4, 3}; // 各阶段运行次数
    uint32_t current_phase_iterations = 1;
    
    if (g_test_state.current_phase < sizeof(phase_iterations) / sizeof(phase_iterations[0])) {
        current_phase_iterations = phase_iterations[g_test_state.current_phase];
    }
    
    if (g_test_state.phase_counter >= current_phase_iterations) {
        g_test_state.current_phase++;
        g_test_state.phase_counter = 0;
        
        if (g_test_state.current_phase >= TEST_COMPLETED) {
            // 测试轮次完成，开始新轮次
            g_test_state.test_round++;
            g_test_state.current_phase = TEST_INIT;
            g_test_state.all_tests_passed = 1; // 重置测试状态
            
            TEST_LOG("========== Test Round %lu Completed ==========", g_test_state.test_round);
            osDelay(2000); // 轮次间暂停
        }
    }
}

/**
 * @brief 底盘电机集成测试任务函数
 * @param argument 任务参数（未使用）
 */
void Chassis_motor_integration_test_task(void const *argument)
{
    // 初始化UART用于日志输出
    test_uart = Uart_register(&huart1, NULL);
    
    // 等待系统初始化
    osDelay(3000);
    
    TEST_LOG("========================================");
    TEST_LOG("  Chassis-Motor Integration Test Started");
    TEST_LOG("========================================");
    
    // 注册消息中心（模拟决策层）
    chassis_cmd_pub = Pub_register("chassis_cmd", sizeof(Chassis_cmd_send_t));
    chassis_feedback_sub = Sub_register("chassis_feedback", sizeof(Chassis_feedback_info_t));
    
    // 创建底盘任务和电机任务
    osThreadDef(chassis_test_task, Chassis_control_task, osPriorityNormal, 0, 1024);
    chassis_test_task_handle = osThreadCreate(osThread(chassis_test_task), NULL);
    
    osThreadDef(motor_test_task, Motor_control_task, osPriorityHigh, 0, 1024);
    motor_test_task_handle = osThreadCreate(osThread(motor_test_task), NULL);
    
    if (chassis_test_task_handle && motor_test_task_handle) {
        TEST_LOG("✓ Chassis and Motor tasks created successfully");
    } else {
        TEST_LOG("✗ Failed to create tasks");
        return;
    }
    
    // 等待任务启动
    osDelay(1000);
    
    // 初始化测试状态
    g_test_state.current_phase = TEST_INIT;
    g_test_state.phase_counter = 0;
    g_test_state.test_round = 1;
    g_test_state.all_tests_passed = 1;
    
    // 主测试循环
    for (;;)
    {
        Execute_current_test_phase();
        
        // 测试间隔
        osDelay(500);
        
        Advance_to_next_phase();
    }
}