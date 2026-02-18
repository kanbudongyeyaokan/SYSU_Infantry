/**
 * @file    dji_motor.c
 * @brief   大疆电机驱动实现 (修正回调函数名)
 * @author  SYSU电控组
 */

#include "dji_motor.h"
#include "bsp_can.h"
#include "stdlib.h"
#include "bsp_usart.h"
#include "algorithm_pid.h"
#include "main.h"
#include "stdio.h"
#include "bsp_wdg.h" // 引入看门狗库

// 电机实例数组
static Djimotor_device_t *motor_instances[MAX_MOTOR_COUNT] = {NULL};
static uint8_t motor_count = 0;

// 发送缓冲区 [0]:CAN1 [1]:CAN2; 每个CAN口有3个ID (0:1FF, 1:200, 2:2FF)
static uint8_t motor_can_buffer[2][3][8];
// 发送标志位
static uint8_t motor_can_flag[2][3] = {0};

// 预定义发送用的 CAN 控制器句柄
static Can_controller_t can1_tx_handlers[3];
static Can_controller_t can2_tx_handlers[3];

extern Uart_instance_t* uart_instance;

/* --- 内部辅助函数 --- */

/**
 * @brief 电机离线回调函数 (由看门狗触发)
 * @note  [修正] 统一函数名为 Motor_Offline_Callback
 */
static void Motor_Offline_Callback(void *device)
{
    Djimotor_device_t *motor = (Djimotor_device_t *)device;

    // 清空速度和电流反馈，防止 PID 积分暴涨
    motor->motor_measure.angular_velocity = 0;
    motor->motor_measure.real_current = 0;

    // 将状态设为 STOP，停止计算输出
    motor->motor_status = MOTOR_STOP;
}

// 解译电机反馈数据
static void Decode_djimotor(Can_controller_t* can_dev, void *context)
{
    Djimotor_device_t *motor = (Djimotor_device_t*)context;
    uint8_t *rxbuff = can_dev->rx_buffer;
    Djimotor_measure_t *measure = &(motor->motor_measure);

    // 1. 喂狗
    if (motor->wdg != NULL) {
        Watchdog_feed(motor->wdg);
    }

    // 2. 保存旧值
    measure->last_ecd = measure->current_ecd;

    // 3. 解析新值
    measure->current_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angular_velocity = (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3])); // rpm
    measure->real_current = ((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->motor_temperature = rxbuff[6];

    // 4. 多圈角度解算
    int32_t diff = measure->current_ecd - measure->last_ecd;
    if (diff > 4096) {
        diff -= 8192;
        measure->total_round--;
    } else if (diff < -4096) {
        diff += 8192;
        measure->total_round++;
    }

    measure->current_angle = measure->current_ecd * ECD_ANGLE_COEF_DJI; // 0-360
    measure->total_angle += diff * ECD_ANGLE_COEF_DJI;
}

// 初始化全局发送句柄
static void Init_Global_Tx_Handlers(void)
{
    static uint8_t is_init = 0;
    if (is_init) return;

    for (int i = 0; i < 3; i++) {
        can1_tx_handlers[i].can_handle = &hcan1;
        can1_tx_handlers[i].tx_config.IDE = CAN_ID_STD;
        can1_tx_handlers[i].tx_config.RTR = CAN_RTR_DATA;
        can1_tx_handlers[i].tx_config.DLC = 8;

        can2_tx_handlers[i].can_handle = &hcan2;
        can2_tx_handlers[i].tx_config.IDE = CAN_ID_STD;
        can2_tx_handlers[i].tx_config.RTR = CAN_RTR_DATA;
        can2_tx_handlers[i].tx_config.DLC = 8;
    }
    can1_tx_handlers[0].tx_config.StdId = 0x1FF;
    can1_tx_handlers[1].tx_config.StdId = 0x200;
    can1_tx_handlers[2].tx_config.StdId = 0x2FF;

    can2_tx_handlers[0].tx_config.StdId = 0x1FF;
    can2_tx_handlers[1].tx_config.StdId = 0x200;
    can2_tx_handlers[2].tx_config.StdId = 0x2FF;

    is_init = 1;
}

// --- 公共接口 ---

Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config)
{
    if (motor_count >= MAX_MOTOR_COUNT) return NULL;

    Djimotor_device_t *motor = (Djimotor_device_t *)malloc(sizeof(Djimotor_device_t));
    if (motor == NULL) return NULL;
    // 初始化全局发送句柄
    Init_Global_Tx_Handlers();

    // 1. 基础配置
    memset(motor, 0, sizeof(Djimotor_device_t));
    strncpy(motor->motor_name, config->motor_name, sizeof(motor->motor_name) - 1);
    motor->motor_type = config->motor_type;
    motor->motor_status = config->motor_status;
    motor->deadzone_compensation = config->deadzone_compensation;

    // PID 初始化
    motor->motor_pid.close_loop = config->motor_controller_init.close_loop;
    motor->motor_pid.angle_source = config->motor_controller_init.angle_source;
    motor->motor_pid.speed_source = config->motor_controller_init.speed_source;
    motor->motor_pid.other_angle_feedback_ptr = config->motor_controller_init.other_angle_feedback_ptr;
    motor->motor_pid.other_speed_feedback_ptr = config->motor_controller_init.other_speed_feedback_ptr;

    Pid_init(&motor->motor_pid.current_pid, &config->motor_controller_init.current_pid);
    Pid_init(&motor->motor_pid.angle_pid, &config->motor_controller_init.angle_pid);
    Pid_init(&motor->motor_pid.speed_pid, &config->motor_controller_init.speed_pid);

    // 2. CAN 注册
    Can_init_t can_config;
    memset(&can_config, 0, sizeof(Can_init_t));
    can_config.can_handle = config->can_init.can_handle;
    can_config.can_id = config->can_init.can_id;
    can_config.tx_id  = config->can_init.tx_id;
    can_config.rx_id  = config->can_init.rx_id;
    can_config.receive_callback = Decode_djimotor;
    can_config.context = motor;

    motor->can_controller = Can_device_init(&can_config);
    if (motor->can_controller == NULL) {
        free(motor);
        return NULL;
    }

    // 3. 看门狗注册
    Watchdog_init_t wdg_conf;
    wdg_conf.owner_id = motor;
    wdg_conf.reload_count = 50;
    // [修正] 这里赋值正确的函数名
    wdg_conf.callback = Motor_Offline_Callback;
    motor->wdg = Watchdog_register(&wdg_conf);

    motor_instances[motor_count++] = motor;
    return motor;
}

void Djimotor_change_controller(Djimotor_device_t *motor, Djimotor_controller_init_t ctrl_params)
{
    if (!motor) return;

    // 1. 【进入临界区】关闭全局中断，防止被打断
    // 具体的函数名取决于你的 RTOS 或 HAL 库，例如：
    __disable_irq();

    motor->motor_pid.close_loop = ctrl_params.close_loop;
    motor->motor_pid.angle_source = ctrl_params.angle_source;
    motor->motor_pid.speed_source = ctrl_params.speed_source;
    motor->motor_pid.other_angle_feedback_ptr = ctrl_params.other_angle_feedback_ptr;
    motor->motor_pid.other_speed_feedback_ptr = ctrl_params.other_speed_feedback_ptr;
    Pid_init(&(motor->motor_pid.speed_pid),&(ctrl_params.speed_pid));
    Pid_init(&(motor->motor_pid.angle_pid),&(ctrl_params.angle_pid));
    Pid_init(&(motor->motor_pid.current_pid),&(ctrl_params.current_pid));
    motor->motor_pid.speed_feedforward = 0.0f;

    // 2. 【退出临界区】恢复全局中断
    __enable_irq();
}

void Djimotor_set_target(Djimotor_device_t *motor, float target) {
    if (motor) motor->motor_pid.pid_target = target;
}

Djimotor_status_e Djimotor_get_status(Djimotor_device_t *motor) {
    if (motor == NULL) return MOTOR_STOP;
    if (motor->wdg && !Watchdog_is_online(motor->wdg)) {
        return MOTOR_STOP;
    }
    return motor->motor_status;
}

Djimotor_measure_t Djimotor_get_measure(Djimotor_device_t *motor) {
    Djimotor_measure_t measure = {0};
    if (motor != NULL) {
        measure = motor->motor_measure;
    }
    return measure;
}

void Djimotor_set_status(Djimotor_device_t *motor, Djimotor_status_e status) {
    if (motor) motor->motor_status = status;
}

void Djimotor_set_deadzone(Djimotor_device_t *motor, int16_t deadzone) {
    if (motor) motor->deadzone_compensation = deadzone;
}

int16_t Djimotor_get_deadzone(Djimotor_device_t *motor) {
    if (motor) return motor->deadzone_compensation;
    return 0;
}

// 计算控制输出并填充到缓冲区
static void Calculate_Motor_Output(Djimotor_device_t *motor) {

    // 离线检查
    if (motor->wdg && !Watchdog_is_online(motor->wdg)) {
        motor->motor_status = MOTOR_STOP;
    }

    float output = 0.0f;
    Djimotor_measure_t *measure = &motor->motor_measure;
    //获取电机角度反馈值
    float angle_feedback = measure->total_angle;
    if (motor->motor_pid.angle_source == OTHER_FEEDBACK && motor->motor_pid.other_angle_feedback_ptr) {
        angle_feedback = *(motor->motor_pid.other_angle_feedback_ptr);
    }
    //获取电机速度反馈值
    float speed_feedback = measure->angular_velocity;
    if (motor->motor_pid.speed_source == OTHER_FEEDBACK && motor->motor_pid.other_speed_feedback_ptr) {
        speed_feedback = *(motor->motor_pid.other_speed_feedback_ptr);
    }
    //获取电机电流反馈值
    float current_feedback = measure->real_current;

    if (motor->motor_status == MOTOR_STOP) {
        output = 0.0f;
        Pid_reset(&motor->motor_pid.current_pid);
        Pid_reset(&motor->motor_pid.speed_pid);
        Pid_reset(&motor->motor_pid.angle_pid);
    } else {
        switch (motor->motor_pid.close_loop) {
            case OPEN_LOOP:
                output = motor->motor_pid.pid_target;
                break;
            case CURRENT_LOOP:
                output = Pid_calculate(&motor->motor_pid.current_pid, current_feedback, motor->motor_pid.pid_target);
                break;
            case SPEED_LOOP:
                output = Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback, motor->motor_pid.pid_target);
                break;
            case ANGLE_LOOP:
                output = Pid_calculate(&motor->motor_pid.angle_pid, angle_feedback, motor->motor_pid.pid_target);
                break;
            case SPEED_AND_CURRENT_LOOP: {
                float current_target = Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback, motor->motor_pid.pid_target);
                output = Pid_calculate(&motor->motor_pid.current_pid, current_feedback, current_target);
                break;
            }
            case ANGLE_AND_SPEED_LOOP: {
                float speed_target = Pid_calculate(&motor->motor_pid.angle_pid, angle_feedback, motor->motor_pid.pid_target)
                                     + motor->motor_pid.speed_feedforward; // 速度前馈补偿

                output = Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback, speed_target);
                                    // 力矩前馈可以在这里添加
                break;
            }
            case ANGLE_AND_CURRENT_LOOP: {
                // 力位混控：角度环输出直接作为电流环输入
                float current_target = Pid_calculate(&motor->motor_pid.angle_pid, 
                    angle_feedback, 
                    motor->motor_pid.pid_target);
                output = Pid_calculate(&motor->motor_pid.current_pid, current_feedback, current_target);
                break;
            }
            default:
                output = 0.0f;
                break;
        }
    }

    int16_t current_val = (int16_t)output;

    CAN_HandleTypeDef *hcan = motor->can_controller->can_handle;
    uint32_t tx_id = motor->can_controller->tx_id;
    uint32_t can_id = motor->can_controller->can_id;

    uint8_t can_idx = (hcan == &hcan1) ? 0 : 1;
    uint8_t group_idx = 0;

    if (can_id == 0x1FF) group_idx = 0;
    else if (can_id == 0x200) group_idx = 1;
    else if (can_id == 0x2FF) group_idx = 2;
    else return;

    uint8_t buffer_offset = 0;
    if (motor->motor_type == GM6020) {
        if (tx_id <= 4) buffer_offset = (tx_id - 1) * 2;
        else            buffer_offset = (tx_id - 5) * 2;
    } else {
        if (tx_id <= 4) buffer_offset = (tx_id - 1) * 2;
        else            buffer_offset = (tx_id - 5) * 2;
    }

    motor_can_buffer[can_idx][group_idx][buffer_offset]     = (uint8_t)(current_val >> 8);
    motor_can_buffer[can_idx][group_idx][buffer_offset + 1] = (uint8_t)(current_val);

    motor_can_flag[can_idx][group_idx] = 1;
}

void Djimotor_control_all(void) {
    memset(motor_can_flag, 0, sizeof(motor_can_flag));

    for (uint8_t i = 0; i < motor_count; i++) {
        Calculate_Motor_Output(motor_instances[i]);
    }

    if (motor_can_flag[0][0]) Can_send_data(&can1_tx_handlers[0], motor_can_buffer[0][0]);
    if (motor_can_flag[0][1]) Can_send_data(&can1_tx_handlers[1], motor_can_buffer[0][1]);
    if (motor_can_flag[0][2]) Can_send_data(&can1_tx_handlers[2], motor_can_buffer[0][2]);

    if (motor_can_flag[1][0]) Can_send_data(&can2_tx_handlers[0], motor_can_buffer[1][0]);
    if (motor_can_flag[1][1]) Can_send_data(&can2_tx_handlers[1], motor_can_buffer[1][1]);
    if (motor_can_flag[1][2]) Can_send_data(&can2_tx_handlers[2], motor_can_buffer[1][2]);
}