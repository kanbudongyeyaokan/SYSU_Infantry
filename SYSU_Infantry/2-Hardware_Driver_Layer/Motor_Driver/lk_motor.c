#include "lk_motor.h"
#include <stdlib.h>
#include <stdio.h>
#include "main.h" // 必须包含 main.h 以使用 __NOP() 或 HAL 库定义

// 领控命令宏定义
#define LK_CMD_READ_STATUS1     0x9A
#define LK_CMD_READ_STATUS2     0x9C
#define LK_CMD_READ_STATUS3     0x9D
#define LK_CMD_MOTOR_OFF        0x80
#define LK_CMD_MOTOR_RUN        0x88
#define LK_CMD_MOTOR_STOP       0x81
#define LK_CMD_OPEN_LOOP        0xA0
#define LK_CMD_TORQUE_LOOP      0xA1
#define LK_CMD_SPEED_LOOP       0xA2
#define LK_CMD_POS_LOOP_1       0xA3
#define LK_CMD_POS_LOOP_2       0xA4
#define LK_CMD_POS_LOOP_3       0xA5
#define LK_CMD_POS_LOOP_4       0xA6

static Lkmotor_device_t *lk_motor_instances[MAX_LK_MOTOR_COUNT] = {NULL};
static uint8_t lk_motor_count = 0;

// 辅助函数：写入 Buffer
static void Write_int16_to_buf(uint8_t *buf, int16_t data) {
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
}

static void Write_int32_to_buf(uint8_t *buf, int32_t data) {
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
    buf[2] = (uint8_t)((data >> 16) & 0xFF);
    buf[3] = (uint8_t)((data >> 24) & 0xFF);
}

// CAN 回调函数
static void Decode_lkmotor(Can_controller_t* can_dev, void *context)
{
    Lkmotor_device_t *motor = (Lkmotor_device_t*)context;
    uint8_t *rx_data = can_dev->rx_buffer;
    Lkmotor_measure_t *measure = &motor->motor_measure;

    measure->temperature = (int8_t)rx_data[1];
    measure->current_torque = (int16_t)(rx_data[2] | (rx_data[3] << 8));
    measure->speed_dps = (int16_t)(rx_data[4] | (rx_data[5] << 8));
    measure->encoder_raw = (uint16_t)(rx_data[6] | (rx_data[7] << 8));

    measure->angular_velocity = (float)measure->speed_dps * LK_SPEED_TO_RPM;

    if (!measure->total_angle_initialized) {
        measure->last_encoder = measure->encoder_raw;
        measure->total_round = 0;
        measure->total_angle = (float)measure->encoder_raw * (360.0f / 65535.0f);
        measure->total_angle_initialized = true;
    } else {
        int32_t diff = (int32_t)measure->encoder_raw - (int32_t)measure->last_encoder;
        if (diff > 32768) {
            measure->total_round--;
        } else if (diff < -32768) {
            measure->total_round++;
        }
        measure->last_encoder = measure->encoder_raw;
        measure->total_angle = measure->total_round * 360.0f + (float)measure->encoder_raw * (360.0f / 65535.0f);
    }
}

// 初始化函数
Lkmotor_device_t *Lk_Motor_Init(Lkmotor_init_config_t *config)
{
    if (lk_motor_count >= MAX_LK_MOTOR_COUNT) return NULL;

    Lkmotor_device_t *motor = (Lkmotor_device_t *)malloc(sizeof(Lkmotor_device_t));
    if (motor == NULL) return NULL;
    memset(motor, 0, sizeof(Lkmotor_device_t));

    strncpy(motor->motor_name, config->motor_name, 16);
    motor->motor_type = config->motor_type;
    motor->motor_status = config->motor_status;

    motor->motor_controller.close_loop = config->motor_controller_init.close_loop;
    motor->motor_controller.max_speed = 3600;

    Can_init_t can_config;
    memcpy(&can_config, &config->can_init, sizeof(Can_init_t));

    motor->hardware_run_cmd_sent = false;
    // ID 映射: 发送 0x140+ID, 接收 0x180+ID (如果你的硬件回复异常, 请自行修改 rx_id)
    uint32_t motor_id = config->can_init.can_id;
    can_config.tx_id = motor_id;
    can_config.can_id = 0x140 + motor_id;
    can_config.rx_id = 0x180 + motor_id;
    can_config.receive_callback = Decode_lkmotor;
    can_config.context = motor;

    motor->can_controller = Can_device_init(&can_config);
    if (motor->can_controller == NULL) {
        free(motor);
        return NULL;
    }

    lk_motor_instances[lk_motor_count++] = motor;
    return motor;
}

void Lkmotor_set_target(Lkmotor_device_t *motor, float target)
{
    if (motor == NULL) return;
    motor->motor_controller.pid_target = target;
}

void Lkmotor_set_status(Lkmotor_device_t *motor, Lkmotor_status_e status)
{
    if (motor == NULL) return;
    motor->motor_status = status;
}

Lkmotor_measure_t Lkmotor_get_measure(Lkmotor_device_t *motor)
{
    Lkmotor_measure_t empty = {0};
    if (motor == NULL) return empty;
    return motor->motor_measure;
}

/* ================= 完美解决方案：分时发送版 ================= */
/* 文件位置: lk_motor.c */

void Lkmotor_control_all(void)
{
    // 定义静态变量，用于记录当前轮次 (0: 发送偶数索引电机, 1: 发送奇数索引电机)
    // 这样可以将 4 个电机的发送压力分摊到 2 个控制周期中
    static uint8_t send_group_toggle = 0;

    // 切换分组
    send_group_toggle = !send_group_toggle;

    for (int i = 0; i < lk_motor_count; i++) {

        // 【核心逻辑】分时发送过滤
        // 如果当前是第0组，只发 i=0, 2, 4... 的电机
        // 如果当前是第1组，只发 i=1, 3, 5... 的电机
        // 这样每次循环最多只发送总数的一半，4个电机变2个，完美避开 STM32 的 3 邮箱限制
        if ((i % 2) != send_group_toggle) {
            continue;
        }

        Lkmotor_device_t *motor = lk_motor_instances[i];
        if (motor == NULL || motor->can_controller == NULL) continue;

        uint8_t tx_data[8] = {0};

        // 1. 状态机逻辑 (保持不变)
        if (motor->motor_status == LK_MOTOR_STOP) {
            tx_data[0] = LK_CMD_MOTOR_OFF;
            motor->hardware_run_cmd_sent = false;
        }
        else if (motor->hardware_run_cmd_sent == false) {
            tx_data[0] = LK_CMD_MOTOR_RUN;
            motor->hardware_run_cmd_sent = true;
        }
        else {
            // 2. 正常闭环控制逻辑 (保持不变)
            float target = motor->motor_controller.pid_target;
            switch (motor->motor_controller.close_loop) {
                case LK_CURRENT_LOOP:
                    tx_data[0] = LK_CMD_TORQUE_LOOP;
                    Write_int16_to_buf(&tx_data[4], (int16_t)target);
                    break;
                case LK_SPEED_LOOP:
                case LK_SPEED_AND_CURRENT_LOOP:
                    tx_data[0] = LK_CMD_SPEED_LOOP;
                    Write_int16_to_buf(&tx_data[2], 2000);
                    int32_t speed_ctrl = (int32_t)(target * LK_RPM_TO_SPEED_CTRL);
                    Write_int32_to_buf(&tx_data[4], speed_ctrl);
                    break;
                case LK_ANGLE_LOOP:
                case LK_ANGLE_AND_SPEED_LOOP:
                    tx_data[0] = LK_CMD_POS_LOOP_2;
                    uint16_t max_spd_dps = (uint16_t)motor->motor_controller.max_speed;
                    if (max_spd_dps == 0) max_spd_dps = 360;
                    Write_int16_to_buf(&tx_data[2], max_spd_dps);
                    int32_t angle_ctrl = (int32_t)(target * 100.0f);
                    Write_int32_to_buf(&tx_data[4], angle_ctrl);
                    break;
                case LK_OPEN_LOOP:
                default:
                    tx_data[0] = LK_CMD_OPEN_LOOP;
                    Write_int16_to_buf(&tx_data[4], (int16_t)target);
                    break;
            }
        }

        // 3. 直接发送，不需要重试循环了！
        // 因为现在每次只发 2 个包，邮箱绝对够用，发了就走，速度最快
        Can_send_data(motor->can_controller, tx_data);
    }
}