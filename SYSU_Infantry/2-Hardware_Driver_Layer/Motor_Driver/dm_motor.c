/**
 * @file    dm_motor.c
 * @brief   达妙电机驱动实现 (MIT模式 + 位置速度模式)
 * @author  SYSU电控组
 * @note    基于MIT猎豹协议，支持参数独立配置
 */

#include "dm_motor.h"
#include "bsp_can.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "stdio.h"
#include "bsp_wdg.h"
#include <math.h>

/* ======================== 私有变量 ======================== */

// 电机实例数组
static DMmotor_device_t *dm_motor_instances[DM_MOTOR_MAX_COUNT] = {NULL};
static uint8_t dm_motor_count = 0;

/* ======================== 私有函数 ======================== */

/**
 * @brief 浮点数映射到无符号整数 (MIT协议编码)
 * @note  需要传入参数配置以支持不同量程的电机
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    // 限幅
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 无符号整数映射到浮点数 (MIT协议解码)
 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 发送MIT模式状态命令帧 (使能/失能/设零/清错)
 */
static void DMmotor_send_cmd(DMmotor_device_t *motor, DMmotor_cmd_e cmd)
{
    if (motor == NULL || motor->can_controller == NULL) return;
    
    uint8_t tx_buff[8];
    memset(tx_buff, 0xFF, 7);   // 前7字节填充0xFF
    tx_buff[7] = (uint8_t)cmd;  // 第8字节是命令
    
    Can_send_data(motor->can_controller, tx_buff);
}

/**
 * @brief 发送MIT控制帧 (核心控制逻辑)
 */
static void DMmotor_send_mit_frame(DMmotor_device_t *motor)
{
    if (motor == NULL || motor->can_controller == NULL) return;

    // 离线检查
    /*
    if (motor->wdg && !Watchdog_is_online(motor->wdg))
    {
         return; 
    }
    */
    // 获取参数指针
    DMmotor_params_t *p = &motor->params;
    
    // 动态范围 (注意: 最小值通常是最大值的相反数)
    float P_MAX = p->p_max; float P_MIN = -p->p_max;
    float V_MAX = p->v_max; float V_MIN = -p->v_max;
    float T_MAX = p->t_max; float T_MIN = -p->t_max;
    // KP, KD 暂时使用默认范围，也可扩展
    float KP_MAX = 500.0f; float KP_MIN = 0.0f;
    float KD_MAX = 5.0f;   float KD_MIN = 0.0f;

    if (motor->motor_status == DM_MOTOR_STOP)
    {
        // 构造零力矩帧
        uint16_t p_int = float_to_uint(0.0f, P_MIN, P_MAX, 16);
        uint16_t v_int = float_to_uint(0.0f, V_MIN, V_MAX, 12);
        uint16_t kp_int = float_to_uint(0.0f, KP_MIN, KP_MAX, 12);
        uint16_t kd_int = float_to_uint(0.0f, KD_MIN, KD_MAX, 12);
        uint16_t t_int = float_to_uint(0.0f, T_MIN, T_MAX, 12);
        
        uint8_t tx_buff[8];
        tx_buff[0] = (uint8_t)(p_int >> 8);
        tx_buff[1] = (uint8_t)(p_int & 0xFF);
        tx_buff[2] = (uint8_t)(v_int >> 4);
        tx_buff[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
        tx_buff[4] = (uint8_t)(kp_int & 0xFF);
        tx_buff[5] = (uint8_t)(kd_int >> 4);
        tx_buff[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
        tx_buff[7] = (uint8_t)(t_int & 0xFF);
        
        Can_send_data(motor->can_controller, tx_buff);
        return;
    }

    // 正常控制模式
    DMmotor_cmd_check_t *cmd = &motor->cmd_data;
    
    uint16_t p_u = float_to_uint(cmd->p_des, P_MIN, P_MAX, 16);
    uint16_t v_u = float_to_uint(cmd->v_des, V_MIN, V_MAX, 12);
    uint16_t kp_u = float_to_uint(cmd->kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_u = float_to_uint(cmd->kd, KD_MIN, KD_MAX, 12);
    uint16_t t_u = float_to_uint(cmd->t_ff, T_MIN, T_MAX, 12);
    
    // 拼包
    uint8_t tx_buff[8];
    tx_buff[0] = (uint8_t)(p_u >> 8);
    tx_buff[1] = (uint8_t)(p_u & 0xFF);
    tx_buff[2] = (uint8_t)(v_u >> 4);
    tx_buff[3] = (uint8_t)(((v_u & 0xF) << 4) | (kp_u >> 8));
    tx_buff[4] = (uint8_t)(kp_u & 0xFF);
    tx_buff[5] = (uint8_t)(kd_u >> 4);
    tx_buff[6] = (uint8_t)(((kd_u & 0xF) << 4) | (t_u >> 8));
    tx_buff[7] = (uint8_t)(t_u & 0xFF);
    
    Can_send_data(motor->can_controller, tx_buff);
}

/**
 * @brief 发送位置速度控制帧 (0x100 + ID)
 */
static void DMmotor_send_pos_vel_frame(DMmotor_device_t *motor)
{
    if (motor == NULL || motor->can_controller == NULL) return;

    // 离线检查：如果看门狗检测到离线且并非处于停止状态，则不发送
    if (motor->wdg && !Watchdog_is_online(motor->wdg) && motor->motor_status != DM_MOTOR_STOP) {
        return;
    }

    // 1. 获取基础 ID
    uint32_t base_id = motor->can_controller->can_id;
    uint32_t original_can_id = motor->can_controller->can_id;
    
    // 2. 临时修改发送 ID 为位置速度模式偏移 ID (0x100 + SlaveID)
    // 根据达妙协议，Pos-Vel 模式的 ID 偏移量为 0x100
    uint32_t target_id = 0x100 + base_id;
    
    motor->can_controller->can_id = target_id;
    motor->can_controller->tx_config.StdId = target_id;

    // 3. 准备数据
    float p_des = motor->cmd_data.pos_vel_p_des;
    float v_des = motor->cmd_data.pos_vel_v_des;

    uint8_t tx_buff[8];
    memcpy(&tx_buff[0], &p_des, 4);
    memcpy(&tx_buff[4], &v_des, 4);

    // 4. 发送 CAN 帧
    Can_send_data(motor->can_controller, tx_buff);

    // 5. 恢复 ID
    motor->can_controller->can_id = original_can_id;
    motor->can_controller->tx_config.StdId = original_can_id;
}


/**
 * @brief 电机离线回调函数 (由看门狗触发)
 */
static void DMmotor_offline_callback(void *device)
{
    DMmotor_device_t *motor = (DMmotor_device_t *)device;
    
    // 清空反馈
    motor->motor_measure.velocity = 0;
    motor->motor_measure.torque = 0;
    
    // 设置状态为停止
   // motor->motor_status = DM_MOTOR_STOP;
}

/**
 * @brief CAN接收回调函数
 */
static void Decode_DMmotor(Can_controller_t *can_dev, void *context)
{
    DMmotor_device_t *motor = (DMmotor_device_t *)context;
    uint8_t *rxbuff = can_dev->rx_buffer;
    DMmotor_measure_t *measure = &(motor->motor_measure);
    
    if (motor->wdg != NULL) Watchdog_feed(motor->wdg);
    
    measure->id = rxbuff[0] & 0x0F;
    measure->state = (rxbuff[0] >> 4) & 0x0F;
    measure->last_position = measure->position;
    
    // 获取参数指针
    DMmotor_params_t *p = &motor->params;
    float P_MAX = p->p_max; float P_MIN = -p->p_max;
    float V_MAX = p->v_max; float V_MIN = -p->v_max;
    float T_MAX = p->t_max; float T_MIN = -p->t_max;
    
    uint16_t pos_raw = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(pos_raw, P_MIN, P_MAX, 16);
    
    uint16_t vel_raw = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(vel_raw, V_MIN, V_MAX, 12);
    
    uint16_t torque_raw = (uint16_t)(((rxbuff[4] & 0x0F) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(torque_raw, T_MIN, T_MAX, 12);
    
    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    
    float pos_diff = measure->position - measure->last_position;
    if (pos_diff > (P_MAX - P_MIN) / 2.0f) measure->total_round--;
    else if (pos_diff < -(P_MAX - P_MIN) / 2.0f) measure->total_round++;
    measure->total_angle = (float)measure->total_round * (P_MAX - P_MIN) + measure->position;
}

/* ======================== 公共函数 ======================== */

DMmotor_device_t *DM_Motor_Init(DMmotor_init_config_t *config)
{
    if (config == NULL) return NULL;
    if (dm_motor_count >= DM_MOTOR_MAX_COUNT) return NULL;
    
    DMmotor_device_t *motor = (DMmotor_device_t *)malloc(sizeof(DMmotor_device_t));
    if (motor == NULL) return NULL;
    
    memset(motor, 0, sizeof(DMmotor_device_t));
    strncpy(motor->motor_name, config->motor_name, sizeof(motor->motor_name) - 1);
    motor->motor_type = config->motor_type;
    motor->motor_status = config->motor_status;
    motor->master_id = config->master_id;
    motor->mode = DM_CTRL_MIT; // 默认MIT
    
    // 复制电机参数
    // 如果config中没有设置参数(全0)，使用默认安全值
    if (config->params.p_max == 0.0f)
    {
        motor->params.p_max = 12.5f;
        motor->params.v_max = 30.0f;
        motor->params.t_max = 10.0f;
    }
    else
    {
        motor->params = config->params;
    }
    
    // 初始化CMD
    motor->cmd_data.p_des = 0.0f;
    motor->cmd_data.v_des = 0.0f;
    motor->cmd_data.kp = 0.0f;
    motor->cmd_data.kd = 0.0f;
    motor->cmd_data.t_ff = 0.0f;
    
    Can_init_t can_config;
    memset(&can_config, 0, sizeof(Can_init_t));
    can_config.can_handle = config->can_init.can_handle;
    can_config.can_id = config->can_init.can_id;
    can_config.tx_id = config->can_init.tx_id;
    can_config.rx_id = config->can_init.rx_id;
    can_config.receive_callback = Decode_DMmotor;
    can_config.context = motor;
    
    motor->can_controller = Can_device_init(&can_config);
    if (motor->can_controller == NULL)
    {
        free(motor);
        return NULL;
    }
    
    Watchdog_init_t wdg_conf;
    wdg_conf.owner_id = motor;
    wdg_conf.reload_count = 50;
    wdg_conf.callback = DMmotor_offline_callback;
    motor->wdg = Watchdog_register(&wdg_conf);
    
    DMmotor_send_cmd(motor, DM_CMD_ENABLE);
    dm_motor_instances[dm_motor_count++] = motor;
    
    return motor;
}

void DMmotor_control_mit(DMmotor_device_t *motor, float p_des, float v_des, float kp, float kd, float t_ff)
{
    if (motor != NULL)
    {
        motor->mode = DM_CTRL_MIT;
        motor->cmd_data.p_des = p_des;
        motor->cmd_data.v_des = v_des;
        motor->cmd_data.kp = kp;
        motor->cmd_data.kd = kd;
        motor->cmd_data.t_ff = t_ff;
    }
}

void DMmotor_control_mit_with_gravity(DMmotor_device_t *motor, float target_p, float kp, float kd, float horizontal_offset)
{
    if (motor == NULL) return;

    // 1. 获取当前实际角度 (相对于水平面)
    // 假设 horizontal_offset 是臂部水平时的 "total_angle"
    float current_theta = motor->motor_measure.total_angle - horizontal_offset;
    
    // 2. 计算当前角度下的重力矩
    // Tau = mgl * cos(theta)
    float t_gravity = motor->params.gravity_mgl * cosf(current_theta);
    
    // 3. 调用基础 MIT 控制
    DMmotor_control_mit(motor, target_p, 0.0f, kp, kd, t_gravity);
}

void DMmotor_control_pos_vel(DMmotor_device_t *motor, float p_des, float v_des)
{
    if (motor != NULL)
    {
        motor->mode = DM_CTRL_POS_VEL;
        motor->cmd_data.pos_vel_p_des = p_des;
        motor->cmd_data.pos_vel_v_des = v_des;
    }
}

void DMmotor_enable(DMmotor_device_t *motor)
{
    if (motor != NULL)
    {
        motor->motor_status = DM_MOTOR_ENABLED;
        DMmotor_send_cmd(motor, DM_CMD_ENABLE);
    }
}

void DMmotor_stop(DMmotor_device_t *motor)
{
    if (motor != NULL)
    {
   //     motor->motor_status = DM_MOTOR_STOP;
    }
}

void DMmotor_set_zero(DMmotor_device_t *motor)
{
    if (motor != NULL)
    {
        DMmotor_send_cmd(motor, DM_CMD_SET_ZERO);
        motor->motor_measure.total_round = 0;
        motor->motor_measure.total_angle = 0;
    }
}

void DMmotor_clear_error(DMmotor_device_t *motor)
{
    if (motor != NULL)
    {
        DMmotor_send_cmd(motor, DM_CMD_CLEAR_ERROR);
    }
}

DMmotor_status_e DMmotor_get_status(DMmotor_device_t *motor)
{
    // if (motor == NULL) return DM_MOTOR_STOP;
    // if (motor->wdg && !Watchdog_is_online(motor->wdg))
    // {
    //     return DM_MOTOR_STOP;
    // }
    return motor->motor_status;
}

DMmotor_measure_t DMmotor_get_measure(DMmotor_device_t *motor)
{
    DMmotor_measure_t measure = {0};
    if (motor != NULL)
    {
        measure = motor->motor_measure;
    }
    return measure;
}

void DMmotor_control_all(void)
{
    for (uint8_t i = 0; i < dm_motor_count; i++)
    {
        DMmotor_device_t *motor = dm_motor_instances[i];
        if (motor != NULL)
        {
            switch (motor->mode)
            {
                case DM_CTRL_MIT:
                    DMmotor_send_mit_frame(motor);
                    break;
                case DM_CTRL_POS_VEL:
                    DMmotor_send_pos_vel_frame(motor);
                    break;
                default:
                    DMmotor_send_mit_frame(motor); // 默认
                    break;
            }
        }
    }
}
