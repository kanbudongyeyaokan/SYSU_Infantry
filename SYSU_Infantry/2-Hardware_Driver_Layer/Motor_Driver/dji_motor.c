#include "dji_motor.h"
#include "bsp_can.h"
#include "stdlib.h"

// 电机实例数组,用于管理已初始化的电机列表
static Djimotor_device_t *motor_instances[MAX_MOTOR_COUNT] = {NULL};
static uint8_t motor_count = 0;

/**对应不同电机发送CAN-ID的8字节数组**/
static uint8_t motor_



//解译电机反馈回来的数据
static void Decode_djimotor(Djimotor_device_t *djimotor)
{
    uint8_t *rxbuff = djimotor->can_controller->rx_buffer;
    Djimotor_measure_t *measure = &(djimotor->motor_measure); // 保存了当前电机的所有信息

    // 解析数据
    measure->current_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];//当前电机编码器值
    measure->current_angle = ECD_ANGLE_COEF_DJI * (float)measure->current_ecd;
    measure->angular_velocity = (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));//角速度
    measure->real_current = ((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->motor_temperature = rxbuff[6];
    measure->last_ecd = measure->current_ecd;

}

// 电机初始化函数
Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config) {
    // 超过最大电机支持数量
    if (motor_count >= MAX_MOTOR_COUNT) {
        return NULL;
    }
    Djimotor_device_t *motor = (Djimotor_device_t *)malloc(sizeof(Djimotor_device_t));
    if (motor == NULL) {
        return NULL;
    }

    // 初始化基本参数
    memset(motor, 0, sizeof(Djimotor_device_t));
    strncpy(motor->motor_name, config->motor_name, sizeof(motor->motor_name) - 1);//名字
    motor->motor_type = config->motor_type;//电机类型
    motor->motor_status = MOTOR_STOP;//电机运动状态
    //控制器初始化
    Pid_init(&motor->motor_pid.current_pid,&config->motor_controller_init.current_pid);//电流环
    Pid_init(&motor->motor_pid.angle_pid,&config->motor_controller_init.angle_pid);    //角度环
    Pid_init(&motor->motor_pid.speed_pid,&config->motor_controller_init.speed_pid);    //速度环

    // 配置CAN设备
    Can_init_t can_config;
    memset(&can_config, 0, sizeof(Can_init_t));
    can_config.can_handle = config->can_init.can_handle;//CAN句柄
    can_config.can_id =  config->can_init.can_id;//电机的CAN-ID，值为1-8
    can_config.rx_id = config->can_init.rx_id;//电机的CAN接收ID，比如0X201
    config->can_init.receive_callback = Decode_djimotor(motor);//这里放大疆电机的解析函数

    // 注册CAN设备
    motor->can_controller = Can_device_register(&can_config);
    if (motor->can_controller == NULL) {
        free(motor);
        return NULL;
    }

    // 添加到电机实例数组
    motor_instances[motor_count++] = motor;

    return motor;
}


