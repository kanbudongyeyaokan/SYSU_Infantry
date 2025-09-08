#include "dji_motor.h"
#include "bsp_can.h"
#include "stdlib.h"

// 电机实例数组,用于管理已初始化的电机列表
static Djimotor_device_t *motor_instances[MAX_MOTOR_COUNT] = {NULL};
static uint8_t motor_count = 0;

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
    can_config.receive_callback =Djimotor_decoder;//这里放大疆电机的解析函数

    // 初始化CAN发送配置
    can_config.tx_config.StdId = DJI_Get_Tx_ID(config->type, config->id);
    can_config.tx_config.IDE = CAN_ID_STD;
    can_config.tx_config.RTR = CAN_RTR_DATA;
    can_config.tx_config.DLC = 8;

    // 注册CAN设备
    motor->can_dev = Can_device_register(&can_config);
    if (motor->can_dev == NULL) {
        free(motor);
        return NULL;
    }

    // 添加到电机实例数组
    motor_instances[motor_count++] = motor;

    return motor;
}