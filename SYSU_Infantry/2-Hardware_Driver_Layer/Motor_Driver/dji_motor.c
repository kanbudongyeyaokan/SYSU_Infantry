#include "dji_motor.h"
#include "bsp_can.h"
#include "stdlib.h"

// 电机实例数组,用于管理已初始化的电机列表
static Djimotor_device_t *motor_instances[MAX_MOTOR_COUNT] = {NULL};
static uint8_t motor_count = 0;

/**对应不同电机发送CAN-ID的8字节数组,4个电机的共享发送缓冲区**/
static uint8_t djimotor_can1_0x1ff_tx[8]={0};
static uint8_t djimotor_can1_0x200_tx[8]={0};
static uint8_t djimotor_can1_0x2ff_tx[8]={0};
static uint8_t djimotor_can2_0x1ff_tx[8]={0};
static uint8_t djimotor_can2_0x200_tx[8]={0};
static uint8_t djimotor_can2_0x2ff_tx[8]={0};
/****************************************************/

// 获取缓冲区索引
static uint8_t* Get_buffer_pointer(CAN_HandleTypeDef *hcan, uint32_t can_id) {
    if (hcan == &hcan1) {
        if (can_id == 0x1FF) return djimotor_can1_0x1ff_tx;
        if (can_id == 0x200) return djimotor_can1_0x200_tx;
        if (can_id == 0x2FF) return djimotor_can1_0x2ff_tx;
    } else if (hcan == &hcan2) {
        if (can_id == 0x1FF) return djimotor_can2_0x1ff_tx;
        if (can_id == 0x200) return djimotor_can2_0x200_tx;
        if (can_id == 0x2FF) return djimotor_can2_0x2ff_tx;
    }
    return NULL; // 无效索引
}


//解译电机反馈回来的数据
static void Decode_djimotor(Can_controller_t* can_dev,void *context)
{
    // 将上下文转换为电机实例
    Djimotor_device_t *motor = (Djimotor_device_t*)context;

    uint8_t *rxbuff = can_dev->rx_buffer;
    Djimotor_measure_t *measure = &(motor->motor_measure); // 保存了当前电机的所有信息

    // 解析数据
    measure->current_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];//当前电机编码器值
    measure->last_ecd = measure->current_ecd;
    measure->current_angle = ECD_ANGLE_COEF_DJI * (float)measure->current_ecd;

    measure->angular_velocity = (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));//角速度

    measure->real_current = ((int16_t)(rxbuff[4] << 8 | rxbuff[5]));//电流值

    measure->motor_temperature = rxbuff[6];//温度
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
    can_config.can_id =  config->can_init.can_id;//电机的CAN总线ID
    can_config.tx_id  =  config->can_init.tx_id;//电机的设备CAN-ID
    can_config.rx_id = config->can_init.rx_id;//电机的CAN接收ID，比如0X201
    can_config.receive_callback = Decode_djimotor;//这里放大疆电机的解析函数

    can_config.context = motor; //将电机实例传入上下文

    // 初始化CAN设备
    motor->can_controller = Can_device_init(&can_config);
    if (motor->can_controller == NULL) {
        free(motor);
        return NULL;
    }

    // 添加到电机实例数组
    motor_instances[motor_count++] = motor;

    return motor;
}

//电机控制函数
void Djimotor_ctrl()

