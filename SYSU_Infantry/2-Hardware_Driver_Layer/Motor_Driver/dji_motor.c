#include "dji_motor.h"
#include "bsp_can.h"
#include "stdlib.h"
#include "bsp_usart.h"
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

static int16_t current_motor = 0;
static uint16_t current_motor_count = 0;

extern UartInstance_t* uart_instance;

// 缓冲区更新标志
static uint8_t buffer_updated[6] = {0}; // 0:CAN1_0x1FF, 1:CAN1_0x200, 2:CAN1_0x2FF, 3:CAN2_0x1FF, 4:CAN2_0x200, 5:CAN2_0x2FF

// 获取缓冲区指针
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

// 获取缓冲区索引
static uint8_t Get_buffer_index(CAN_HandleTypeDef *hcan, uint32_t can_id) {
    if (hcan == &hcan1) {
        if (can_id == 0x1FF) return 0;
        if (can_id == 0x200) return 1;
        if (can_id == 0x2FF) return 2;
    } else if (hcan == &hcan2) {
        if (can_id == 0x1FF) return 3;
        if (can_id == 0x200) return 4;
        if (can_id == 0x2FF) return 5;
    }
    return 0xFF; // 无效索引
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

/****电机控制函数****/
// 设置目标值
void Djimotor_set_target(Djimotor_device_t *motor, float target) {
    //安全检查
    motor->motor_pid.pid_target = target;
}

// 获取电机状态
Djimotor_status_e Djimotor_get_status(Djimotor_device_t *motor) {
    if (motor == NULL) return MOTOR_STOP;
    return motor->motor_status;
}

// 获取电机测量数据
Djimotor_measure_t Djimotor_get_measure(Djimotor_device_t *motor) {
    Djimotor_measure_t measure = {0};
    if (motor != NULL) {
        measure = motor->motor_measure;
    }
    return measure;
}

//设置电机状态
void Djimotor_set_status(Djimotor_device_t *motor,Djimotor_status_e status) {
    motor->motor_status = status;
}

// 计算控制输出
static void Calculate_Motor_Output(Djimotor_device_t *motor) {
     //  if ((motor == NULL || (motor->motor_status != MOTOR_ENABLED))) return;

    float output = 0.0f;

    Djimotor_measure_t *measure = &motor->motor_measure;

    // 获取角度反馈值
    float angle_feedback = 0.0f;
    if (motor->motor_pid.angle_source == MOTOR_FEEDBACK) {
        angle_feedback = measure->current_angle;
    } else if (motor->motor_pid.angle_source == OTHER_FEEDBACK) {
        if (motor->motor_pid.other_angle_feedback_ptr != NULL) {
            angle_feedback = *(motor->motor_pid.other_angle_feedback_ptr);
        }
    }

    // 获取速度反馈值
    float speed_feedback = 0.0f;
    if (motor->motor_pid.speed_source == MOTOR_FEEDBACK) {
        speed_feedback = measure->angular_velocity;
    } else if (motor->motor_pid.speed_source == OTHER_FEEDBACK) {
        if (motor->motor_pid.other_speed_feedback_ptr != NULL) {
            // 转换速度单位(外部反馈单位是度/秒)
            speed_feedback = *(motor->motor_pid.other_speed_feedback_ptr) * 60.0f / 360.0f;
        }
    }

    // 获取电流反馈值 (使用电机自身反馈)
    float current_feedback = measure->real_current; // mA转A

  //  Uart_printf(uart_instance,"before sw\n");
    // 根据控制类型选择控制策略
    switch (motor->motor_pid.close_loop) {
        case OPEN_LOOP: // 开环控制
            output = motor->motor_pid.pid_target; // 直接使用目标值
          //  Uart_printf(uart_instance,"open loop\r\n");
            break;

        case CURRENT_LOOP: // 电流环
            output = Pid_calculate(&motor->motor_pid.current_pid,
                                  current_feedback,
                                  motor->motor_pid.pid_target);
            break;

        case SPEED_LOOP: // 速度环
            output = Pid_calculate(&motor->motor_pid.speed_pid,
                                  speed_feedback,
                                  motor->motor_pid.pid_target);
            break;

        case ANGLE_LOOP: // 角度环
            output = Pid_calculate(&motor->motor_pid.angle_pid,
                                  angle_feedback,
                                  motor->motor_pid.pid_target);
            break;

        case SPEED_AND_CURRENT_LOOP: // 速度环+电流环
        {
            // 外环：速度环
            float current_target = Pid_calculate(&motor->motor_pid.speed_pid,
                                                speed_feedback,
                                                motor->motor_pid.pid_target);

            // 内环：电流环
            output = Pid_calculate(&motor->motor_pid.current_pid,
                                  current_feedback,
                                  current_target);
            break;
        }

        case ANGLE_AND_SPEED_LOOP: // 角度环+速度环
        {
            // 外环：角度环
            float speed_target = Pid_calculate(&motor->motor_pid.angle_pid,
                                              angle_feedback,
                                              motor->motor_pid.pid_target);

            // 内环：速度环
            output = Pid_calculate(&motor->motor_pid.speed_pid,
                                  speed_feedback,
                                  speed_target);
            break;
        }

        default: // 未知控制类型
            output = 1.0f;
            break;
    }

    // 转换为电流值
    int16_t current_val = (int16_t)(output);
   // Uart_printf(uart_instance,"control output:%d\r\n",output);
    current_motor = current_val;

    // 获取缓冲区指针
    uint8_t *buffer = Get_buffer_pointer(motor->can_controller->can_handle,
                                        motor->can_controller->can_id);
    if (buffer == NULL) return;

    // 获取电机在缓冲区中的位置 (1-8对应0-7)
    uint8_t motor_num = motor->can_controller->tx_id - 1;

    //对超出5的ID进行处理,
    if (motor->can_controller->tx_id > 4)
        motor_num -= 4;
    // 写入缓冲区
    buffer[motor_num * 2] = (uint8_t)(current_val >> 8);
    buffer[motor_num * 2 + 1] = (uint8_t)(current_val & 0xFF);
    // 标记缓冲区更新
    uint8_t buf_idx = Get_buffer_index(motor->can_controller->can_handle,
                                      motor->can_controller->can_id);
    if (buf_idx != 0xFF) {
        buffer_updated[buf_idx] = 1;
    }
}
// 管理所有电机的控制命令发送
void Djimotor_control_all(void) {
    // 计算所有电机的输出
    for (uint8_t i = 0; i < motor_count; i++) {
        Calculate_Motor_Output(motor_instances[i]);
    }
    // Uart_printf(uart_instance,"all output\r\n");
    // 发送所有更新的缓冲区
    for (uint8_t i = 0; i < 6; i++) {
        if (buffer_updated[i]) {
            // 准备发送数据
            Can_controller_t temp_can;
            memset(&temp_can, 0, sizeof(Can_controller_t));

            // 根据索引确定CAN句柄和发送ID
            if (i < 3) {
                temp_can.can_handle = &hcan1;
                if (i == 0) temp_can.tx_config.StdId = 0x1FF;
                else if (i == 1) temp_can.tx_config.StdId = 0x200;
                else if (i == 2) temp_can.tx_config.StdId = 0x2FF;
            } else {
                temp_can.can_handle = &hcan2;
                if (i == 3) temp_can.tx_config.StdId = 0x1FF;
                else if (i == 4) temp_can.tx_config.StdId = 0x200;
                else if (i == 5) temp_can.tx_config.StdId = 0x2FF;
            }

            // 设置发送配置
            temp_can.tx_config.IDE = CAN_ID_STD;
            temp_can.tx_config.RTR = CAN_RTR_DATA;
            temp_can.tx_config.DLC = 8;

            // 获取缓冲区数据
            uint8_t *tx_data = Get_buffer_pointer(temp_can.can_handle, temp_can.tx_config.StdId);

            // 发送数据
            if (tx_data != NULL) {
                Can_send_data(&temp_can, tx_data);
            }

            // 清除更新标志
            buffer_updated[i] = 0;
        }
    }
}

