/**
 * @file    dji_motor.c
 * @brief   大疆电机驱动实现
 * @author  SYSU电控组
 * @note    该文件实现了大疆全系列电机（M3508, GM6020, M2006等）的通用驱动。
 *          主要功能包括：
 *          1. CAN协议解析与反馈数据处理（Decode）
 *          2. 多圈角度解算
 *          3. PID控制计算（Calc_Output）
 *          4. CAN指令打包发送（Send_All_Bus）
 *          5. 掉线检测与保护（基于看门狗）
 */

#include "dji_motor.h"
#include "algorithm_pid.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_wdg.h" // 引入看门狗库
#include "main.h"
#include "stdio.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "buzzer_music.h"
#include "buzzer_alarm.h"
#include "error_handler.h"


// 电机实例数组
static Djimotor_device_t *motor_instances[MAX_MOTOR_COUNT] = {NULL};
static uint8_t motor_count = 0;

// 发送缓冲区 [0]:CAN1 [1]:CAN2; 每个CAN口有3个ID组 (0:1FF, 1:200, 2:2FF)
// 大疆电机的控制帧ID分配：
// 1FF: GM6020 ID 1-4
// 200: M3508/M2006 ID 1-4
// 2FF: GM6020 ID 5-7, M3508/M2006 ID 5-8
static uint8_t motor_can_buffer[2][3][8];
// 发送标志位，用于标记哪一组ID需要发送
static uint8_t motor_can_flag[2][3] = {0};

// 预定义发送用的 CAN 控制器句柄
static Can_controller_t can1_tx_handlers[3];
static Can_controller_t can2_tx_handlers[3];

extern Uart_instance_t *uart_instance;

/* --- 内部辅助函数 --- */

/**
 * @brief   电机离线回调函数 (由看门狗触发)
 * @note    当电机超时未收到反馈时触发，负责停止电机输出以保护系统
 */
static void Motor_Offline_Callback(void *device) {
  Djimotor_device_t *motor = (Djimotor_device_t *)device;
  ERROR_CRITICAL("MOTOR", "motor %s offline", motor->motor_name);
  // 清空速度和电流反馈，防止 PID 积分暴涨
  motor->motor_measure.angular_velocity = 0;
  motor->motor_measure.real_current = 0;

    // 将状态设为 STOP，停止计算输出
  motor->motor_status = MOTOR_STOP;

  //Watchdog_buzzer_alarm(motor->motor_name); // 触发看门狗报警，提示电机掉线
}

/**
 * @brief   解译电机反馈数据 (CAN接收回调)
 * @param   can_dev: CAN设备句柄
 * @param   context: 上下文指针，指向对应的 Djimotor_device_t 实例
 * @note    解析 CAN 报文中的 8 字节数据：
 *          [0-1]: 转子机械角度 (0-8191)
 *          [2-3]: 转子转速 (rpm)
 *          [4-5]: 实际转矩电流
 *          [6]:   电机温度
 *          [7]:   保留
 */
static void Decode_djimotor(Can_controller_t *can_dev, void *context) {
  Djimotor_device_t *motor = (Djimotor_device_t *)context;
  uint8_t *rxbuff = can_dev->rx_buffer;
  Djimotor_measure_t *measure = &(motor->motor_measure);

  // 1. 喂狗：收到数据说明电机在线
  if (motor->wdg != NULL) {
    Watchdog_feed(motor->wdg);
  }

  // 2. 保存旧值，用于计算差值
  measure->last_ecd = measure->current_ecd;

  // 3. 解析新值 (大端序)
  measure->current_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
  measure->angular_velocity =
      (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3])); // rpm
  measure->real_current = ((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
  measure->motor_temperature = rxbuff[6];

  // 4. 多圈角度解算
  // 处理 0 <-> 8191 的边界跳变
  int32_t diff = measure->current_ecd - measure->last_ecd;
  if (diff > 4096) {
    // 发生了反向跳变 (例如 0 -> 8190)，实际上是转了一圈
    diff -= 8192;
    measure->total_round--;
  } else if (diff < -4096) {
    // 发生了正向跳变 (例如 8190 -> 0)
    diff += 8192;
    measure->total_round++;
  }

  // 更新连续的角度值
  measure->current_angle = measure->current_ecd * ECD_ANGLE_COEF_DJI; // 0-360度
  measure->total_angle += diff * ECD_ANGLE_COEF_DJI; // 累计总角度
}

// 初始化全局发送句柄
static void Init_Global_Tx_Handlers(void) {
  static uint8_t is_init = 0;
  if (is_init)
    return;

  // 初始化 CAN1 和 CAN2 的发送配置
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
  
  // 设置标准帧ID (控制帧)
  // Group 0: 0x1FF
  can1_tx_handlers[0].tx_config.StdId = 0x1FF;
  can1_tx_handlers[1].tx_config.StdId = 0x200;
  can1_tx_handlers[2].tx_config.StdId = 0x2FF;

  can2_tx_handlers[0].tx_config.StdId = 0x1FF;
  can2_tx_handlers[1].tx_config.StdId = 0x200;
  can2_tx_handlers[2].tx_config.StdId = 0x2FF;

  is_init = 1;
}

// --- 公共接口 ---

/**
 * @brief   初始化并注册一个大疆电机
 * @param   config: 初始化配置结构体
 * @return  Djimotor_device_t*: 返回分配的电机对象指针
 */
Djimotor_device_t *DJI_Motor_Init(Djimotor_init_config_t *config) {
  if (motor_count >= MAX_MOTOR_COUNT)
    return NULL;

  Djimotor_device_t *motor =
      (Djimotor_device_t *)malloc(sizeof(Djimotor_device_t));
  if (motor == NULL)
    return NULL;
    
  // 初始化全局发送句柄 (只需一次)
  Init_Global_Tx_Handlers();

  // 1. 基础配置拷贝
  memset(motor, 0, sizeof(Djimotor_device_t));
  strncpy(motor->motor_name, config->motor_name, sizeof(motor->motor_name) - 1);
  motor->motor_type = config->motor_type;
  motor->motor_status = config->motor_status;
  motor->deadzone_compensation = config->deadzone_compensation;

  // PID 初始化
  motor->motor_pid.close_loop = config->motor_controller_init.close_loop;
  motor->motor_pid.angle_source = config->motor_controller_init.angle_source;
  motor->motor_pid.speed_source = config->motor_controller_init.speed_source;
  motor->motor_pid.other_angle_feedback_ptr =
      config->motor_controller_init.other_angle_feedback_ptr;
  motor->motor_pid.other_speed_feedback_ptr =
      config->motor_controller_init.other_speed_feedback_ptr;

  Pid_init(&motor->motor_pid.current_pid,
           &config->motor_controller_init.current_pid);
  Pid_init(&motor->motor_pid.angle_pid,
           &config->motor_controller_init.angle_pid);
  Pid_init(&motor->motor_pid.speed_pid,
           &config->motor_controller_init.speed_pid);

  // CAN 接收配置注册
  Can_init_t can_config;
  memset(&can_config, 0, sizeof(Can_init_t));
  can_config.can_handle = config->can_init.can_handle;
  can_config.can_id = config->can_init.can_id; // 电机的反馈ID (如 0x201)
  can_config.tx_id = config->can_init.tx_id;   // 发送时的ID索引 (1-8)
  can_config.rx_id = config->can_init.rx_id;
  can_config.receive_callback = Decode_djimotor;
  can_config.context = motor;

  motor->can_controller = Can_device_init(&can_config);
  if (motor->can_controller == NULL) {
    free(motor);
    return NULL;
  }

  // 看门狗注册
  Watchdog_init_t wdg_conf;
  wdg_conf.owner_id = motor;
  wdg_conf.reload_count = 300; // 3 * 100ms = 300ms 超时
  // [修正] 这里赋值正确的函数名
  wdg_conf.callback = Motor_Offline_Callback;
  wdg_conf.online_callback = NULL;
  strcpy(wdg_conf.name, motor->motor_name);
  motor->wdg = Watchdog_register(&wdg_conf);

  motor_instances[motor_count++] = motor;
  return motor;
}

/**
 * @brief   动态修改电机控制参数
 * @param   motor: 电机对象
 * @param   ctrl_params: 新的 PID 参数配置
 * @note    由于参数修改可能发生在运行时，使用临界区保护防止被打断
 */
void Djimotor_change_controller(Djimotor_device_t *motor,
                                Djimotor_controller_init_t ctrl_params) {
  if (!motor)
    return;

  // 1. 【进入临界区】关闭全局中断，防止被打断
  // 具体的函数名取决于你的 RTOS 或 HAL 库，例如：
  __disable_irq();

  motor->motor_pid.close_loop = ctrl_params.close_loop;
  motor->motor_pid.angle_source = ctrl_params.angle_source;
  motor->motor_pid.speed_source = ctrl_params.speed_source;
  motor->motor_pid.other_angle_feedback_ptr =
      ctrl_params.other_angle_feedback_ptr;
  motor->motor_pid.other_speed_feedback_ptr =
      ctrl_params.other_speed_feedback_ptr;
  Pid_init(&(motor->motor_pid.speed_pid), &(ctrl_params.speed_pid));
  Pid_init(&(motor->motor_pid.angle_pid), &(ctrl_params.angle_pid));
  Pid_init(&(motor->motor_pid.current_pid), &(ctrl_params.current_pid));
  motor->motor_pid.speed_feedforward = 0.0f;

  // 2. 【退出临界区】恢复全局中断
  __enable_irq();
}

void Djimotor_set_target(Djimotor_device_t *motor, float target) {
  if (motor)
    motor->motor_pid.pid_target = target;
}

Djimotor_status_e Djimotor_get_status(Djimotor_device_t *motor) {
  if (motor == NULL)
    return MOTOR_STOP;
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
  if (motor)
    motor->motor_status = status;
}

void Djimotor_set_deadzone(Djimotor_device_t *motor, int16_t deadzone) {
  if (motor)
    motor->deadzone_compensation = deadzone;
}

int16_t Djimotor_get_deadzone(Djimotor_device_t *motor) {
  if (motor)
    return motor->deadzone_compensation;
  return 0;
}

/**
 * @brief   计算电机输出 (PID计算核心)
 * @param   motor: 电机对象
 * @note    该函数仅进行 PID 运算并将结果存入 motor->out_current，不涉及 CAN 发送。
 *          由各功能任务（chassis, gimbal等）定期调用。
 */
void Djimotor_Calc_Output(Djimotor_device_t *motor) {
  if (motor == NULL)
    return;

  // 1. 离线检查：如果掉线，强制停止输出
  // if (motor->wdg && !Watchdog_is_online(motor->wdg)) {
  //   motor->motor_status = MOTOR_STOP;
  // }

  float output = 0.0f;
  Djimotor_measure_t *measure = &motor->motor_measure;

  // 2. 获取反馈值
  float angle_feedback = measure->total_angle;
  if (motor->motor_pid.angle_source == OTHER_FEEDBACK &&
      motor->motor_pid.other_angle_feedback_ptr) {
    angle_feedback = *(motor->motor_pid.other_angle_feedback_ptr);
  }
  float speed_feedback = measure->angular_velocity;
  if (motor->motor_pid.speed_source == OTHER_FEEDBACK &&
      motor->motor_pid.other_speed_feedback_ptr) {
    speed_feedback = *(motor->motor_pid.other_speed_feedback_ptr);
  }
  float current_feedback = measure->real_current;

  // 3. PID 计算
  if (motor->motor_status == MOTOR_STOP) {
    output = 0.0f;
    // 停止时复位 PID 状态，清除积分项
    Pid_reset(&motor->motor_pid.current_pid);
    Pid_reset(&motor->motor_pid.speed_pid);
    Pid_reset(&motor->motor_pid.angle_pid);
  } else {
    // 根据配置的闭环模式进行串级PID计算
    switch (motor->motor_pid.close_loop) {
    case OPEN_LOOP:
      output = motor->motor_pid.pid_target;
      break;
    case CURRENT_LOOP:
      output = Pid_calculate(&motor->motor_pid.current_pid, current_feedback,
                             motor->motor_pid.pid_target);
      break;
    case SPEED_LOOP:
      output = Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback,
                             motor->motor_pid.pid_target);
      break;
    case ANGLE_LOOP:
      output = Pid_calculate(&motor->motor_pid.angle_pid, angle_feedback,
                             motor->motor_pid.pid_target);
      break;
    case SPEED_AND_CURRENT_LOOP: {
      // 速度环输出作为电流环输入
      float current_target =
          Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback,
                        motor->motor_pid.pid_target);
      output = Pid_calculate(&motor->motor_pid.current_pid, current_feedback,
                             current_target);
      break;
    }
    case ANGLE_AND_SPEED_LOOP: {
      // 角度环输出作为速度环输入 (带前馈)
      float speed_target =
          Pid_calculate(&motor->motor_pid.angle_pid, angle_feedback,
                        motor->motor_pid.pid_target) +
          motor->motor_pid.speed_feedforward;
      output = Pid_calculate(&motor->motor_pid.speed_pid, speed_feedback,
                             speed_target);
      break;
    }
    default:
      output = 0.0f;
      break;
    }
  }

  // [核心修改] 将计算结果存入结构体，而不是直接填 CAN buffer
  motor->out_current = (int16_t)output;
}

/**
 * @brief   发送所有电机的控制指令 (由 Motor_control_task 调用)
 * @note    该函数遍历所有电机，将计算好的 out_current 打包并通过 CAN 发送。
 *          运行频率极高 (1kHz)，务必保证效率。
 */
void Djimotor_Send_All_Bus(void) {
  // 清空标志位
  memset(motor_can_flag, 0, sizeof(motor_can_flag));

  // 遍历所有注册的电机，将 out_current 填入 buffer
  for (uint8_t i = 0; i < motor_count; i++) {
    Djimotor_device_t *motor = motor_instances[i];

    // 在读取的那一瞬间加锁，防止多线程竞争（虽然 out_current 通常是原子读写的，但加锁更保险）
    taskENTER_CRITICAL();
    int16_t current_val = motor->out_current;
    taskEXIT_CRITICAL();

    // 大疆电机的 CAN ID 匹配
    CAN_HandleTypeDef *hcan = motor->can_controller->can_handle;
    uint32_t tx_id = motor->can_controller->tx_id;
    uint32_t can_id = motor->can_controller->can_id;

    uint8_t can_idx = (hcan == &hcan1) ? 0 : 1;
    uint8_t group_idx = 0;

    if (can_id == 0x1FF)
      group_idx = 0;
    else if (can_id == 0x200)
      group_idx = 1;
    else if (can_id == 0x2FF)
      group_idx = 2;
    else
      continue;

    // 计算 buffer 偏移量
    uint8_t buffer_offset = 0;
    // 注意：GM6020 ID逻辑可能需要根据实际手册确认，这里沿用你的逻辑
    if (motor->motor_type == GM6020) {
      if (tx_id <= 4)
        buffer_offset = (tx_id - 1) * 2;
      else
        buffer_offset = (tx_id - 5) * 2;
    } else {
      if (tx_id <= 4)
        buffer_offset = (tx_id - 1) * 2;
      else
        buffer_offset = (tx_id - 5) * 2;
    }

    // 填充 buffer (大端序: 高8位在前)
    motor_can_buffer[can_idx][group_idx][buffer_offset] =
        (uint8_t)(current_val >> 8);
    motor_can_buffer[can_idx][group_idx][buffer_offset + 1] =
        (uint8_t)(current_val);

    // 标记该组需要发送
    motor_can_flag[can_idx][group_idx] = 1;
  }

  // 统一发送各组报文
  // CAN1
  if (motor_can_flag[0][0])
    Can_send_data(&can1_tx_handlers[0], motor_can_buffer[0][0]);
  if (motor_can_flag[0][1])
    Can_send_data(&can1_tx_handlers[1], motor_can_buffer[0][1]);
  if (motor_can_flag[0][2])
    Can_send_data(&can1_tx_handlers[2], motor_can_buffer[0][2]);

  // CAN2
  if (motor_can_flag[1][0])
    Can_send_data(&can2_tx_handlers[0], motor_can_buffer[1][0]);
  if (motor_can_flag[1][1])
    Can_send_data(&can2_tx_handlers[1], motor_can_buffer[1][1]);
  if (motor_can_flag[1][2])
    Can_send_data(&can2_tx_handlers[2], motor_can_buffer[1][2]);
}