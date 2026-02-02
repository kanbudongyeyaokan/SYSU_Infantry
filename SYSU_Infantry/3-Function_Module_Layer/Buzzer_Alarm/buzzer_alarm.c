#include "buzzer_alarm.h"
#include "buzzer_music.h"
#include "tim.h"
#include "robot_task.h"
#include "queue.h"
// 全局蜂鸣器设备实例
Buzzer_device_t buzzer_dev;
// 蜂鸣器播放消息队列ID


/**
 * @brief 蜂鸣器告警初始化（硬件+消息队列）
 * @note  系统初始化阶段调用（main/System_Init中）
 */
void Buzzer_alarm_init(void)
{
    Buzzer_init(&buzzer_dev, &htim4, TIM_CHANNEL_3); // 示例：TIM4_CH3
    Buzzer_set_volume(&buzzer_dev, 80);              // 告警音量（0-100）
}

/**
 * @brief 发送蜂鸣器告警指令（中断/任务均可安全调用）
 * @param alarm_type 告警类型（哪个模块离线）
 * @note  非阻塞，只投递指令，不执行播放
 */
void Buzzer_send_alarm(Buzzer_Alarm_Type_e alarm_type)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 中断安全的队列发送
    xQueueSendFromISR(Buzzer_cmd_queue_handle, &alarm_type,&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Buzzer_alarm_handle_command(Buzzer_Alarm_Type_e type) {

    switch (type) {
        case BUZZER_ALARM_MOTOR:
            Buzzer_set_frequency(&buzzer_dev, DoFreq);
            vTaskDelay(100);
            Buzzer_set_frequency(&buzzer_dev, 0);
            vTaskDelay(100);
            break;
        case BUZZER_ALARM_UART:
            Buzzer_set_frequency(&buzzer_dev, SiFreq);
            vTaskDelay(100);
            Buzzer_set_frequency(&buzzer_dev, 0);
            vTaskDelay(100);
            break;

        default:
            Buzzer_set_frequency(&buzzer_dev, 0);
            break;
    }

}