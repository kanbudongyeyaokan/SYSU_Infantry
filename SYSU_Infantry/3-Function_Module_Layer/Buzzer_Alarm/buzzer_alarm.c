#include "buzzer_alarm.h"
#include "buzzer_music.h"
#include "tim.h"
#include "robot_task.h"
#include "queue.h"
#include <string.h>
// 全局蜂鸣器设备实例
static Buzzer_device_t buzzer;


/**
 * @brief 蜂鸣器告警初始化（硬件+消息队列）
 * @note  系统初始化阶段调用（main/System_Init中）
 */
void Buzzer_alarm_init(void) {
    Buzzer_init(&buzzer, &htim4, TIM_CHANNEL_3); // 示例：TIM4_CH3
    Buzzer_set_volume(&buzzer, 80); // 告警音量（0-100）
}


/**
 * @brief  解析看门狗名称，将对应报警次数写入FreeRTOS队列
 * @param  wdg_name: 离线电机的看门狗名称
 * @note   轻量函数，无阻塞/延时，可在看门狗回调中直接调用
 *         报警次数：CHASSIS_FR(1)→FL(2)→BL(3)→BR(4)→yaw(5)→pitch(6)→FRIC_L(7)→FRIC_R(8)→LOADER(9)
 */
void Watchdog_buzzer_alarm(const char *wdg_name) {
    // 1. 入参+队列判空校验（防止空指针/队列未创建）
    if (wdg_name == NULL || Buzzer_cmd_queue_handle == NULL) {
        return;
    }

    // 2. 解析电机名称，赋值对应报警次数（默认0：未知电机，不报警/可改为10）
    uint8_t alarm_times = 0;
    if (strcmp(wdg_name, "RC") == 0) {
        alarm_times = ALARM_RC; // 20
    } else if (strcmp(wdg_name, "BMI088") == 0) {
        alarm_times = ALARM_BMI088; // 21
    } else if (strcmp(wdg_name, "Referee") == 0) {
        alarm_times = ALARM_REFEREE; // 22
    } else if (strcmp(wdg_name, "CHASSIS_FR") == 0) {
        alarm_times = 1;
    } else if (strcmp(wdg_name, "CHASSIS_FL") == 0) {
        alarm_times = 2;
    } else if (strcmp(wdg_name, "CHASSIS_BL") == 0) {
        alarm_times = 3;
    } else if (strcmp(wdg_name, "CHASSIS_BR") == 0) {
        alarm_times = 4;
    } else if (strcmp(wdg_name, "yaw_motor") == 0) {
        alarm_times = 5;
    } else if (strcmp(wdg_name, "pitch_motor") == 0) {
        alarm_times = 6;
    } else if (strcmp(wdg_name, "FRICTION_L") == 0) {
        alarm_times = 7;
    } else if (strcmp(wdg_name, "FRICTION_R") == 0) {
        alarm_times = 8;
    } else if (strcmp(wdg_name, "LOADER") == 0) {
        alarm_times = 9;
    }

    // 3. 解析到有效次数，写入队列（非阻塞写：队列满则直接返回，不等待）
    if (alarm_times > 0) {
        xQueueSend(Buzzer_cmd_queue_handle, &alarm_times, 0);
    }
}

void Alarm_handle_command(uint8_t *cmd) {
    if (*cmd > 0) {
        // 特殊值（20/21/22）
        if (*cmd >= ALARM_RC) {
            // 大于等于20，均为音乐播放指令
            switch (*cmd) {
                case ALARM_RC: // RC离线：推荐播放System_Start/Intel（短促科技感）
                    Play_Mario_Die(&buzzer);
                    break;
                case ALARM_BMI088: // BMI088离线：推荐播放EVA_Start/Cyber_Boot（警示感）
                    Play_Windows_Shutdown(&buzzer);
                    break;
                case ALARM_REFEREE: // 裁判系统离线：推荐播放Mario_Die/Windows_Shutdown（故障警示）
                    Play_Intel(&buzzer);
                    break;
                default:
                    break;
            }
            // 音乐播放完成后，暂停一段时间（避免重复触发刷屏）
            vTaskDelay(pdMS_TO_TICKS(1000)); // 播放后暂停5秒
        }
        //常规值（1-9）响对应次数的蜂鸣
        else if (*cmd >= 1 && *cmd <= 9) {
            for (uint8_t i = 0; i < *cmd; i++) {
                Buzzer_start(&buzzer);
                Buzzer_set_volume(&buzzer, HIGH);
                Buzzer_set_frequency(&buzzer,1000);
                vTaskDelay(pdMS_TO_TICKS(BUZZER_SINGLE_MS));
                //Buzzer_stop(&buzzer);
                Buzzer_set_frequency(&buzzer,0);
                // 最后一次蜂鸣后，不添加间隔
                if (i != *cmd - 1) {
                    vTaskDelay(pdMS_TO_TICKS(BUZZER_INTERVAL_MS));
                }
            }
            // 多次蜂鸣后，暂停一段时间
            vTaskDelay(pdMS_TO_TICKS(BUZZER_PAUSE_MS));
        }
    }
    // 清空接收值，防止下次误触发
    *cmd = 0;
}
