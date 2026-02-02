#ifndef SYSU_INFANTRY_BUZZER_ALARM_H
#define SYSU_INFANTRY_BUZZER_ALARM_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "buzzer_driver.h" // 你的蜂鸣器驱动头文件


// ********** 第一步：定义多模块枚举（区分不同模块的离线告警）**********
typedef enum {
    BUZZER_ALARM_NONE = 0,        // 无告警
    BUZZER_ALARM_MOTOR,          // 电机离线(can)
    BUZZER_ALARM_UART,            // UART模块离线(遥控器)
    BUZZER_ALARM_MAX              // 告警类型最大值
} Buzzer_Alarm_Type_e;



// 函数声明
void Buzzer_alarm_init(void);
void Buzzer_send_alarm(Buzzer_Alarm_Type_e alarm_type);
void Buzzer_alarm_handle_command(Buzzer_Alarm_Type_e type);

#endif //SYSU_INFANTRY_BUZZER_ALARM_H