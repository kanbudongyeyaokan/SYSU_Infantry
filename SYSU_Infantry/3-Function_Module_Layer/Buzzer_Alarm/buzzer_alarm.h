#ifndef SYSU_INFANTRY_BUZZER_ALARM_H
#define SYSU_INFANTRY_BUZZER_ALARM_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "buzzer_driver.h" // 你的蜂鸣器驱动头文件

// 定义特殊报警值：大于电机的1-9，避免冲突
#define ALARM_RC          20  // RC离线：播放专属音乐
#define ALARM_BMI088      21  // BMI088离线：播放专属音乐
#define ALARM_REFEREE     22  // 裁判系统离线：播放专属音乐
#define BUZZER_SINGLE_MS       30 // 单下响铃时长
#define BUZZER_INTERVAL_MS     200 // 同组响铃间隔
#define BUZZER_PAUSE_MS        500// 响完一组后的停顿时长

// 函数声明
void Buzzer_alarm_init(void);
void Watchdog_buzzer_alarm(const char *wdg_name);
void Alarm_handle_command(uint8_t *cmd);

#endif //SYSU_INFANTRY_BUZZER_ALARM_H