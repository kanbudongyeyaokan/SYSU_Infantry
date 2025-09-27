#ifndef SYSU_INFANTRY_BUZZER_MUSIC_H
#define SYSU_INFANTRY_BUZZER_MUSIC_H
#include "main.h"
#include "buzzer_driver.h"
// 定义音符结构
typedef struct {
    uint32_t freq;      // 频率
    uint32_t duration;  // 持续时间(ms)
} Note_t;

void Play_Twinkle_Twinkle(Buzzer_device_t* buzzer);

void Play_Super_Mario(Buzzer_device_t* buzzer);

#endif //SYSU_INFANTRY_BUZZER_MUSIC_H