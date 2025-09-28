#ifndef SYSU_INFANTRY_BUZZER_DRIVER_H
#define SYSU_INFANTRY_BUZZER_DRIVER_H

#include "tim.h"

// 预分频后的C板定时器时钟为1MHz
#define TIMER_CLOCK_FREQ 1000000 // 1MHz

/****************************蜂鸣器音调*********************************/
#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988
#define DoHighFreq 1047  // C6
#define ReHighFreq 1175 // D6
#define MiHighFreq 1319 // E6
/**********************蜂鸣器配置***************************/
//蜂鸣器音量设置
typedef enum {
    VERY_LOW = 0,
    LOW,
    MEDIUM,
    HIGH,
    VERY_HIGH
}Buzzer_volume_e;

typedef enum {
    BUZZER_ON = 0,
    BUZZER_OFF
}Buzzer_state;

//蜂鸣器配置
typedef struct {
    TIM_HandleTypeDef *htim;//蜂鸣器所用的定时器
    uint32_t tim_channel;   //定时器通道
    Buzzer_volume_e buzzer_volume;//音量
    Buzzer_state buzzer_state;  //蜂鸣器状态
}Buzzer_device_t;

//蜂鸣器初始化
void Buzzer_init(Buzzer_device_t *buzzer,TIM_HandleTypeDef *htim,uint32_t tim_channel);

//蜂鸣器设置频率
void Buzzer_set_frequency(Buzzer_device_t *buzzer,uint32_t freq);

//蜂鸣器设置音量
void Buzzer_set_volume(Buzzer_device_t *buzzer,Buzzer_volume_e volume);

//蜂鸣器开启
void Buzzer_start(Buzzer_device_t *buzzer);

//蜂鸣器停止
void Buzzer_stop(Buzzer_device_t *buzzer);



#endif //SYSU_INFANTRY_BUZZER_DRIVER_H