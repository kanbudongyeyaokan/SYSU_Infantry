## 简单的给蜂鸣器的使用写一个驱动
## 驱动库说明
>蜂鸣器的音调频率设定都放在下方
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

//蜂鸣器配置
typedef struct {
    TIM_HandleTypeDef *htim;//蜂鸣器所用的定时器
    uint32_t tim_channel;   //定时器通道
    Buzzer_volume_e buzzer_volume;//音量
    Buzzer_state buzzer_state;  //蜂鸣器状态
}Buzzer_device_t;

## 使用方法
1. 现在需要使用的任务处定义一个Buzzer_device_t类型的实例
2. 然后使用Buzzer_init()初始化这个实例
3. 之后便可以使用Buzzer_set_frequency()和Buzzer_set_volume()来设置蜂鸣器的音调和音量
4. 如何唱一首歌详细见buzzer_music.c
```
    static Buzzer_device_t test_buzzer={0};
    Buzzer_init(&test_buzzer,&htim4,TIM_CHANNEL_3);
    Buzzer_set_volume(&test_buzzer,MEDIUM);
    Buzzer_set_frequency(&test_buzzer,DoFreq);
```

## buzzer_music里面写的是编写好的歌曲，直接调用函数就能实现蜂鸣器唱歌

传入定时器与通道

设置频率和音量