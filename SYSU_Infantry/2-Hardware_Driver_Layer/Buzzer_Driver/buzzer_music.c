#include "buzzer_driver.h"
#include "main.h"
#include "buzzer_music.h"
#include "tim.h"

// 《小星星》乐谱
static const Note_t twinkle_twinkle[] = {
    // 第一句：一闪一闪亮晶晶
    {DoFreq, 500}, {DoFreq, 500}, {SoFreq, 500}, {SoFreq, 500},
    {LaFreq, 500}, {LaFreq, 500}, {SoFreq, 1000},
    
    // 第二句：满天都是小星星
    {FaFreq, 500}, {FaFreq, 500}, {MiFreq, 500}, {MiFreq, 500},
    {ReFreq, 500}, {ReFreq, 500}, {DoFreq, 1000},
    
    // 第三句：挂在天空放光明
    {SoFreq, 500}, {SoFreq, 500}, {FaFreq, 500}, {FaFreq, 500},
    {MiFreq, 500}, {MiFreq, 500}, {ReFreq, 1000},
    
    // 第四句：好像许多小眼睛
    {SoFreq, 500}, {SoFreq, 500}, {FaFreq, 500}, {FaFreq, 500},
    {MiFreq, 500}, {MiFreq, 500}, {ReFreq, 1000},
    
    // 重复第一句：一闪一闪亮晶晶
    {DoFreq, 500}, {DoFreq, 500}, {SoFreq, 500}, {SoFreq, 500},
    {LaFreq, 500}, {LaFreq, 500}, {SoFreq, 1000},
    
    // 重复第二句：满天都是小星星
    {FaFreq, 500}, {FaFreq, 500}, {MiFreq, 500}, {MiFreq, 500},
    {ReFreq, 500}, {ReFreq, 500}, {DoFreq, 1000},
    
    // 结束
    {0, 0} // 结束标志
};

// 《超级马里奥》主题曲乐谱
static const Note_t super_mario[] = {
    {MiFreq, 200}, {MiFreq, 200}, {0, 200}, {MiFreq, 200}, {0, 200}, {DoFreq, 200}, {MiFreq, 200}, {0, 200},
    {SoFreq, 400}, {0, 400}, {DoHighFreq, 400}, {0, 400},
    {SoFreq, 400}, {0, 400}, {MiFreq, 400}, {0, 400},
    {LaFreq, 200}, {0, 200}, {SiFreq, 200}, {0, 200}, {LaFreq, 200}, {0, 200}, {SoFreq, 300}, {MiFreq, 300}, {SoFreq, 300}, {LaFreq, 300},
    {SiFreq, 400}, {0, 400}, {DoHighFreq, 400}, {SiFreq, 400}, {LaFreq, 400},
    {SoFreq, 300}, {DoFreq, 300}, {ReFreq, 300}, {SoFreq, 300}, {0, 300}, {SoFreq, 300}, {0, 300},
    {0, 0}
};

/*播放小星星*/
void Play_Twinkle_Twinkle(Buzzer_device_t* buzzer)
{
    // 设置中高音量
    Buzzer_set_volume(buzzer, MEDIUM);

    // 播放整首曲子
    int i = 0;
    while (twinkle_twinkle[i].duration != 0) {
        // 播放当前音符
        Buzzer_set_frequency(buzzer, twinkle_twinkle[i].freq);

        // 持续指定时间
        HAL_Delay(twinkle_twinkle[i].duration);

        i++;
    }

    // 停止蜂鸣器
    Buzzer_set_frequency(buzzer, 0);
}
/*播放马里奥*/
void Play_Super_Mario(Buzzer_device_t* buzzer) {
    // 设置中高音量
    Buzzer_set_volume(buzzer, MEDIUM);

    // 播放整首曲子
    int i = 0;
    while (super_mario[i].duration != 0) {
        // 播放当前音符
        Buzzer_set_frequency(buzzer, super_mario[i].freq);

        // 持续指定时间
        HAL_Delay(super_mario[i].duration);
        i++;
    }

    // 停止蜂鸣器
    Buzzer_set_frequency(buzzer, 0);
}