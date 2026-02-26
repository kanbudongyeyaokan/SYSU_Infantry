#include "buzzer_driver.h"
#include "main.h"
#include "buzzer_music.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
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
void Play_Twinkle_Twinkle(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    // 设置中高音量
    Buzzer_set_volume(buzzer, MEDIUM);

    // 播放整首曲子
    int i = 0;
    while (twinkle_twinkle[i].duration != 0)
    {
        // 播放当前音符
        Buzzer_set_frequency(buzzer, twinkle_twinkle[i].freq);

        // 持续指定时间
        vTaskDelay(twinkle_twinkle[i].duration);

        i++;
    }

    // 停止蜂鸣器
    Buzzer_set_frequency(buzzer, 0);
}
/*播放马里奥*/
void Play_Super_Mario(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    // 设置中高音量
    Buzzer_set_volume(buzzer, MEDIUM);

    // 播放整首曲子
    int i = 0;
    while (super_mario[i].duration != 0)
    {
        // 播放当前音符
        Buzzer_set_frequency(buzzer, super_mario[i].freq);

        // 持续指定时间
        vTaskDelay(super_mario[i].duration);
        i++;
    }

    // 停止蜂鸣器
    Buzzer_set_frequency(buzzer, 0);
}

static const Note_t system_start_notes[] = {
    {DoFreq, 100},      // Do (中音)
    {MiFreq, 100},      // Mi
    {SoFreq, 100},      // Sol
    {DoHighFreq, 150},  // Do (高音) - 稍微长一点作为结束感
    {0, 0}
};

void Play_System_Start(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    // 开机通常环境嘈杂，设置高音量
    Buzzer_set_volume(buzzer, HIGH);

    int i = 0;
    while (system_start_notes[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, system_start_notes[i].freq);
        // 注意：这里是阻塞延时，开机初始化时使用无妨，
        // 但如果在控制循环中调用，建议改为非阻塞实现
        vTaskDelay(system_start_notes[i].duration);
        i++;
    }

    // 播放完毕关闭蜂鸣器
    Buzzer_stop(buzzer);
}

// 这里的节奏需要稍微快一点，更有激情
// 定义一些便于阅读的宏（可选，为了让谱子更好看）
#define H_Do DoHighFreq
#define H_Re ReHighFreq
#define H_Mi MiHighFreq

static const Note_t eva_chorus[] = {
    // 1. Zankoku (残酷) - 像残酷的天使一样
    {H_Do, 200}, {SiFreq, 200}, {LaFreq, 200}, {SiFreq, 200}, {H_Do, 200},
    {H_Re, 200}, {H_Mi, 400},

    // 2. No you ni (一样)
    {H_Re, 200}, {H_Do, 200}, {H_Re, 200}, {H_Mi, 600},

    // 3. Shonen (少年) - 少年啊
    {H_Mi, 200}, {H_Re, 200}, {H_Do, 200}, {SiFreq, 200}, {LaFreq, 200},
    {SiFreq, 200}, {H_Do, 400},

    // 4. Shin wa ni nare (神话) - 成为神话吧
    {SiFreq, 200}, {LaFreq, 200}, {SiFreq, 200}, {H_Do, 200},
    {LaFreq, 600}, // 这里的La长一点，作为结束

    {0, 0}
};

void Play_EVA_Start(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, HIGH); // 开机要响亮
    int i = 0;
    while (eva_chorus[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, eva_chorus[i].freq);
        vTaskDelay(eva_chorus[i].duration); // 阻塞式播放
        i++;
    }
    Buzzer_stop(buzzer);
}

// 快速升调音效
static const Note_t cyber_boot[] = {
    // 阶段1：底层自检 (低音快速爬升)
    {DoFreq, 50}, {ReFreq, 50}, {MiFreq, 50}, {FaFreq, 50}, {SoFreq, 50}, {LaFreq, 50}, {SiFreq, 50},

    // 阶段2：核心模块上线 (高音快速爬升)
    {DoHighFreq, 40}, {ReHighFreq, 40}, {MiHighFreq, 40}, {FaFreq * 2, 40}, {SoFreq * 2, 40}, // 简单的倍频模拟更高音

    // 阶段3：系统就绪 (三次短促的高频确认音)
    {0, 100}, // 停顿一下
    {DoHighFreq, 80}, {0, 50},
    {DoHighFreq, 80}, {0, 50},
    {MiHighFreq, 400}, // 最后一声长鸣，表示Ready

    {0, 0}
};

void Play_Cyber_Boot(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, MEDIUM); // 这种高频音效不用太大声就很刺耳
    int i = 0;
    while (cyber_boot[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, cyber_boot[i].freq);
        vTaskDelay(cyber_boot[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

/***************** 补充半音定义 (放在.c文件顶部) *****************/
#define DoSharp  554
#define ReSharp  622
#define FaSharp  740
#define SoSharp  831
#define LaSharp  932
#define DoHighSharp 1109
#define ReHighSharp 1245
#define FaHighSharp 1480
#define SoHighSharp 1661

/***************** 方案一：Only My Railgun (副歌高潮版) *****************/
// 节奏：极快 (BPM ~145)
static const Note_t railgun_notes[] = {
    // Ha-na-te (释放!)
    {LaFreq, 150}, {LaFreq, 150}, {LaFreq, 150},
    // Ko-ko-ro (心)
    {SoFreq, 150}, {FaFreq, 150}, {SoFreq, 150},
    // ni (在)
    {LaFreq, 300},
    // ki-za-n-da (刻下)
    {DoHighFreq, 150}, {SiFreq, 150}, {LaFreq, 150}, {SoFreq, 150},
    // yu-me (梦想)
    {FaFreq, 150}, {SoFreq, 300},
    // wo (的)
    {LaFreq, 150}, {LaFreq, 400},

    // mi-ra-i (未来)
    {DoHighFreq, 150}, {DoHighFreq, 150}, {DoHighFreq, 150},
    // sa-e (甚至)
    {ReHighFreq, 150}, {DoHighFreq, 150},
    // o-ki-za-ri (抛在脑后)
    {SiFreq, 150}, {LaFreq, 150}, {SoFreq, 150}, {LaFreq, 150},
    // ni (的)
    {SiFreq, 300},
    // shi-te (做)
    {SoFreq, 150}, {DoHighFreq, 400},

    // 结尾拖长音，表示系统就绪
    {0, 0}
};

void Play_Railgun(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, MEDIUM); // 节奏快，中音量更清晰
    int i = 0;
    while (railgun_notes[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, railgun_notes[i].freq);
        // 这首歌节奏很快，为了听感连续，可以把Delay稍微减少一点点(比如乘0.9)，或者直接用
        vTaskDelay(railgun_notes[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

/***************** 方案二：JoJo 黄金之风 (那个最帅的钢琴段) *****************/
// 节奏：强劲有力
static const Note_t jojo_notes[] = {
    // 标志性的前奏：Jo Jo ~ Golden Wind
    {SiFreq, 200}, {ReHighFreq, 200},
    {0, 100}, // 停顿增加节奏感
    {ReHighFreq, 150}, {MiHighFreq, 150}, {FaHighSharp, 150}, {SoHighSharp, 150},
    {FaHighSharp, 150}, {MiHighFreq, 150}, {ReHighFreq, 150}, {DoHighSharp, 150},

    // 重复洗脑旋律
    {SiFreq, 150}, {SiFreq, 150}, {LaFreq, 150}, {SiFreq, 400},
    {0, 200},

    // 结尾爆发
    {ReHighFreq, 150}, {SiFreq, 150}, {FaHighSharp, 600},

    {0, 0}
};

void Play_JoJo(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, HIGH); // JoJo必须大声！
    int i = 0;
    while (jojo_notes[i].duration != 0)
    {
        // 对于0频率（休止符），需要停止输出但保持延时
        if (jojo_notes[i].freq == 0)
        {
            Buzzer_set_frequency(buzzer, 0);
        }
        else
        {
            Buzzer_set_frequency(buzzer, jojo_notes[i].freq);
        }
        vTaskDelay(jojo_notes[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

// 只需要这四个音，识别度满分
static const Note_t sans_intro[] = {
    {ReFreq, 80},       // D (短)
    {ReFreq, 80},       // D (短)
    {ReHighFreq, 160},  // High D (长)
    {LaFreq, 200},      // A (长，带一点压迫感)

    // 如果想稍微多一点，可以加上后面这个 (可选，不想太长就删掉下面这行)
    // {SoSharp, 150}, // G# (需要之前的半音宏定义，没有定义的话可以用 SoFreq 代替)

    {0, 0}
};

void Play_Sans(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, MEDIUM);
    int i = 0;
    while (sans_intro[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, sans_intro[i].freq);
        vTaskDelay(sans_intro[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

// 经典的五音节：Du Du Du Du Du
static const Note_t intel_jingle[] = {
    {ReFreq, 80},       // D
    {ReFreq, 80},       // D
    {SoFreq, 80},       // G
    {ReFreq, 80},       // D
    {LaFreq, 250},      // A (尾音拉长)
    {0, 0}
};

void Play_Intel(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, HIGH); // 广告音就是要清脆响亮
    int i = 0;
    while (intel_jingle[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, intel_jingle[i].freq);
        vTaskDelay(intel_jingle[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

// 补充一些必要的半音 (如果之前没加的话)
#define ReHighSharp 1245
#define FaHighSharp 1480
#define SoSharp  831
#define LaSharp  932

/***************** 方案一：黑人抬棺 (Astronomia) *****************/
// 这是一个洗脑的 Loop
static const Note_t coffin_dance[] = {
    // 前奏铺垫
    {FaFreq, 100}, {SoFreq, 100}, {LaFreq, 100}, {SoFreq, 100},
    {DoHighFreq, 100}, {SiFreq, 100}, {LaFreq, 100}, {SoFreq, 100},

    // 注入灵魂的主旋律 (Re-Re-Re-La...)
    {FaFreq, 200}, {SoFreq, 200}, {DoHighFreq, 400},
    {SiFreq, 200}, {LaFreq, 200}, {SoFreq, 200}, {LaFreq, 200},
    {SiFreq, 400}, // 那个魔性的转折

    // 快速结尾
    {DoHighFreq, 100}, {SiFreq, 100}, {LaFreq, 100}, {SoFreq, 100}, {FaFreq, 400},

    {0, 0}
};

void Play_Coffin_Dance(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, HIGH); // 必须大声，气势要足
    int i = 0;
    while (coffin_dance[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, coffin_dance[i].freq);
        vTaskDelay(coffin_dance[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

/***************** 方案二：Windows XP 关机音效 *****************/
// 旋律：Eb - Bb - F - F# (大概是这个感觉，降调比较慵懒)
static const Note_t win_shutdown[] = {
    {ReHighSharp, 300}, // 高音Eb
    {LaSharp, 300},     // Bb
    {FaFreq, 300},      // F
    {ReFreq, 600},      // Low Eb (低沉的结尾)

    {0, 0}
};

void Play_Windows_Shutdown(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 

    Buzzer_set_volume(buzzer, MEDIUM);
    int i = 0;
    while (win_shutdown[i].duration != 0)
    {
        Buzzer_set_frequency(buzzer, win_shutdown[i].freq);
        vTaskDelay(win_shutdown[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}

/***************** 方案三：马里奥 死亡/Game Over *****************/
// 急促的下降音阶
static const Note_t mario_die[] = {
    {SiFreq, 100},
    {FaHighSharp, 100}, // 需要定义 FaHighFreq (1397) 或者用 FaFreq*2
    {0, 50}, // 稍微停顿
    {FaHighSharp, 100},
    {FaFreq, 100},
    {MiFreq, 100},
    {ReFreq, 100},
    {DoFreq, 100},
    {0, 0}
};
// 注：如果你懒得定义HighFa，可以直接把上面的FaHighFreq改成 FaFreq*2

void Play_Mario_Die(Buzzer_device_t *buzzer)
{
    Buzzer_start(buzzer); // 
    Buzzer_set_volume(buzzer, MEDIUM);
    int i = 0;
    while (mario_die[i].duration != 0)
    {
        if (mario_die[i].freq == 0)
            Buzzer_set_frequency(buzzer, 0);
        else
            Buzzer_set_frequency(buzzer, mario_die[i].freq);

        vTaskDelay(mario_die[i].duration);
        i++;
    }
    Buzzer_stop(buzzer);
}