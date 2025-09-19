## DWT库的使用说明

## 使用步骤
>1.在MAIN函数调用DWT_Init()函数，如果是C板，C板主频为168MHZ，则在MAIN函数初始化部分写DWT_Init(168);
>2.初始化完毕之后就可以调用库里的接口

## 接口说明
用于计算两次进入同一个函数的时间差
>float DWT_GetDeltaT(uint32_t *cnt_last);
使用案例：
```c
static uint32_t cnt;
float deltaT;

deltaT=DWT_GetDeltaT(&cnt);
```

获取当前时间,单位为秒,即初始化后的时间
>float DWT_GetTimeline_s(void);
使用范例：
### 计算执行某部分代码的耗时
```c
float start,end;
start=DWT_DetTimeline_ms();

// some proc to go... 
for(uint8_t i=0;i<10;i++)
 foo();

end = DWT_DetTimeline_ms()-start;
```

毫秒级别的延时
>void DWT_delay_ms(uint32_t delay_ms);

微秒级别的延时
>void DWT_delay_us(uint32_t delay_us);