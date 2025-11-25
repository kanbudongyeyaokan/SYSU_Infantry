# **STM32 DWT 高精度计时库使用手册**

## **1\. 简介 (Introduction)**

本库基于 Cortex-M 内核自带的 **DWT (Data Watchpoint and Trace)** 单元中的 CYCCNT (Cycle Count) 寄存器实现。

### **为什么使用 DWT 而不是 HAL\_GetTick?**

* **精度差异**：HAL\_GetTick 基于 Systick中断，精度通常只有 **1ms**。对于 1kHz (1ms周期) 的电机控制来说，这太粗糙了。DWT 直接对 CPU 时钟周期计数，精度可达 **1/168000000 秒 (约 5.9纳秒)**。  
* **零开销**：读取寄存器仅需几条汇编指令，不需要进入中断，对系统性能影响极小。

## **2\. 核心原理 (Implementation Logic)**

### **2.1 硬件基础**

* **寄存器**：DWT-\>CYCCNT 是一个 32 位向上计数的寄存器，每经过一个 CPU 时钟周期加 1。  
* **溢出特性**：在 168MHz 主频下，2^32 / 168000000 ≈ 25.56 秒。即计数器每 25 秒左右会溢出归零一次。

### **2.2 软件扩展 (64位时间轴)**

为了解决 32 位寄存器只能记录短时间的问题，本库维护了一个 **64位虚拟计数器** (CYCCNT64)。

* **检测溢出**：每次调用获取时间函数时，都会检查 Current\_Count \< Last\_Count。如果成立，说明发生了溢出，Round\_Count 加 1。  
* **计算公式**：Total\_Time \= (Round\_Count \* 2^32 \+ Current\_Count) / CPU\_Freq。

### **2.3 差值计算 (Delta T)**

用于 PID 积分时间计算 (dt)。利用无符号整数减法的**回绕特性**，即使发生溢出也能正确计算差值：

// 假设是 32 位无符号数  
// Count\_Now \= 10 (溢出后), Count\_Last \= 0xFFFFFFF0 (溢出前)  
// dt\_cnt \= 10 \- 0xFFFFFFF0 \= 20 (正确差值)  
float dt \= (uint32\_t)(cnt\_now \- \*cnt\_last) / CPU\_Freq;

## **3\. 文件结构**

* bsp\_dwt.h: 接口声明与结构体定义。  
* bsp\_dwt.c: 核心实现。内部包含静态变量维护时间轴状态。

## **4\. 使用指南 (Usage)**

### **步骤 1: 初始化 (最重要\!)**

**务必在 main 函数的最开始调用**，在任何其他外设（如电机、PID）初始化之前。

// main.c  
int main(void)  
{  
    HAL\_Init();  
    SystemClock\_Config();

    // F407 / F405 填 168  
    // F427 / A板 填 180  
    DWT\_Init(168); 

    // 其他初始化...  
    MX\_CAN1\_Init();  
    DJI\_Motor\_Init(...);  
}

### **步骤 2: 用于 PID 算法 (计算 dt)**

这是本库最主要的应用场景。

// 在 PID 结构体或函数中维护一个上一次计数的变量  
static uint32\_t pid\_last\_cnt \= 0; 

// 首次运行时初始化时间戳  
if (pid\_last\_cnt \== 0\) {  
    DWT\_GetDeltaT(\&pid\_last\_cnt);  
    return;  
}

// 在 PID 计算循环中  
float dt \= DWT\_GetDeltaT(\&pid\_last\_cnt); // 获取距离上次调用的时间间隔(秒)  
// 使用 dt 进行积分/微分计算...

### **步骤 3: 高精度微秒延时**

用于需要纳秒/微秒级精度的驱动（如软件 SPI、I2C 或某些传感器的时序要求）。

DWT\_Delay\_us(50); // 延时 50 微秒  
DWT\_Delay(0.0005f); // 延时 0.5 毫秒

*注意：在 RTOS 任务中，如果是毫秒级长延时，建议仍使用 osDelay 以让出 CPU 权限。DWT\_Delay 是死等延时，会占用 CPU。*

### **步骤 4: 性能测试/代码运行时间测量**

用于测量一段代码执行了多久。

uint32\_t start\_cnt;  
float execute\_time;

// 1\. 记录开始  
DWT\_GetDeltaT(\&start\_cnt);

// 2\. 运行你的代码  
Complex\_Algorithm();

// 3\. 计算耗时  
execute\_time \= DWT\_GetDeltaT(\&start\_cnt);  
printf("Cost: %f s\\r\\n", execute\_time);

## **5\. 注意事项 (Precautions)**

1. **初始化检查**：库内部增加了 dt\_initialized 标志位。如果忘记调用 DWT\_Init，所有计时函数会返回安全值（非零），防止除零错误导致单片机卡死。  
2. **长时间运行**：如果系统长时间（超过25秒）没有任何 DWT 函数被调用，计数器可能会回绕多次导致时间轴丢失。  
   * **建议**：在某个定时任务（如 100Hz 的任务）中调用 DWT\_SysTimeUpdate() 作为一个“心跳”，保持时间轴同步。  
3. **多线程安全**：DWT\_GetDeltaT 和时间轴更新函数内部已使用 \_\_disable\_irq() 临界区保护，可以在中断和主循环中安全混用。

// 推荐在任务中加入心跳  
void WatchDog\_Task(void const \* argument)  
{  
    for(;;)  
    {  
        DWT\_SysTimeUpdate(); // 防止 DWT 溢出计数丢失  
        osDelay(1000);  
    }  
}  
