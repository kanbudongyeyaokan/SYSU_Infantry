#include "buzzer_driver.h"
#include "tim.h"
#include <string.h>
#include "main.h"

// 音量级别对应的占空比映射
static const float volume_duty_cycle[] = {
    [VERY_LOW] = 0.10f,  // 5%
    [LOW]      = 0.30f,  // 30%
    [MEDIUM]   = 0.50f,  // 50%
    [HIGH]     = 0.70f,  // 70%
    [VERY_HIGH]= 0.90f   // 90%
};

/**
 * @brief 蜂鸣器初始化
 *
 * @param buzzer 蜂鸣器实例指针
 * @param htim 定时器句柄
 * @param tim_channel 定时器通道
 */
void Buzzer_init(Buzzer_device_t *buzzer, TIM_HandleTypeDef *htim, uint32_t tim_channel)
{

    //清零
    memset(buzzer, 0, sizeof(Buzzer_device_t));
    // 保存配置参数
    buzzer->htim = htim;
    buzzer->tim_channel = tim_channel;
    // 启动PWM输出
    HAL_TIM_PWM_Start(buzzer->htim, buzzer->tim_channel);
    // 设置默认音量
    buzzer->buzzer_volume = MEDIUM;
    //开启蜂鸣器
    buzzer->buzzer_state = BUZZER_ON;
    // 初始化为静音
    Buzzer_set_frequency(buzzer, 0);
}

/**
 * @brief 设置蜂鸣器频率
 *
 * @param buzzer 蜂鸣器实例指针
 * @param freq 频率值(Hz), 0表示静音
 */
void Buzzer_set_frequency(Buzzer_device_t *buzzer, uint32_t freq)
{
   // if (freq == 0 || buzzer->buzzer_state == BUZZER_OFF) {
   //     // 静音 - 停止PWM输出
   //     HAL_TIM_PWM_Stop(buzzer->htim, buzzer->tim_channel);
   //     return;
  //  }
    //不停止PWM输出，而是设置占空比为0实现静音，防止状态管理混乱
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(buzzer->htim, buzzer->tim_channel, 0);
        return;
    }
    // 计算自动重载值 (ARR)
    uint32_t period = (TIMER_CLOCK_FREQ / freq) - 1;

    // 配置定时器
    __HAL_TIM_SET_AUTORELOAD(buzzer->htim, period);

    // 计算比较值 (CCR)
    uint32_t ccr = (uint32_t)(period * volume_duty_cycle[buzzer->buzzer_volume]);
    __HAL_TIM_SET_COMPARE(buzzer->htim, buzzer->tim_channel, ccr);

}

/**
 * @brief 设置蜂鸣器音量（简化版）
 *
 * @param buzzer 蜂鸣器实例指针
 * @param volume 音量级别
 */
void Buzzer_set_volume(Buzzer_device_t *buzzer, Buzzer_volume_e volume)
{
    // 保存音量设置
    buzzer->buzzer_volume = volume;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(buzzer->htim);
    uint32_t ccr = (uint32_t)(arr * volume_duty_cycle[volume]);
    __HAL_TIM_SET_COMPARE(buzzer->htim, buzzer->tim_channel, ccr);
}
//开启
void Buzzer_start(Buzzer_device_t *buzzer) {
    buzzer->buzzer_state = BUZZER_ON;
}
//停止蜂鸣器（静音，频率设为 0，但 PWM 保持运行）
void Buzzer_stop(Buzzer_device_t *buzzer) {
    buzzer->buzzer_state = BUZZER_OFF;
    Buzzer_set_frequency(buzzer, 0);  // 静音，但不停止 PWM
}
