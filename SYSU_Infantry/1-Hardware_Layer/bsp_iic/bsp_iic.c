/* bsp_iic.c */
#include "bsp_iic.h"
#include "hwt606_iic.h"
#include "hwt101_iic.h" 

extern I2C_HandleTypeDef hi2c2;

/**
 * @brief HAL库 I2C 接收完成回调 
 * 这是唯一的入口，不要在其他地方再定义这个函数了！
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) 
    {
        // 尝试分发给 HWT606
        HWT606_RxCpltCallback(hi2c);
        
        // 如果有 HWT101，也分发给它 
       // HWT101_RxCpltCallback(hi2c);
    }
}