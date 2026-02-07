/**
* @Author : SYSU电控组
* @Date   : 2025-12-1
* @Note   : C板BMI088驱动库 (适配标准INS接口 + BSP_SPI)
*/
#ifndef BMI088_H
#define BMI088_H

#include "ins.h"       // 必须包含，因为我们要实现这个接口
#include "bsp_spi.h"   // 依赖底层BSP
#include "main.h"      // 获取GPIO定义

// ================= 配置部分 (C板默认引脚) =================
// 如果需要改引脚，去 .c 文件或者通过 Config 结构体传入
// 这里为了 ins_task 简洁，我们默认 C 板定义

// ================= API =================

/**
 * @brief 获取 BMI088 的标准驱动接口实例
 * @note  调用此函数会返回一个符合 Ins_driver_interface_t 的指针，
 * 可以直接传给 Ins_init()。
 */
const Ins_driver_interface_t* BMI088_Get_Driver(void);

// 辅助功能 (如果任务层还需要读温度做其他控制)
float BMI088_Get_Temp_Raw(void);

#endif // BMI088_H