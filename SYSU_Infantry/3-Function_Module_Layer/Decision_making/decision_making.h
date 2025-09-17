#ifndef SYSU_INFANTRY_DECISION_MAKING_H
#define SYSU_INFANTRY_DECISION_MAKING_H

#include "main.h"

/**************决策*****************/

/*机器人控制来源------键鼠/遥控器*/
#define RC_CTRL 0
#define KEYBOARD_CTRL 1

/******************决策任务发送给各个模块的信息**********************/
//底盘
typedef struct
{
 
}Chassis_cmd;


/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command();

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
*/
void RC_ctrl_set();

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
static void Keyboard_ctrl_set();

#endif //SYSU_INFANTRY_DECISION_MAKING_H