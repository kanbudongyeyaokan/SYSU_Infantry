#ifndef SYSU_INFANTRY_SHOOT_H
#define SYSU_INFANTRY_SHOOT_H

#include "dji_motor.h"
//电机初始化
void Shoot_motors_init(void);

//发射任务初始化
void Shoot_task_init(void);

//发射任务处理控制命令
void Shoot_handle_command(void);
#endif //SYSU_INFANTRY_SHOOT_H