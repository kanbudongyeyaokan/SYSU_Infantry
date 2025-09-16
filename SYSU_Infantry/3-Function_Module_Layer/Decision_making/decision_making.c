#include "decision_making.h"
//通信
#include "message_center.h"
//状态机获取
#include "robot_definitions.h"
//遥控器控制、
#include "remote_control.h"


//存储遥控器数据，CURRENT-当前数据,LAST-上一次数据
static RC_ctrl_t *rc_data;




/**
 * @brief 根据遥控器左边开关决定机器人是键鼠控制还是遥控器控制,并且调用对应的控制函数
 *
*/
void Robot_set_command()
{
    //左边开关打下，进入遥控器控制模式
    if (rc_data[CURRNET].rc.Lswitch == SWITCH_IS_DOWN)
    {
        RC_ctrl_set();
    }
    //左边开关打上，进入键盘控制模式
    else if (rc_data[CURRNET].rc.Lswitch == SWITCH_IS_UP)
    {
        Keyboard_ctrl_set();
    }
}




/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
*/
void RC_ctrl_set()
{
    /**根据遥控器开关状态设定模式**/

    /**底盘/云台模式设定**/
    //如果右边开关打下，则进入底盘跟随云台模式
    if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_DOWN)
    {

    }
    //如果右边开关打中间，则进入小陀螺模式
    else if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_MID)
    {

    }
    //如果右边开关打上，则进入底盘自由模式，此时底盘不跟随云台
    else if (rc_data[CURRNET].rc.Rswitch == SWITCH_IS_UP)
    {

    }
    /**射击模式设定**/
    //左边拨轮往上打开启摩擦轮
    if (rc_data[CURRNET].rc.dial < -100)
    {

    }
    //正常情况下不打开摩擦轮
    else
    {

    }
    //左边拨轮往上打到底，开始发射子弹
    if (rc_data[CURRNET].rc.dial < -500)
    {

    }
    //正常情况下不发射子弹
    else
    {

    }

    /****************控制量设定*****************/
    //底盘控制量




    //云台控制量

}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
*/
void Keyboard_ctrl_set()
{


}

