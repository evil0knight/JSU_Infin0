/*
 * motor_pid_duty.c
 *
 *  Created on: 2024年9月29日
 *      Author: 17104
 */

#include "PID.h"

//位置PID结构体声明示例:
//place_PID_struct Place;

//位置PID结构体初始化
//示例:place_pid_init(&Place,0,0,0,0,100,50);//kp,ki,kd,target,limit,separate
void place_pid_init(place_PID_struct * Place,float Kp,float Ki,float Kd,float target,float limit,float separate)
{
    Place -> Kp = Kp;//比例增益
    Place -> Ki = Ki;//积分增益
    Place -> Kd = Kd;//微分增益

    Place -> target_data = target;//目标数据

    Place -> E_integral_limit = limit;//积分限幅
    Place -> E_integral_separate = separate;//积分分离

    Place -> now_data = 0;//实时数据初始化
    Place -> E_integral = 0;//积分初始化
    Place -> last_E = 0;//上一次误差初始化
    Place -> output = 0;//输出初始化
}

//位置PID计算函数
//示例:place_pid_update(&Place);
void place_pid_update(place_PID_struct * Place, float E_differentiation)
{
    float E = Place -> target_data - Place -> now_data;//计算误差
    Place -> E_integral += Place -> last_E;//计算积分
//    float E_differentiation = (E - Place -> last_E);//计算微分

    if(Place -> E_integral > Place -> E_integral_limit) Place -> E_integral = Place -> E_integral_limit;//积分限幅
    else if(Place -> E_integral < (-1) * Place -> E_integral_limit) Place -> E_integral = (-1) * Place -> E_integral_limit;
    if(E > Place -> E_integral_separate) Place -> E_integral = 0;//积分分离
    else if(E < (-1) * Place -> E_integral_separate) Place -> E_integral = 0;

    Place -> output = Place -> Kp * E + Place -> Ki * Place -> E_integral + Place -> Kd * E_differentiation;//计算输出

    Place -> last_E = E;//更新误差
}

//位置PID结构体声明示例:
//increment_PID_struct Place;

//增量PID结构体初始化
//示例:increment_pid_init(&Place,0,0,0,0,100);//kp,ki,kd,target,limit
void increment_pid_init(increment_PID_struct * Place,float Kp,float Ki,float Kd,float target,float limit)
{
    Place -> Kp = Kp;//比例增益
    Place -> Ki = Ki;//积分增益
    Place -> Kd = Kd;//微分增益

    Place -> target_data = target;//目标数据

    Place -> increment_limit = limit;//增量限幅

    Place -> now_data = 0;//实时数据初始化
    Place -> last_E = 0;//上一次误差初始化
    Place -> last_last_E = 0;//上上一次误差初始化
    Place -> output = 0;//输出初始化
}

//增量PID计算函数
//示例:increment_pid_update(&Place);
void increment_pid_update(increment_PID_struct * Place)
{
    float E = Place -> target_data - Place -> now_data;//计算误差
    float E_differentiation = (E - 2 * Place -> last_E + Place -> last_last_E);//计算微分

    float result = Place -> Kp * (E) + Place -> Ki * E + Place -> Kd * E_differentiation;//计算增量

    if(result > Place -> increment_limit) result = Place -> increment_limit;//增量限幅
    if(result < (-1) * Place -> increment_limit) result = (-1) * Place -> increment_limit;

    Place -> last_E = E;//更新误差
    Place -> last_last_E = Place -> last_E;

    Place -> output += result;//输出更新
}
