/*
 * motor_pid_duty.h
 *
 *  Created on: 2024年9月29日
 *      Author: 17104
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

//位置PID结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float now_data;
    float target_data;
    float last_E;
    float E_integral;

    float E_integral_limit;
    float E_integral_separate;

    float output;

}place_PID_struct;

//增量PID结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float now_data;
    float target_data;
    float last_E;
    float last_last_E;

    float increment_limit;

    float output;
}increment_PID_struct;

//位置PID函数声明
void place_pid_init(place_PID_struct * Place,float Kp,float Ki,float Kd,float target,float limit,float separate);
void place_pid_update(place_PID_struct * Place, float E_differentiation);

//增量PID函数声明
void increment_pid_init(increment_PID_struct * Place,float Kp,float Ki,float Kd,float target,float limit);
void increment_pid_update(increment_PID_struct * Place);

#endif /* CODE_PID_H_ */
