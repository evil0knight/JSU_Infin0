/*
 * steering_engine.c
 *
 *  Created on: 2024年11月16日
 *      Author: 17104
 */

#include <steering_engine.h>

//舵机角度换算（内部调用）
void steering_engine_angle_conversion(steering_engine_struct * steer,float angle)
{
    steer -> duty_now = (uint32) (250.0 + (angle + steer -> angle_err) / 180.0 *1000.0) * 4;
//    steer -> duty_now = (uint32) ((15.0 + (angle + steer -> angle_err) / 3.0) * 100.0);
}

//舵机角度换算（内部调用）
uint32 conversion(float angle)
{
    return (uint32)(250.0 + angle / 180.0 * 1000.0) * 4;
//    return (uint32)((15.0 + angle / 3.0) * 100.0);
}

//舵机硬件初始化（内部调用）
void steering_engine_hardware_init(steering_engine_struct * steer)
{
    pwm_init(steer -> pwm_pin, steer -> freq, steer -> duty_now);
}

//舵机结构体声明示例:
//steering_engine_struct steer;

//舵机初始化
//示例：steering_engine_init(&steer,ATOM1_CH7_P20_11,50,90.0,0.0,180.0,-7.0);
void steering_engine_init(steering_engine_struct * steer,pwm_channel_enum pwm,uint32 freq,
        float angle_init,float min_angle,float max_angle,float input_err)
{
    steer -> pwm_pin = pwm;//pwm引脚初始化
    steer -> freq = freq;//舵机pwm频率初始化
    steer -> duty_now = conversion(angle_init + input_err);//舵机占空比初始化
    steer -> duty_min = conversion(min_angle + input_err);//舵机占空比最小值
    steer -> duty_max = conversion(max_angle + input_err);//舵机占空比最大值
    steer -> angle_err = input_err;//舵机机械误差
    steering_engine_hardware_init(steer);//舵机硬件初始化
}

//舵机状态更新
//示例：steering_engine_run(&steer,2);
void steering_engine_run(steering_engine_struct * steer,float angle)
{
    steering_engine_angle_conversion(steer,angle);//舵机角度换算

    if(steer -> duty_now > steer -> duty_max) steer -> duty_now = steer -> duty_max;//占空比限幅
    else if(steer -> duty_now < steer -> duty_min) steer -> duty_now = steer -> duty_min;

//    printf("%f,%lu\n",angle,steer -> duty_now);
    pwm_set_duty(steer -> pwm_pin, steer -> duty_now);//pwm引脚状态更新
//    printf("%lu\n",steer -> duty_now);
//    printf("444\n");
}
