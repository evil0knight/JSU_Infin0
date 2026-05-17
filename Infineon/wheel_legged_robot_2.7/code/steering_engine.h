/*
 * steering_engine.h
 *
 *  Created on: 2024Äê11ÔÂ16ÈÕ
 *      Author: 17104
 */

#ifndef CODE_STEERING_ENGINE_H_
#define CODE_STEERING_ENGINE_H_

//#define PI 3.1415926

#include "zf_common_headfile.h"

typedef struct
{
    pwm_channel_enum pwm_pin;
    uint32 freq;
    uint32 duty_now;
    uint32 duty_min;
    uint32 duty_max;
//    float angle_min;
//    float angle_max;
    float angle_err;
}steering_engine_struct;

void steering_engine_angle_conversion(steering_engine_struct * steer,float angle);
uint32 conversion(float angle);
void steering_engine_hardware_init(steering_engine_struct * steer);
void steering_engine_init(steering_engine_struct * steer,pwm_channel_enum pwm,uint32 freq,
float angle_init,float min_angle,float max_angle,float input_err);
void steering_engine_run(steering_engine_struct * steer,float angle);

#endif /* CODE_STEERING_ENGINE_H_ */
