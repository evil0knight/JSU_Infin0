/*
 * foc.h
 *
 *  Created on: 2025Äê2ÔÂ27ÈÕ
 *      Author: 17104
 */

#ifndef CODE_FOC_H_
#define CODE_FOC_H_

#include "zf_common_headfile.h"
//#include <PID.h>
//#include "INA240A2.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
//float voltage_power_supply;
// float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
#define _2PI 6.28318530718f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

typedef enum{
    corotation,
    reversal
}direction;


typedef struct{
//    INA240A2_strust Ia;
//    INA240A2_strust Ib;
    menc15a_module_enum encoder;
    uint32 offset_me;
    pwm_channel_enum A;
    pwm_channel_enum B;
    pwm_channel_enum C;
    int Pair_of_Poles;
    float batter_voltage;
    float supply_voltage;
    float pit_time_us;
    int32 location;
    int32 prev_location;
    direction dir;
    int capacitor_charging_timer;
    int capacitor_discharging_timer;
    int capacitor_charging_flag;
    int prev_sector;
//    increment_PID_struct I_loop;
//    KalmanFilter I;


}foc_struct;

extern foc_struct motor_L;

float _normalizeAngle(float angle);
void foc_init(foc_struct *foc,pwm_channel_enum input_A,pwm_channel_enum input_B,pwm_channel_enum input_C,menc15a_module_enum encoder,direction dir,
        float batter_voltage,float supply_voltage,int Pair_of_Poles,float pit_time_ms);
float foc_get_speed(foc_struct *foc);
void foc_openloop_update(foc_struct *foc,float Uq);
#endif /* CODE_FOC_H_ */
