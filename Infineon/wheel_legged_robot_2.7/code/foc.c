/*
 * foc.c
 *
 *  Created on: 2025年2月27日
 *      Author: 17104
 */

#include "foc.h"
#include <universal_filter.h>

float temp_Ia,temp_Ib;
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);  //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + _2PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

void foc_init(foc_struct *foc,pwm_channel_enum input_A,pwm_channel_enum input_B,pwm_channel_enum input_C,menc15a_module_enum encoder,direction dir,
        float batter_voltage,float supply_voltage,int Pair_of_Poles,float pit_time_us)
{
    menc15a_init();
    foc -> encoder = encoder;
    foc -> A = input_A;
    foc -> B = input_B;
    foc -> C = input_C;
    pwm_init(input_A, 20000, 1500);
    pwm_init(input_B, 20000, 0);
    pwm_init(input_C, 20000, 0);
    foc -> batter_voltage = batter_voltage;
    foc -> supply_voltage = supply_voltage;
    foc -> Pair_of_Poles = Pair_of_Poles;
    foc -> pit_time_us = pit_time_us;
    foc -> dir = dir;
    foc -> capacitor_charging_timer = 0;
    foc -> capacitor_discharging_timer = 0;
    foc -> capacitor_charging_flag = 0;
//    foc -> now_sector = 0;
    foc -> prev_sector = 0;
    system_delay_ms(500);
    uint32 i, offset = 0;
    for(i = 0; i < 1024; i++)
    {
        offset += menc15a_get_absolute_data(foc -> encoder);
    }
    pwm_set_duty(foc -> A, 0);
    foc -> offset_me = offset >> 10;
        foc -> location = menc15a_get_absolute_data(foc -> encoder);
    foc -> prev_location = menc15a_get_absolute_data(foc -> encoder);
}
float foc_get_speed(foc_struct *foc)
{
    foc -> location = menc15a_get_absolute_data(foc -> encoder);
    int32 offset_location = 0;
    if((32768 / 2) < abs(foc -> location - foc -> prev_location))
    {
        offset_location = ((32768 / 2) < foc -> location ? (foc -> location - 32768 - foc -> prev_location) : (foc -> location + 32768 - foc -> prev_location));
    }
    else
    {
        offset_location = (foc -> location - foc -> prev_location);
    }
    foc -> prev_location = foc -> location;
    return (float)offset_location * _2PI * 1000 / 32768  / pit_time0_ms;//rad/s
}
/*Uq[0] = -1.7f * (
KL[1] * (input.actual_target_speed - Instantaneous_speed) + // 速度误差项
KL[2] * imu.roll + // 姿态补偿
KL[3] * now_gx // 角加速度补偿
);*/
void foc_openloop_update(foc_struct *foc,float Uq)
{

    if(Uq > 5.0)Uq = 5.0;
    else if(Uq < -5.0) Uq = -5.0;
    //换算电角度
    uint32 angle_me = (uint32)menc15a_get_absolute_data(foc -> encoder);
    float angle_el = ((float)(angle_me < foc -> offset_me ? angle_me + 32768 - foc -> offset_me : angle_me - foc -> offset_me) * foc -> Pair_of_Poles) / 32768 * _2PI;
    if(foc -> dir == reversal){angle_el = _2PI - angle_el;}


//    float speed = (float)menc15a_absolute_offset_data[foc -> encoder] / 32768 * _2PI * 1000 / foc -> pit_time_us;//rad/s

    if(Uq < 0){angle_el = _normalizeAngle(angle_el - _PI_2);Uq *= -1;}
    else{angle_el = _normalizeAngle(angle_el + _PI_2);}
//    angle_el = _2PI - angle_el;
    int sector = floor(angle_el / _PI_3) + 1;
    if (sector != foc->prev_sector) {
           // 换相时关闭所有PWM
           pwm_set_duty(foc->A, 0);
           pwm_set_duty(foc->B, 0);
           pwm_set_duty(foc->C, 0);

           system_delay_us(1); // 阻塞等待2μs（需实现微秒延时）

           foc->prev_sector = sector;
       }
//    printf("%f,%d\n",angle_el,sector);
    // calculate the duty cycles
    float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / foc -> supply_voltage;
    float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / foc -> supply_voltage;
    float T0 = 1 - T1 - T2;

    float Ta, Tb, Tc;
    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 2:
        Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 3:
        Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
        break;
    case 4:
        Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 5:
        Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 6:
        Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
        break;
    default:
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    float Ua = Ta * foc -> supply_voltage;
    float Ub = Tb * foc -> supply_voltage;
    float Uc = Tc * foc -> supply_voltage;
//    printf("%f,%f,%f\n",Ua,Ub,Uc);

    // 限制上限
    if(Ua > foc -> supply_voltage || Ub > foc -> supply_voltage || Uc > foc -> supply_voltage)
    Ua = _constrain(Ua, 0.0f, foc -> supply_voltage);
    Ub = _constrain(Ub, 0.0f, foc -> supply_voltage);
    Uc = _constrain(Uc, 0.0f, foc -> supply_voltage);
      // 计算占空比
      // 限制占空比从0到1
    uint32 dc_a = Ua / foc -> batter_voltage * PWM_DUTY_MAX;
    uint32 dc_b = Ub / foc -> batter_voltage * PWM_DUTY_MAX;
    uint32 dc_c = Uc / foc -> batter_voltage * PWM_DUTY_MAX;

//    printf("%lu,%lu,%lu\n",dc_a,dc_b,dc_c);
//    if(dc_a > PWM_DUTY_MAX * 0.85f){dc_a = PWM_DUTY_MAX * 0.85f;}
//    if(dc_b > PWM_DUTY_MAX * 0.85f){dc_b = PWM_DUTY_MAX * 0.85f;}
//    if(dc_c > PWM_DUTY_MAX * 0.85f){dc_c = PWM_DUTY_MAX * 0.85f;}
    dc_a = _constrain(dc_a,PWM_DUTY_MAX * 0.05f,PWM_DUTY_MAX * 0.85f);
    dc_b = _constrain(dc_b,PWM_DUTY_MAX * 0.05f,PWM_DUTY_MAX * 0.85f);
    dc_c = _constrain(dc_c,PWM_DUTY_MAX * 0.05f,PWM_DUTY_MAX * 0.85f);

    // 在低占空比时强制充电
    if (dc_a < PWM_DUTY_MAX * 0.1 || dc_b < PWM_DUTY_MAX * 0.1 || dc_c < PWM_DUTY_MAX * 0.1)
    {
        // 每100ms插入1ms全关断窗口
        foc -> capacitor_discharging_timer += pit_time3_us;
//        static uint32 last_charge = 0;
        if (foc -> capacitor_discharging_timer >= 100 * 1000)
        {
            foc -> capacitor_charging_flag = 1;
            foc -> capacitor_charging_timer = 0;
            foc -> capacitor_discharging_timer = 0;
       }
    }

    if(foc -> capacitor_charging_flag == 1)
    {
        pwm_set_duty(foc->A, 0);
        pwm_set_duty(foc->B, 0);
        pwm_set_duty(foc->C, 0);
        foc -> capacitor_charging_timer += pit_time3_us;
        if (foc -> capacitor_charging_timer >= 1 * 1000)
        {
            foc -> capacitor_charging_flag = 0;
            foc -> capacitor_charging_timer = 0;
            foc -> capacitor_discharging_timer = 0;
        }
    }
    else
    {
        pwm_set_duty(foc -> A, dc_a);
        pwm_set_duty(foc -> B, dc_b);
        pwm_set_duty(foc -> C, dc_c);
    }

//    printf("%f,%lu,%f\n",foc->Ia.output,dc_a,(float)adc_convert(foc->Ia.adc_pin) - foc->Ia.offset);
//    return speed;rear_angle
}
