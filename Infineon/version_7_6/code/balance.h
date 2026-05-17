/*
 * balance.h
 *
 *  Created on: 2022年12月7日
 *      Author: 11150
 */

#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_
#include "zf_common_typedef.h"


void balanceinit(void);
void motor_alone(float pwm_alone);

void stop_car(void);
float Angle_PID(float Angle_Data,float Angle_m,float balance_angle);//角度环
float Gyro_PID(float Gyro_Data,float Gyro_m);   //角速度环
float Speed_PID(float Speed_New,float Speed_m,int speeed_low);//速度环
float Speed_Measure(void);
typedef struct
{




          //zc 直立控制

          float fgyrokp;//p
          float fgyroki;//p

          float flywheelkp;//d
          float flywheelkd;//1.7f


          float flymotorKp;
          float flymotorKi;
          float F_zc_Tzero;

          float I_zc_speed;
          float I_zc_output;



 }_pid_typedef;


 extern _pid_typedef pid;
 void PID_init(void);
 void keyinit(void);
 void scankey(void);
 void change_page(void);
 //void change();

 //void CTRL_compute_AngleY(void);
 //void CTRL_compute_SpeedY();
// float P_balance_Control(float Angle,float Angle_Zero,float Gyro);
 //float Velocity_Control_A(int encoder);

 float flywheel_speedL(float Angle,float Angle_m);
 float Gyro_PIDL(float Gyro_Data,float Gyro_m);
 float fspeedL(float INencoder);//飞轮速度闭环控制


 void shuchu1(void);
 void motorAB_set(float pwm,float pwm1);          //balance



 void flash_init(void);

 float turn_gryo(float gyro_z,float jiaodu);//转向角速度环
 float turn_angle(float error_of_CameraOrBalance,float sudu);//转向角度环
 float turn_speed(float INencoder);//转向速度闭环控制

 float jueduizhi(float err);
 void protect(float fy,float fg);
#endif /* CODE_BALANCE_H_ */
