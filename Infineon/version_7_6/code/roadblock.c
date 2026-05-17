/*
 * roadblock.c
 *
 *  Created on: 2023年6月18日
 *      Author: 11150
 */


#include "extern.h"

#include "zf_common_headfile.h"
#include"roundabout.h"
#include"rampway.h"
#include"garage.h"
#include "stdlib.h"

uint8 roadblock_state;
int roadblock_Pixle_num;
uint8 roadblock_Pixle_flag;

//uint8 broken_road_state;
int broken_road__Pixle_num;
uint8 broken_road_Pixle_flag;


int broken_road_num;
uint8 broken_road_flag;
uint8 broken_road_state;



//void roadblock_electricity_judge(void)
//void roadblock_electricity_control(void)
extern float speed_alone;



//extern int jishi;





float jiaodu_kp=0.4;
float jiaodu_kd=0;
float rampway_turn(float true_jiaodu,float jiaodu)//转向角速度环//向左z轴为正  外环向左也应该为正
{

  static float PWM_out=0;
  static float error=0;
    static float last_error=0;


    error=true_jiaodu-jiaodu;
  PWM_out = jiaodu_kp*error+jiaodu_kd*(error-last_error);
  last_error=error;


   if(PWM_out>=30)
        PWM_out=30;
    if(PWM_out<=-30)
        PWM_out=-30;

  return PWM_out;
}



float jiaodu_kp1=0.25;
float jiaodu_kd1=0;
float rampway_turn1(float true_jiaodu,float jiaodu)//转向角速度环//向左z轴为正  外环向左也应该为正
{

  static float PWM_out=0;
  static float error=0;
    static float last_error=0;


    error=true_jiaodu-jiaodu;
  PWM_out = jiaodu_kp1*error+jiaodu_kd1*(error-last_error);
  last_error=error;


   if(PWM_out>=30)
        PWM_out=30;
    if(PWM_out<=-30)
        PWM_out=-30;

  return PWM_out;
}



uint8 white_flag;//白点返回标志
uint8 white_op_flag;//是否开启白点计数
int whiet_Pixle_num;



extern float piao_angle;
float angle3;//目标
float angle4;//目前
float angle5;//目标-目前
extern float pwm_alone;
extern float GyroP;////0.52;//2;//1; //
extern float GyroI;

extern int16 x_distance;
extern int16 y_distance;
extern float yaw_angle1;
