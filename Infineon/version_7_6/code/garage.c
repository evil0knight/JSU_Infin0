/*
 * garage.c
 *
 *  Created on: 2023年6月18日
 *      Author: 11150
 */
#include "extern.h"
#include "garage.h"

#include "zf_common_headfile.h"
#include"roundabout.h"
#include"roadblock.h"
#include"rampway.h"
#include "gyro.h"
uint8 garage_state;
uint8 garage_count;
uint8 garage_flag;
uint8 garage_left_flag;
uint8 garage_right_flag;

uint8 garage_wide;
uint8 garage_num;
uint8 garage_row;



float speed_alone;//
float straight_speed=-100;
float other_speed=-85;
void speed_control(int error_m)//速度控制
{
//    if(mode==4)
//    {
        if(error_m<4&&error_m>-4)//转向偏差在+-5之间，速度增加
        {
            speed_alone=straight_speed;
        }
        else
            speed_alone=other_speed;
//    }
//    else
//    {
//        speed_alone=-80;
//    }

}






//用于出库的转向pid--实际未用

float chuku_kp=10;
float chuku_kd=1;
float turn_chuku(float z_su,float actual)//转向角速度环//向左z轴为正  外环向左也应该为正
{

  static float PWM_out=0;
  static float error=0;
    static float last_error=0;
  //static float last=0;
//      data_conversion(gyro_Z,gyro_z,0,0, virtual_scope_data);//使用虚拟示波器数据转换函数转换数据，至多同时转换四个数据
//     uart_putbuff(UART_1,virtual_scope_data,10);  //数据转换完成后，使用串口发送将数组的内容发送出去
    error=z_su-actual;
  PWM_out = chuku_kp*error+chuku_kd*(error-last_error);
  last_error=error;
   if(PWM_out>=7000)
        PWM_out=7000;
    if(PWM_out<=-7000)
        PWM_out=-7000;

  return PWM_out;
}




// extern float PWM_Y;






  /*  if(pid_flag==1)
    {

        PWM_Y+=turn_chuku(Get_Gyro_Z(),-200)
    }*/


