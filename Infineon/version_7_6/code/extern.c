/*
 * extern.c
 *
 *  Created on: 2023Äê6ÔÂ7ÈÕ
 *      Author: 11150
 */




#include "zf_common_headfile.h"
//#include "function_basic.h"

#define dt 0.01f
float gz;
float gy;
float yaw_angle;
float pitch_angle;

float speed_j;
float distance;

uint8 icm20602_yaw_state;
uint8 icm20602_pitch_state;
uint8 distance_state;


/*void road_off(void)
{
    int road_off_num=0;
     uint8 row=119;
//      for(uint8 row=119;row>110;row--)
//      {
            for(uint8 column=0;column<187;column++)
            {
                if(Pixle[row][column]==0x00)
                {road_off_num++;}
            }
        if((road_off_num>150||Threshold<140)&&roadblock_state==0)//&&roundabout_state==0)
        {
//          ips114_show_string(190, 80, "O0");
      ips200_show_string(200, 30, "O0");
            deviation_elements_state=2;
            deviation_elements=0;
        }
}*/










extern int16 zero_data;

void get_angle(uint8 mode)
{
     icm20602_get_gyro();
    switch(mode)
    {
        case 1:
     gz = icm20602_gyro_transition(icm20602_gyro_z-zero_data);
   yaw_angle+=gz*dt;      break;

        case 2:
     gy = icm20602_gyro_transition(icm20602_gyro_y);
   pitch_angle+=gy*dt;   break;

        default:  break;
    }
}


extern float bmq;
void get_distance(void)
{
      speed_j=-bmq;
        distance+=speed_j*dt;
}









void pit_handle(void)
{
        if(icm20602_yaw_state==1)          {get_angle(1);}
         if(icm20602_pitch_state==1)   {get_angle(2);}
        if(distance_state==1)         {get_distance();}
}





