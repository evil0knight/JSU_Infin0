/*
 * rtk.c
 *
 *  Created on: 2024年6月22日
 *      Author: 11150
 */

#include "rtk.h"
#include "zf_common_headfile.h"
#include "gyro.h"

//#define dt2 0.01//积分时间

double angle_1;//方向角


double speed_rtk;//速度，rtk获取

double angle_2;//方位角


double angle_3;//偏差角，方向角-方位角

double angle_4;//缓冲偏角，变化较angle_3更平稳

int rtk_flagpoint;//打点标志位，为1打点，无需清零，自动清零
//int rtk_flagpoint1;//发车标志位
double rtk_target_distance;//巡点时的距离目标点的距离，rtk经纬度计算获得
double rtk_targets[200][2] ; // 示例目标点列表，之前测377，最多208个点，待商榷
extern int mode;//模式
//int num_targets = sizeof(targets) / sizeof(targets[0]);
int rtk_current_target = 0;
int rtk_define_target = 0; // current_target当前目标点的索引，define_target储存的点索引

void rtk_point_get(void)//模式1：遥控器采点
{
    if ( rtk_flagpoint== 1)//遥控器打点，或者自动打点
    {

            rtk_targets[rtk_define_target][0] = gnss.latitude;
            rtk_targets[rtk_define_target][1] = gnss.longitude;


            printf("x=%f",rtk_targets[rtk_define_target][0]);
            printf("y=%f\n",rtk_targets[rtk_define_target][1]);
            rtk_define_target++;


    }
    rtk_flagpoint=0;//清零



}
/*void erase()//擦除函数，必须在打点和巡点之间调用一次，打点和巡点时不可以调用
{
    yaw_angle1=0;//偏航角置零(模式1与模式2之间操作)+
    x_distance=0;
    y_distance=0;
    current_target=0;

}*/


double rtk_judge_distance=0.3;//0.5m
void rtk_pianjiao_get()//模式2：偏差角获取,当到达最后目标点时自动停车
{
    angle_1=-gnss.antenna_direction-90;//方向角
    angle_2=-get_two_points_azimuth(gnss.latitude,gnss.longitude,rtk_targets[rtk_current_target][0],rtk_targets[rtk_current_target][1]);//方位角
//    angle_2=get_two_points_azimuth2((float)gnss.latitude*1000000,(float)gnss.longitude*1000000,(float)rtk_targets[rtk_current_target][0]*1000000,(float)rtk_targets[rtk_current_target][1]*1000000 );//方位角

    angle_4=0;
    if(angle_1>180)
    {
        angle_1-=360;
    }
    if(angle_1<-180)
    {
        angle_1+=360;
    }
    if(angle_2>180)
    {
        angle_2-=360;
    }
    if(angle_2<-180)
    {
        angle_2+=360;
    }


    angle_3 = angle_2-angle_1;//目标方向（由当前坐标与目标坐标计算求得）


    if(angle_3>180)
    {
        angle_3=angle_3-360;
    }
    if(angle_3<-180)
    {
        angle_3=angle_3+360;
    }

//    if(fabs(angle_4-angle_3)<20)
//    {
//        if(fabs(angle_4-angle_3)<8)
//        {
//            angle_4=angle_3;
//        }
//        else if(angle_4>angle_3)
//        {
//            angle_4-=3;
//        }
//        else if(angle_4<angle_3)
//        {
//            angle_4+=3;
//        }
//    }
//    else
//    {
//        if(angle_4>angle_3)
//        {
//            angle_4-=5;
//        }
//        else if(angle_4<angle_3)
//        {
//            angle_4+=5;
//        }
//    }

    rtk_target_distance=get_two_points_distance(gnss.latitude,gnss.longitude,rtk_targets[rtk_current_target][0],rtk_targets[rtk_current_target][1]);
    if(rtk_target_distance<rtk_judge_distance)//跟随巡点判断
    {
        rtk_current_target++;
    }


    if(rtk_current_target>=rtk_define_target)
    {

        rtk_current_target=0;

        mode=-2;

    }

}

