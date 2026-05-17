/*
 * gyro.c
 *
 *  Created on: 2022年11月22日
 *      Author: 11150
 */



#include "zf_common_headfile.h"
#include "gyro.h"

#include "karman.h"
#include "balance.h"


#define dt1 0.01f

void jiaodu()

{ float fy,fg;
float Angle1=0.0;//原始值换算角度 原来用的是int
float Angle2=0.0;
      icm20602_get_acc();   // 获取ICM20602的加速度测量数值
      icm20602_get_gyro();  // 获取ICM20602的角速度测量数值




    Angle1 = ((acos((double)icm20602_acc_x * 1.0 / 4096.0) * 180.0 / 3.141593) - 90-0.5);//-3.5;//+10;
    Angle2 = ((acos((double)icm20602_acc_y * 1.0 / 4096.0) * 180.0 / 3.141593) - 90-0.5);//-3.5;//+10;

   // angel=KalmanFilter(Get_Attitude(),Get_Gyro()/65.5)+7.1-2.8;


    //float KalmanFilter(float Angle1,float icm20602_gyro_x);   //   Angle1     icm20602_gyro_x
  // float KalmanFilter(float Angle2,float icm20602_gyro_y);   //   Angle1     icm20602_gyro_x

  fy=KalmanFilter((float)Angle1,(float) icm20602_gyro_y/65.5)-0.388;
  fg=KalmanFilter1((float)Angle2,(float) icm20602_gyro_x/65.5);//+0.8
 // printf("%d,%f,%f\n", 0,fy,Angle1);
 // printf("%d,%f,%f\n", 0,fg,Angle2);

  printf("%d,%f,%f\n", 0,Angle2,fg);
}







float Get_Gyro_Fy(void)
{

static float Angle=0;

icm20602_get_gyro();  // 获取ICM20602的角速度测量数值

//gyro_Y=icm_gyro_y;
Angle=icm20602_gyro_y;

return Angle;
}



float Get_Gyro_Fg(void)//获取x轴角速度
{

static float Angle1=0;

icm20602_get_gyro();  // 获取ICM20602的角速度测量数值

//gyro_Y=icm_gyro_y;
Angle1=icm20602_gyro_x;
//tft180_show_float(100,110,Angle1,2,3);
return Angle1;
}


float Get_Gyro_Z(void)
{

static float AngleZ=0;

icm20602_get_gyro();  // 获取ICM20602的角速度测量数值

//gyro_Y=icm_gyro_y;
AngleZ=icm20602_gyro_z;
//tft180_show_float(100,110,Angle1,2,3);
return AngleZ;
}



float Get_Attitude_Fy(void)//获取y轴加速度
{
    static float Angle=0;
    icm20602_get_acc();   // 获取ICM20602的加速度测量数值
    if(icm20602_acc_y>4096) icm20602_acc_y=4096;
    if(icm20602_acc_y<-4096) icm20602_acc_y=-4096;
   // else if(icm20602_acc_y==0) icm20602_acc_y=0;
    Angle = ((acos((double)icm20602_acc_y * 1.0 / 4096.0) * 180.0 / 3.141593) -0.5);
    //printf("%lf",icm20602_acc_y);
    return Angle;
}


float Get_Attitude_Fg(void)//获取x轴加速度
{
    static float Angle1=0;
    icm20602_get_acc();   // 获取ICM20602的加速度测量数值

    if(icm20602_acc_x>4096) icm20602_acc_x=4096;
     if(icm20602_acc_x<-4096) icm20602_acc_x=-4096;
    //else if(icm20602_acc_x==0) icm20602_acc_x=0;
    Angle1 = ((acos((double)icm20602_acc_x * 1.0 / 4096.0) * 180.0 / 3.141593)-0.5);
    return Angle1;
}





//1. 结构体类型定义
typedef struct
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter

//2. 以高度为例 定义卡尔曼结构体并初始化参数
KFP KFP_height={0.02,0,0,0,0.001,0.543};

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 int16 kalmanFilter(KFP *kfp,int16 input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;

     return kfp->out;
 }

 extern float zero_data=0;


void zero_get_init(void)
 {
     int16 z=0;
     for(int i=0;i<2000;i++)
     {
         icm20602_get_gyro();
         z+=icm20602_gyro_z;

     }
       zero_data=z/2000;
 }


 float yaw_angle1;
void Get_Angle_Z(void)//z轴角度获取函数  一维卡尔曼滤波后数据
{

static float Gyro_Z=0;

static int16 icmz_data=0;

icm20602_get_gyro();  // 获取ICM20602的角速度测量数值
icmz_data=kalmanFilter(&KFP_height,icm20602_gyro_z-zero_data)  ;//卡尔曼滤波后z角速度

Gyro_Z=icm20602_gyro_transition(icmz_data);
yaw_angle1+=Gyro_Z*dt1-0.0083700;//处理z轴偏移，二分法



              /* if(yaw_angle1>180)
    {
                    yaw_angle1-=360;
    }

    else if(yaw_angle1<-180)
    {
        yaw_angle1+=360;
                }*/


}
float kp_left=0.00004;//9
float kp_right=0.00004;//9
float max_angle_left=3.6;
float max_angle_right=3;

float Turn_loop_left(int Bias,int sped)//左压弯
{
    float Angle_zero_aim;

    Bias=Bias*1.0;
    sped=sped*1.0;
    Angle_zero_aim=kp_left*Bias*sped*sped/80*16;


    if(Angle_zero_aim >= max_angle_left)Angle_zero_aim=max_angle_left;  //此处4.0为最大压弯角度 限幅4.0
    else if(Angle_zero_aim < -max_angle_left)Angle_zero_aim=-max_angle_left;

    return Angle_zero_aim;
}


float Turn_loop_right(int Bias,int sped)//右压弯
{
    float Angle_zero_aim;

    Bias=Bias*1.0;
    sped=sped*1.0;
    Angle_zero_aim=kp_right*Bias*sped*sped/80*16;


    if(Angle_zero_aim >= max_angle_right)Angle_zero_aim=max_angle_right;  //此处4.0为最大压弯角度 限幅4.0
    else if(Angle_zero_aim < -max_angle_right)Angle_zero_aim=-max_angle_right;

    return Angle_zero_aim;
}


float bend_k1=28.10;
float bend_k2=4.3;//k2>0
float max_angle=3;
float bend(float Bias,float speed)//压弯
{

    float Angle_zero_aim=0;//目标零点

    //
    if(bend_k1*bend_k2*Bias!=0)//防止除零
    {
     Angle_zero_aim=sin(Bias*bend_k2/180/10)*9.8+speed*speed/(bend_k1*bend_k2*Bias);
    }


    if(Angle_zero_aim >= max_angle)Angle_zero_aim=max_angle;  //此处4.0为最大压弯角度 限幅4.0
    else if(Angle_zero_aim < -max_angle)Angle_zero_aim=-max_angle;
    return Angle_zero_aim;

}
