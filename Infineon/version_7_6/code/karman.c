/*
 * karman.c
 *
 *  Created on: 2022年11月23日
 *      Author: 11150
 */




#include "karman.h"
#include "zf_common_typedef.h"


float gyro;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


#define dt 0.005
#define R_angle    0.5//0.1
float C_0=1.0;
#define Q_angle    0.2//0.01   0.32   28
#define Q_gyro      0.1//0.1  18



float KalmanFilter(float Accel,float Gyro)
{
    static float Angle = 0, Gyro_y = 0;

    Angle+=(Gyro - Q_bias) * dt;

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0];

    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3]= Q_gyro;

    PP[0][0] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - Angle;

    PCt_0 = C_0*PP[0][0];
    PCt_1 = C_0*PP[1][0];

    E = R_angle + C_0*PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0*PP[0][1];

    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle   += K_0 * Angle_err;
    Q_bias  += K_1 * Angle_err;
    Gyro_y   = Gyro - Q_bias;
    return Angle;
}

float gyro1;
float Q_bias1, Angle_err1;
float PCt_01, PCt_11, E1;
float K_01, K_11, t_01, t_11;
float Pdot1[4] ={0,0,0,0};
float PP1[2][2] = { { 1, 0 },{ 0, 1 } };
#define dt1 0.005
#define R_angle1    0.5
float C_01=1.0;
#define Q_angle1    0.2
#define Q_gyro1     0.1
float KalmanFilter1(float Accel1,float Gyro1)
{
    static float Angle1 = 0, Gyro_y1 = 0;

    Angle1+=(Gyro1 - Q_bias1) * dt1;

    Pdot1[0]=Q_angle1 - PP1[0][1] - PP1[1][0];

    Pdot1[1] = -PP1[1][1];
    Pdot1[2] = -PP1[1][1];
    Pdot1[3]= Q_gyro1;

    PP1[0][0] += Pdot1[0] * dt1;
    PP1[0][1] += Pdot1[1] * dt1;
    PP1[1][0] += Pdot1[2] * dt1;
    PP1[1][1] += Pdot1[3] * dt1;

    Angle_err1 = Accel1 - Angle1;

    PCt_01 = C_01*PP1[0][0];
    PCt_11 = C_01*PP1[1][0];

    E1 = R_angle1 + C_01*PCt_01;

    K_01 = PCt_01 / E1;
    K_11 = PCt_11 / E1;

    t_01 = PCt_01;
    t_11 = C_01*PP1[0][1];

    PP1[0][0] -= K_01 * t_01;
    PP1[0][1] -= K_01 * t_11;
    PP1[1][0] -= K_11 * t_01;
    PP1[1][1] -= K_11 * t_11;

    Angle1   += K_01 * Angle_err1;
    Q_bias1  += K_11 * Angle_err1;
    Gyro_y1   = Gyro1 - Q_bias1;
    return Angle1;
}


//float angle;                //数据融合后的角度

float acc_ratio =1.2;       //加速度计比例  3.9    3.0    0.9     2.5     3.6  2.0  1.8/3.0
float gyro_ratio =1.00;      //陀螺仪比例    1.42   1.35   1.42     1.3     2.5  3.8  3.9
float dt2 =0.002;           //采样周期

//----------------------------------------------------------------
//  @brief      一阶互补滤波
//  @param      angle_m     加速度计数据
//  @param      gyro_m      陀螺仪数据
//  @return     float       数据融合后的角度
//----------------------------------------------------------------

float angle_calc(float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;

    static float last_angle;
    static uint8 first_angle;

    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio;

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle)*acc_ratio;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now)*dt2;

    //保存当前角度值
    last_angle = temp_angle;

    return temp_angle;
}

/*
float angle_calc1(float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;

    static float last_angle;
    static uint8 first_angle;

    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio;

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle)*acc_ratio;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now)*dt2;

    //保存当前角度值
    last_angle = temp_angle;

    return temp_angle;
}
*/



