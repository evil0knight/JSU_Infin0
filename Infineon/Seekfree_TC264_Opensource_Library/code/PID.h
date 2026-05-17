#include "zf_common_headfile.h"

#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

//PID参数结构体
typedef struct _PID
{
	float kp,ki,kd;//比例，积分，微分系数
	float error,lastError;//误差
	float integral,maxIntegral;//积分和积分限幅
	float output,maxOutput;//输出和输出限幅
	float deadzone;//死区
	float errLpfRatio;//低通滤波系数
}PID;

//串级PID参数结构体
typedef struct _CascadePID
{
    PID zhuanxiang;
	PID sudu_left,sudu_right;
	PID jiaodu_left,jiaodu_right;
	PID jiaosudu_left,jiaosudu_right;
	float run_speed;//速度
    float mechanical_neutral_point;//机械零点
    float turn_value;//转向大小
}CascadePID;

typedef struct _ServoPID
{
    int16 pwm_ph1, pwm_ph4,pwm_ph2,pwm_ph3;
    float highright,highleft;//目标腿高
    float highrightre,highleftre;//
    float angle_roll;//前后倾角
    float angle_pitch;//滚角
    PID Servo;
}Servo_PID;
extern Servo_PID ServoPID;
extern CascadePID pid2_flexible;//站立并级pid结构体
extern CascadePID pid0_stand_c;//站立并级pid结构体
extern CascadePID pid1_walk;//行走pid
extern CascadePID pid3_jump;//跳跃pid
typedef enum        // 模式选择
{
    flexible,         // 静止站立,并级
    stand_c,          // 静止站立串级
    walk,             // 行走
    stopworking,      // 停止运动
    jump,             //跳跃
}mode_choice;
extern mode_choice mode;

void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut);//初始化pid参数
void PID_Init_sudu(CascadePID *pid,float p,float i,float d,float maxI,float maxOut);//内环参数初始化
void PID_Init_jiaodu(CascadePID *pid,float p,float i,float d,float maxI,float maxOut);//中环参数初始化
void position_pid(PID *pid,float reference,float feedback);//单级pid计算，位置式
void incremental_pid_left(PID *pid,float reference,float feedback);//增量式pid，角速度环左
void incremental_pid_right(PID *pid,float reference,float feedback);//增量式pid，角速度环右
void incremental_pid_servo(PID *pid,float reference,float feedback);//增量式pid，舵机
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);//串级pid计算
void PID_Clear(PID *pid);//清空一个pid的历史数据
void PID_Clear_sudu(CascadePID *pid);//清空内环pid的历史数据
void PID_Clear_jiaodu(CascadePID *pid);//清空中环pid的历史数据
void PID_SetMaxOutput(PID *pid,float maxOut);//重新设定pid输出限幅
void PID_SetDeadzone(PID *pid,float deadzone);//设置PID死区
void PID_SetErrLpfRatio(PID *pid,float ratio);//设置LPF（误差的低通滤波）比例
float Angle_Converter(void);////俯仰角角度转化


#endif
