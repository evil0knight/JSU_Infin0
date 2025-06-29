#include "zf_common_headfile.h"

#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

//PID�����ṹ��
typedef struct _PID
{
	float kp,ki,kd;//���������֣�΢��ϵ��
	float error,lastError;//���
	float integral,maxIntegral;//���ֺͻ����޷�
	float output,maxOutput;//���������޷�
	float deadzone;//����
	float errLpfRatio;//��ͨ�˲�ϵ��
}PID;

//����PID�����ṹ��
typedef struct _CascadePID
{
    PID zhuanxiang;
	PID sudu_left,sudu_right;
	PID jiaodu_left,jiaodu_right;
	PID jiaosudu_left,jiaosudu_right;
	float run_speed;//�ٶ�
    float mechanical_neutral_point;//��е���
    float turn_value;//ת���С
}CascadePID;

typedef struct _ServoPID
{
    int16 pwm_ph1, pwm_ph4,pwm_ph2,pwm_ph3;
    float highright,highleft;//Ŀ���ȸ�
    float highrightre,highleftre;//
    float angle_roll;//ǰ�����
    float angle_pitch;//����
    PID Servo;
}Servo_PID;
extern Servo_PID ServoPID;
extern CascadePID pid2_flexible;//վ������pid�ṹ��
extern CascadePID pid0_stand_c;//վ������pid�ṹ��
extern CascadePID pid1_walk;//����pid
extern CascadePID pid3_jump;//��Ծpid
typedef enum        // ģʽѡ��
{
    flexible,         // ��ֹվ��,����
    stand_c,          // ��ֹվ������
    walk,             // ����
    stopworking,      // ֹͣ�˶�
    jump,             //��Ծ
}mode_choice;
extern mode_choice mode;

void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut);//��ʼ��pid����
void PID_Init_sudu(CascadePID *pid,float p,float i,float d,float maxI,float maxOut);//�ڻ�������ʼ��
void PID_Init_jiaodu(CascadePID *pid,float p,float i,float d,float maxI,float maxOut);//�л�������ʼ��
void position_pid(PID *pid,float reference,float feedback);//����pid���㣬λ��ʽ
void incremental_pid_left(PID *pid,float reference,float feedback);//����ʽpid�����ٶȻ���
void incremental_pid_right(PID *pid,float reference,float feedback);//����ʽpid�����ٶȻ���
void incremental_pid_servo(PID *pid,float reference,float feedback);//����ʽpid�����
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);//����pid����
void PID_Clear(PID *pid);//���һ��pid����ʷ����
void PID_Clear_sudu(CascadePID *pid);//����ڻ�pid����ʷ����
void PID_Clear_jiaodu(CascadePID *pid);//����л�pid����ʷ����
void PID_SetMaxOutput(PID *pid,float maxOut);//�����趨pid����޷�
void PID_SetDeadzone(PID *pid,float deadzone);//����PID����
void PID_SetErrLpfRatio(PID *pid,float ratio);//����LPF�����ĵ�ͨ�˲�������
float Angle_Converter(void);////�����ǽǶ�ת��


#endif
