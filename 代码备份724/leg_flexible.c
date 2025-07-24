#include "zf_common_headfile.h"

void flexible_control_table(float high_left,float high_right)
{
    if(high_left==walk_leg_high){
        PID_Init(&pid2_flexible.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����

        PID_Init(&pid2_flexible.jiaodu_left,-60,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaodu_left);//��սǶȻ�pid����ʷ����

        PID_Init(&pid2_flexible.sudu_left,0.14,0.002,0,15,20);//�ٶȻ�������ʼ��
        //PID_Clear(&pid2_flexible.sudu_left);//����ٶȻ�pid����ʷ����
    }
    else if(high_left==10.3){
        PID_Init(&pid2_flexible.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaosudu_left);//��ս��ٶȻ�pid����ʷ

        PID_Init(&pid2_flexible.jiaodu_left,-60,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaodu_left);//��սǶȻ�pid����ʷ

        PID_Init(&pid2_flexible.sudu_left,0.14,0.002,0,15,20);//�ٶȻ�������ʼ��
        //PID_Clear(&pid2_flexible.sudu_left);//����ٶȻ�pid����ʷ����
    }
    if(high_right==walk_leg_high){
        PID_Init(&pid2_flexible.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����

        PID_Init(&pid2_flexible.jiaodu_right,-60,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaodu_right);//��սǶȻ�pid����ʷ

        PID_Init(&pid2_flexible.sudu_right,0.14,0.002,0,15,20);//�ٶȻ�������ʼ��
        //PID_Clear(&pid2_flexible.sudu_right);//����ٶȻ�pid����ʷ����
        }
    else if(high_right==10.3){
        PID_Init(&pid2_flexible.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����

        PID_Init(&pid2_flexible.jiaodu_right,-60,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        //PID_Clear(&pid2_flexible.jiaodu_right);//��սǶȻ�pid����ʷ

        PID_Init(&pid2_flexible.sudu_right,0.14,0.002,0,15,20);//�ٶȻ�������ʼ��
        //PID_Clear(&pid2_flexible.sudu_right);//����ٶȻ�pid����ʷ����
    }
}

