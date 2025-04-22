#include "zf_common_headfile.h"
#ifndef INIT_H_
#define INIT_H_

/*����isr.c��*/

#define SERVO_1                 (ATOM0_CH0_P21_2)       //�������
#define SERVO_2                 (ATOM0_CH1_P21_3)       //�������
#define SERVO_3                 (ATOM0_CH2_P21_4)       //�������
#define SERVO_4                 (ATOM0_CH3_P21_5)       //�������
#define SERVO_FREQ              (300)
#define SERVO1_MID              (4523)                    //�����ֵ
#define SERVO2_MID              (4563)                    //�����ֵ
#define SERVO3_MID              (4350)                    //�����ֵ
#define SERVO4_MID              (4000)                    //�����ֵ
void pid2_flexible_init(void);//����pidվ������
void pid0_stand_c_init(void);//����pidվ������
void pid1_walk_init(void);//����pid����
void pid3_jump_init(void);
void pid_init_all(void);
void servo_init(void);//�����ʼ��

#endif
