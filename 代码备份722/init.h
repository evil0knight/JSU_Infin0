#include "zf_common_headfile.h"
#ifndef INIT_H_
#define INIT_H_

/*用于isr.c中*/

#define SERVO_1                 (ATOM0_CH0_P21_2)       //舵机引脚
#define SERVO_2                 (ATOM0_CH1_P21_3)       //舵机引脚
#define SERVO_3                 (ATOM0_CH2_P21_4)       //舵机引脚
#define SERVO_4                 (ATOM0_CH3_P21_5)       //舵机引脚
#define SERVO_FREQ              (300)
#define SERVO1_MID              (4523)                    //舵机中值
#define SERVO2_MID              (4563)                    //舵机中值
#define SERVO3_MID              (4350)                    //舵机中值
#define SERVO4_MID              (4000)                    //舵机中值
#define BUZZER_PIN              P33_10                      //蜂鸣器
#define walk_speed 300//300
#define flexible_speed 180//180
void pid2_flexible_init(void);//并级pid站立参数
void pid1_walk_init(void);//行走pid参数
void pid3_jump_init(void);
void pid_init_all(void);
void servo_init(void);//舵机初始化
void buzzer_init(void);
void mode_switch(int mode_choice);

#endif
