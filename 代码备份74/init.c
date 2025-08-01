#include "zf_common_headfile.h"


void buzzer_init(void)
{
    gpio_init(P33_10, GPO, GPIO_LOW, GPO_PUSH_PULL);
}
void servo_init(void)
{
    pwm_init(SERVO_1, SERVO_FREQ, SERVO1_MID);
    pwm_init(SERVO_2, SERVO_FREQ, SERVO2_MID);
    pwm_init(SERVO_3, SERVO_FREQ, SERVO3_MID);
    pwm_init(SERVO_4, SERVO_FREQ, SERVO4_MID);
    ServoPID.pwm_ph1=0;
    ServoPID.pwm_ph4=0;
    ServoPID.pwm_ph3=0;
    ServoPID.pwm_ph2=0;
    ServoPID.angle_roll=0;
    ServoPID.angle_pitch=0;//4，4，0.17最大
    ServoPID.highleft=3.3;
    ServoPID.highright=3.3;//3.3
}
void pid2_flexible_init(void)//high=3.3
{
    pid2_flexible.run_speed=160;//速度初始化
    pid2_flexible.mechanical_neutral_point=-5.3;//机械中值初始化
    pid2_flexible.turn_value=0;//转向力度初始化
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角速度环
     * float p                  比例项参数                       0.552
     * float i                  积分项参数                       0.06
     * float d                  微分项参数                       0.03
     * float maxI               积分项限幅                       1500
     * float maxOut             输出限幅                         3000
     * */
    PID_Init(&pid2_flexible.jiaosudu_left,0.12,0.04,0.09,1500,3000);//角速度环参数初始化,机械中值-3
    PID_Clear(&pid2_flexible.jiaosudu_left);//清空角速度环pid的历史数据
    PID_Init(&pid2_flexible.jiaosudu_right,0.12,0.04,0.09,1500,3000);//角速度环参数初始化,机械中值-3
    PID_Clear(&pid2_flexible.jiaosudu_right);//清空角速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角度环
     * float p                  比例项参数                       -113
     * float i                  积分项参数                       0.84
     * float d                  微分项参数                       -336
     * float maxI               积分项限幅                       800
     * float maxOut             输出限幅                         2000
     * */
    PID_Init(&pid2_flexible.jiaodu_left,-50,0,-350,2000,3000);//角度环参数初始化,机械中值-3
    PID_Clear(&pid2_flexible.jiaodu_left);//清空角度环pid的历史数据
    PID_Init(&pid2_flexible.jiaodu_right,-50,0,-350,2000,3000);//角度环参数初始化,机械中值-3
    PID_Clear(&pid2_flexible.jiaodu_right);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pi的速度环
     * float p                  比例项参数                       0.14
     * float i                  积分项参数                       0.0007
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       20
     * float maxOut             输出限幅                         30
     * */
    PID_Init(&pid2_flexible.sudu_left,0.06,0,0,20,30);//速度环参数初始化
    PID_Clear(&pid2_flexible.sudu_left);//清空速度环pid的历史数据
    PID_Init(&pid2_flexible.sudu_right,0.06,0,0,20,30);//速度环参数初始化
    PID_Clear(&pid2_flexible.sudu_right);//清空速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的转向环
     * float p                  比例项参数                       0
     * float i                  积分项参数                       0
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       200
     * float maxOut             输出限幅                         300
     * */
    //PID_Init(&pid2_flexible.zhuanxiang,6,0.005,0.2,800,1500);//转向环参数初始化
    PID_Init(&pid2_flexible.zhuanxiang,6,0.6,0.2,1500,1800);//转向环参数初始化
    pid2_flexible.zhuanxiang.deadzone=60;
    PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//角速度低通滤波，不然太抖
    PID_Clear(&pid2_flexible.zhuanxiang);//清空速度环pid的历史数据
}
void pid0_stand_c_init(void)//串级pid参数
{
    /* CascadePID *pid          pid结构体                      pid0_stand站立pd的角度环
     * float p                  比例项参数                       0.55
     * float i                  积分项参数                       0.04
     * float d                  微分项参数                       0.02
     * float maxI               积分项限幅                       1000
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid0_stand_c.jiaosudu_left,0.552,0.06,0.03,1500,3000);//角度环参数初始化,机械中值-3
     //PID_SetDeadzone(&pid0_stand_c.jiaosudu_left,150);
     PID_Clear(&pid0_stand_c.jiaosudu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid0_stand站立pd的角度环
     * float p                  比例项参数                       -800*0.6=-480
     * float i                  积分项参数                       0
     * float d                  微分项参数                       -1918*0.6=-1150
     * float maxI               积分项限幅                       0
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid0_stand_c.jiaodu_left,-140,0.84,-336,800,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid0_stand_c.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid0_stand站立pi的速度环
     * float p                  比例项参数                       0.2
     * float i                  积分项参数                       0.004
     * float d                  微分项参数                       0.08
     * float maxI               积分项限幅                       8
     * float maxOut             输出限幅                         20
     * */
     PID_Init(&pid0_stand_c.sudu_left,0.08,0.0008,0,15,20);//速度环参数初始化
     PID_Clear(&pid0_stand_c.sudu_left);//清空速度环pid的历史数据
}
void pid1_walk_init(void)//pid行走
{
    pid1_walk.run_speed=200;//速度初始化
    pid1_walk.mechanical_neutral_point=-1;//机械中值初始化
    pid1_walk.turn_value=0;//转向力度初始化
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角速度环
     * float p                  比例项参数                       0.552
     * float i                  积分项参数                       0.06
     * float d                  微分项参数                       0.03
     * float maxI               积分项限幅                       1500
     * float maxOut             输出限幅                         3000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaosudu_left);//清空角速度环pid的历史数据
     PID_Init(&pid1_walk.jiaosudu_right,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaosudu_right);//清空角速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角度环
     * float p                  比例项参数                       -113
     * float i                  积分项参数                       0.84
     * float d                  微分项参数                       -336
     * float maxI               积分项限幅                       800
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid1_walk.jiaodu_left,-113,0.84,-336,800,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pi的速度环
     * float p                  比例项参数                       0.14
     * float i                  积分项参数                       0.0007
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       20
     * float maxOut             输出限幅                         30
     * */
     PID_Init(&pid1_walk.sudu_left,0.08,0.008,0.01,15,20);//速度环参数初始化
     PID_Clear(&pid1_walk.sudu_left);//清空速度环pid的历史数据
     PID_Init(&pid1_walk.sudu_right,0.08,0.008,0.01,15,20);//速度环参数初始化
     PID_Clear(&pid1_walk.sudu_right);//清空速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的转向环
     * float p                  比例项参数                       0
     * float i                  积分项参数                       0
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       200
     * float maxOut             输出限幅                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.4,0.004,0.1,500,1000);//转向环参数初始化
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//角速度低通滤波，不然太抖
     PID_Clear(&pid1_walk.zhuanxiang);//清空速度环pid的历史数据
}
void pid3_jump_init(void)//high=3.3
{
    pid3_jump.run_speed=0;//速度初始化
    pid3_jump.mechanical_neutral_point=-1;//机械中值初始化
    pid3_jump.turn_value=0;//转向力度初始化
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角速度环
     * float p                  比例项参数                       0.552
     * float i                  积分项参数                       0.06
     * float d                  微分项参数                       0.03
     * float maxI               积分项限幅                       1500
     * float maxOut             输出限幅                         3000
     * */
     PID_Init(&pid3_jump.jiaosudu_left,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid3_jump.jiaosudu_left);//清空角速度环pid的历史数据
     PID_Init(&pid3_jump.jiaosudu_right,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid3_jump.jiaosudu_right);//清空角速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角度环
     * float p                  比例项参数                       -113
     * float i                  积分项参数                       0.84
     * float d                  微分项参数                       -336
     * float maxI               积分项限幅                       800
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid3_jump.jiaodu_left,-113,0.84,-336,800,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid3_jump.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pi的速度环
     * float p                  比例项参数                       0.14
     * float i                  积分项参数                       0.0007
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       20
     * float maxOut             输出限幅                         30
     * */
     PID_Init(&pid3_jump.sudu_left,0.08,0.008,0.01,15,20);//速度环参数初始化
     PID_Clear(&pid3_jump.sudu_left);//清空速度环pid的历史数据
     PID_Init(&pid3_jump.sudu_right,0.08,0.008,0.01,15,20);//速度环参数初始化
     PID_Clear(&pid3_jump.sudu_right);//清空速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的转向环
     * float p                  比例项参数                       0
     * float i                  积分项参数                       0
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       200
     * float maxOut             输出限幅                         300
     * */
     PID_Init(&pid3_jump.zhuanxiang,0.5,0.004,0.1,500,1000);//转向环参数初始化
     PID_SetErrLpfRatio(&pid3_jump.zhuanxiang,0.7);//角速度低通滤波，不然太抖
     PID_Clear(&pid3_jump.zhuanxiang);//清空速度环pid的历史数据
}
void pid_init_all(void){
    pid2_flexible_init();//并级pid参数
    pid0_stand_c_init();//串级pid参数
    pid1_walk_init();//行走pid参数
    pid3_jump_init();
}
