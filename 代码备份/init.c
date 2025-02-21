#include "zf_common_headfile.h"

void servo_init(void)
{
    pwm_init(SERVO_1, SERVO_FREQ, SERVO1_MID);
    pwm_init(SERVO_2, SERVO_FREQ, SERVO2_MID);
    pwm_init(SERVO_3, SERVO_FREQ, SERVO3_MID);
    pwm_init(SERVO_4, SERVO_FREQ, SERVO4_MID);
    ServoPID.pwm_ph1=0;
    ServoPID.pwm_ph4=0;
    ServoPID.angle=25;
    ServoPID.high=4;
    //计算
    servo_control_table(ServoPID.high,ServoPID.angle, &ServoPID.pwm_ph1, &ServoPID.pwm_ph4);
    //限幅防卡死
    if(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1)
    {
        zf_assert(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1);
        pwm_set_duty(SERVO_1, SERVO1_MID);
        pwm_set_duty(SERVO_2, SERVO2_MID);
        pwm_set_duty(SERVO_3, SERVO3_MID);
        pwm_set_duty(SERVO_4, SERVO4_MID);
    }
    //更新腿
    pwm_set_duty(SERVO_1, SERVO1_MID - ServoPID.pwm_ph4);
    pwm_set_duty(SERVO_2, SERVO2_MID + ServoPID.pwm_ph4);
    pwm_set_duty(SERVO_3, SERVO3_MID + ServoPID.pwm_ph1);
    pwm_set_duty(SERVO_4, SERVO4_MID - ServoPID.pwm_ph1);
}
void pid0_stand_b_init(void)//并级pid参数
{
    /* CascadePID *pid          pid结构体                      pid0_stand站立pd的角度环
     * float p                  比例项参数                       180*0.6=108
     * float i                  积分项参数                       0
     * float d                  微分项参数                       5500*0.6=3300
     * float maxI               积分项限幅                       0
     * float maxOut             输出限幅                         3000
     * */
     PID_Init(&pid0_stand_b.jiaodu_left,108,0,3300,0,3000);//角度环参数初始化,机械中值-1.2
     PID_Clear(&pid0_stand_b.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid0_stand站立pi的速度环
     * float p                  比例项参数                       18
     * float i                  积分项参数                       0.09
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       1000
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid0_stand_b.sudu,19,0.095,0,1000,2500);//速度环参数初始化
     PID_Clear(&pid0_stand_b.sudu);//清空速度环pid的历史数据
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
     PID_Init(&pid0_stand_c.jiaosudu_left,0.55,0.04,0.02,1000,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid0_stand_c.jiaosudu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid0_stand站立pd的角度环
     * float p                  比例项参数                       -84
     * float i                  积分项参数                       0
     * float d                  微分项参数                       -240
     * float maxI               积分项限幅                       0
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid0_stand_c.jiaodu_left,-84,0,-240,0,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid0_stand_c.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid0_stand站立pi的速度环
     * float p                  比例项参数                       0.2
     * float i                  积分项参数                       0.004
     * float d                  微分项参数                       0.08
     * float maxI               积分项限幅                       8
     * float maxOut             输出限幅                         20
     * */
     PID_Init(&pid0_stand_c.sudu,0.2,0.004,0.08,8,20);//速度环参数初始化
     PID_Clear(&pid0_stand_c.sudu);//清空速度环pid的历史数据
}
void pid1_walk_init(void)//pid行走
{
    pid1_walk.run_speed=100;//速度初始化
    pid1_walk.mechanical_neutral_point=-3;//机械中值初始化
    pid1_walk.turn_value=800;//转向力度初始化
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角速度环
     * float p                  比例项参数                       0.45
     * float i                  积分项参数                       0.04
     * float d                  微分项参数                       0.02
     * float maxI               积分项限幅                       1000
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.45,0.04,0.02,1000,2000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaosudu_left);//清空角速度环pid的历史数据
     PID_Init(&pid1_walk.jiaosudu_right,0.45,0.04,0.02,1000,2000);//角速度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaosudu_right);//清空角速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的角度环
     * float p                  比例项参数                       -84
     * float i                  积分项参数                       0
     * float d                  微分项参数                       -240
     * float maxI               积分项限幅                       0
     * float maxOut             输出限幅                         2000
     * */
     PID_Init(&pid1_walk.jiaodu_left,-84,0,-500,0,2000);//角度环参数初始化,机械中值-3
     PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pi的速度环
     * float p                  比例项参数                       0.2
     * float i                  积分项参数                       0.004
     * float d                  微分项参数                       0.08
     * float maxI               积分项限幅                       8
     * float maxOut             输出限幅                         20
     * */
     PID_Init(&pid1_walk.sudu,0.2,0.004,0.08,8,20);//速度环参数初始化
     PID_Clear(&pid1_walk.sudu);//清空速度环pid的历史数据
    /* CascadePID *pid          pid结构体                      pid1_walk站立pd的转向环
     * float p                  比例项参数                       0
     * float i                  积分项参数                       0
     * float d                  微分项参数                       0
     * float maxI               积分项限幅                       200
     * float maxOut             输出限幅                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.5,0.001,1,500,1000);//转向环参数初始化
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//角速度低通滤波，不然太抖
     PID_Clear(&pid1_walk.zhuanxiang);//清空速度环pid的历史数据
}
void pid_init_all(void){
    pid0_stand_b_init();//并级pid参数
    pid0_stand_c_init();//串级pid参数
    pid1_walk_init();//行走pid参数
}
