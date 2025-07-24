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
    ServoPID.angle_pitch=0;//4��4��0.17���
    ServoPID.highleft=walk_leg_high;
    ServoPID.highright=walk_leg_high;//walk_leg_high
}
void pid2_flexible_init(void)//high=walk_leg_high
{
    pid2_flexible.run_speed=flexible_speed;//�ٶȳ�ʼ��
    pid2_flexible.mechanical_neutral_point=-6.5;//��е��ֵ��ʼ��
    pid1_walk.turn_value=0;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.552
     * float i                  ���������                       0.06
     * float d                  ΢�������                       0.03
     * float maxI               �������޷�                       1500
     * float maxOut             ����޷�                         3000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     //PID_Clear(&pid1_walk.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
     PID_Init(&pid1_walk.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     //PID_Clear(&pid1_walk.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -113
     * float i                  ���������                       0.84
     * float d                  ΢�������                       -336
     * float maxI               �������޷�                       800
     * float maxOut             ����޷�                         2000
     * */
    //  PID_Init(&pid1_walk.jiaodu_left,-113,0.84,-336,800,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
    //  PID_Clear(&pid1_walk.jiaodu_left);//��սǶȻ�pid����ʷ����
    // /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
    //  * float p                  ���������                       0.14
    //  * float i                  ���������                       0.0007
    //  * float d                  ΢�������                       0
    //  * float maxI               �������޷�                       20
    //  * float maxOut             ����޷�                         30
    //  * */
    //  PID_Init(&pid1_walk.sudu_left,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
    //  PID_Clear(&pid1_walk.sudu_left);//����ٶȻ�pid����ʷ����
    //  PID_Init(&pid1_walk.sudu_right,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
    //  PID_Clear(&pid1_walk.sudu_right);//����ٶȻ�pid����ʷ����
    PID_Init(&pid1_walk.jiaodu_left,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Init(&pid1_walk.jiaodu_right,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Init(&pid1_walk.sudu_left,0.06,0.0005,0,30,30);//�ٶȻ�������ʼ��
    PID_Init(&pid1_walk.sudu_right,0.06,0.0005,0,30,30);//�ٶȻ�������ʼ��0.06,
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.4,0.004,0.1,100,2000);//ת�򻷲�����ʼ��
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//���ٶȵ�ͨ�˲�����Ȼ̫��
     //PID_Clear(&pid1_walk.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void mode_switch(int mode_choice){
    if(mode_choice==walk){
        ServoPID.highleft=walk_leg_high;
        ServoPID.highright=walk_leg_high;//walk_leg_high
        ServoPID.angle_roll=0;
        pid1_walk.run_speed=0;//�ٶȳ�ʼ��
        PID_Init(&pid1_walk.jiaodu_left,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.jiaodu_right,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.sudu_left,0.05,0.003,0,15,18);//�ٶȻ�������ʼ��
        PID_Init(&pid1_walk.sudu_right,0.05,0.003,0,15,18);//�ٶȻ�������ʼ��
        mode=walk;
    }
    else if(mode_choice==flexible){
        ServoPID.angle_roll=0;
        PID_Init(&pid1_walk.jiaodu_left,-70,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.jiaodu_right,-70,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.sudu_left,0.18,0.0005,0,30,30);//�ٶȻ�������ʼ��
        PID_Init(&pid1_walk.sudu_right,0.18,0.0005,0,30,30);//�ٶȻ�������ʼ��0.06,
        mode=flexible;
    }
    else if(mode_choice==jump){
        ServoPID.angle_roll=0;
        PID_Init(&pid1_walk.jiaodu_left,-70,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.jiaodu_right,-70,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
        PID_Init(&pid1_walk.sudu_left,0.18,0.0005,0,30,30);//�ٶȻ�������ʼ��
        PID_Init(&pid1_walk.sudu_right,0.18,0.0005,0,30,30);//�ٶȻ�������ʼ��0.06,
        mode=jump;
    }
}
void pid1_walk_init(void)//pid����
{
    pid1_walk.run_speed=walk_speed;//�ٶȳ�ʼ��
    pid1_walk.mechanical_neutral_point=-7;//��е��ֵ��ʼ��-9
    pid1_walk.turn_value=0;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.552
     * float i                  ���������                       0.06
     * float d                  ΢�������                       0.03
     * float maxI               �������޷�                       1500
     * float maxOut             ����޷�                         3000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     //PID_Clear(&pid1_walk.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
     PID_Init(&pid1_walk.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     //PID_Clear(&pid1_walk.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -113
     * float i                  ���������                       0.84
     * float d                  ΢�������                       -336
     * float maxI               �������޷�                       800
     * float maxOut             ����޷�                         2000
     * */
    //  PID_Init(&pid1_walk.jiaodu_left,-113,0.84,-336,800,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
    //  PID_Clear(&pid1_walk.jiaodu_left);//��սǶȻ�pid����ʷ����
    // /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
    //  * float p                  ���������                       0.14
    //  * float i                  ���������                       0.0007
    //  * float d                  ΢�������                       0
    //  * float maxI               �������޷�                       20
    //  * float maxOut             ����޷�                         30
    //  * */
    //  PID_Init(&pid1_walk.sudu_left,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
    //  PID_Clear(&pid1_walk.sudu_left);//����ٶȻ�pid����ʷ����
    //  PID_Init(&pid1_walk.sudu_right,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
    //  PID_Clear(&pid1_walk.sudu_right);//����ٶȻ�pid����ʷ����
    PID_Init(&pid1_walk.jiaodu_left,-50,-0.4,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Init(&pid1_walk.jiaodu_right,-50,-0.4,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Init(&pid1_walk.sudu_left,0.07,0.003,0,15,20);//�ٶȻ�������ʼ��
    PID_Init(&pid1_walk.sudu_right,0.07,0.003,0,15,20);//�ٶȻ�������ʼ��0.06,
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.4,0.004,0.1,100,2000);//ת�򻷲�����ʼ��
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,1);//���ٶȵ�ͨ�˲�����Ȼ̫��
     //PID_Clear(&pid1_walk.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void pid3_jump_init(void)//high=walk_leg_high
{
    pid3_jump.run_speed=0;//�ٶȳ�ʼ��
    pid3_jump.mechanical_neutral_point=-1;//��е��ֵ��ʼ��
    pid3_jump.turn_value=0;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.552
     * float i                  ���������                       0.06
     * float d                  ΢�������                       0.03
     * float maxI               �������޷�                       1500
     * float maxOut             ����޷�                         3000
     * */
     PID_Init(&pid3_jump.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid3_jump.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
     PID_Init(&pid3_jump.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid3_jump.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -113
     * float i                  ���������                       0.84
     * float d                  ΢�������                       -336
     * float maxI               �������޷�                       800
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid3_jump.jiaodu_left,-113,0.84,-336,800,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid3_jump.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
     * float p                  ���������                       0.14
     * float i                  ���������                       0.0007
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       20
     * float maxOut             ����޷�                         30
     * */
     PID_Init(&pid3_jump.sudu_left,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid3_jump.sudu_left);//����ٶȻ�pid����ʷ����
     PID_Init(&pid3_jump.sudu_right,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid3_jump.sudu_right);//����ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
     PID_Init(&pid3_jump.zhuanxiang,0.5,0.004,0.1,500,1000);//ת�򻷲�����ʼ��
     PID_SetErrLpfRatio(&pid3_jump.zhuanxiang,0.7);//���ٶȵ�ͨ�˲�����Ȼ̫��
     PID_Clear(&pid3_jump.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void pid_init_all(void){
    pid2_flexible_init();//����pid����
    pid1_walk_init();//����pid����
    pid3_jump_init();
}
