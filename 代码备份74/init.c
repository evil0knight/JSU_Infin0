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
    ServoPID.highleft=3.3;
    ServoPID.highright=3.3;//3.3
}
void pid2_flexible_init(void)//high=3.3
{
    pid2_flexible.run_speed=160;//�ٶȳ�ʼ��
    pid2_flexible.mechanical_neutral_point=-5.3;//��е��ֵ��ʼ��
    pid2_flexible.turn_value=0;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.552
     * float i                  ���������                       0.06
     * float d                  ΢�������                       0.03
     * float maxI               �������޷�                       1500
     * float maxOut             ����޷�                         3000
     * */
    PID_Init(&pid2_flexible.jiaosudu_left,0.12,0.04,0.09,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
    PID_Clear(&pid2_flexible.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
    PID_Init(&pid2_flexible.jiaosudu_right,0.12,0.04,0.09,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
    PID_Clear(&pid2_flexible.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -113
     * float i                  ���������                       0.84
     * float d                  ΢�������                       -336
     * float maxI               �������޷�                       800
     * float maxOut             ����޷�                         2000
     * */
    PID_Init(&pid2_flexible.jiaodu_left,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Clear(&pid2_flexible.jiaodu_left);//��սǶȻ�pid����ʷ����
    PID_Init(&pid2_flexible.jiaodu_right,-50,0,-350,2000,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
    PID_Clear(&pid2_flexible.jiaodu_right);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
     * float p                  ���������                       0.14
     * float i                  ���������                       0.0007
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       20
     * float maxOut             ����޷�                         30
     * */
    PID_Init(&pid2_flexible.sudu_left,0.06,0,0,20,30);//�ٶȻ�������ʼ��
    PID_Clear(&pid2_flexible.sudu_left);//����ٶȻ�pid����ʷ����
    PID_Init(&pid2_flexible.sudu_right,0.06,0,0,20,30);//�ٶȻ�������ʼ��
    PID_Clear(&pid2_flexible.sudu_right);//����ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
    //PID_Init(&pid2_flexible.zhuanxiang,6,0.005,0.2,800,1500);//ת�򻷲�����ʼ��
    PID_Init(&pid2_flexible.zhuanxiang,6,0.6,0.2,1500,1800);//ת�򻷲�����ʼ��
    pid2_flexible.zhuanxiang.deadzone=60;
    PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//���ٶȵ�ͨ�˲�����Ȼ̫��
    PID_Clear(&pid2_flexible.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void pid0_stand_c_init(void)//����pid����
{
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pd�ĽǶȻ�
     * float p                  ���������                       0.55
     * float i                  ���������                       0.04
     * float d                  ΢�������                       0.02
     * float maxI               �������޷�                       1000
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid0_stand_c.jiaosudu_left,0.552,0.06,0.03,1500,3000);//�ǶȻ�������ʼ��,��е��ֵ-3
     //PID_SetDeadzone(&pid0_stand_c.jiaosudu_left,150);
     PID_Clear(&pid0_stand_c.jiaosudu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pd�ĽǶȻ�
     * float p                  ���������                       -800*0.6=-480
     * float i                  ���������                       0
     * float d                  ΢�������                       -1918*0.6=-1150
     * float maxI               �������޷�                       0
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid0_stand_c.jiaodu_left,-140,0.84,-336,800,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid0_stand_c.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pi���ٶȻ�
     * float p                  ���������                       0.2
     * float i                  ���������                       0.004
     * float d                  ΢�������                       0.08
     * float maxI               �������޷�                       8
     * float maxOut             ����޷�                         20
     * */
     PID_Init(&pid0_stand_c.sudu_left,0.08,0.0008,0,15,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid0_stand_c.sudu_left);//����ٶȻ�pid����ʷ����
}
void pid1_walk_init(void)//pid����
{
    pid1_walk.run_speed=200;//�ٶȳ�ʼ��
    pid1_walk.mechanical_neutral_point=-1;//��е��ֵ��ʼ��
    pid1_walk.turn_value=0;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.552
     * float i                  ���������                       0.06
     * float d                  ΢�������                       0.03
     * float maxI               �������޷�                       1500
     * float maxOut             ����޷�                         3000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
     PID_Init(&pid1_walk.jiaosudu_right,0.552,0.06,0.03,1500,3000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -113
     * float i                  ���������                       0.84
     * float d                  ΢�������                       -336
     * float maxI               �������޷�                       800
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid1_walk.jiaodu_left,-113,0.84,-336,800,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
     * float p                  ���������                       0.14
     * float i                  ���������                       0.0007
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       20
     * float maxOut             ����޷�                         30
     * */
     PID_Init(&pid1_walk.sudu_left,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid1_walk.sudu_left);//����ٶȻ�pid����ʷ����
     PID_Init(&pid1_walk.sudu_right,0.08,0.008,0.01,15,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid1_walk.sudu_right);//����ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.4,0.004,0.1,500,1000);//ת�򻷲�����ʼ��
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//���ٶȵ�ͨ�˲�����Ȼ̫��
     PID_Clear(&pid1_walk.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void pid3_jump_init(void)//high=3.3
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
    pid0_stand_c_init();//����pid����
    pid1_walk_init();//����pid����
    pid3_jump_init();
}
