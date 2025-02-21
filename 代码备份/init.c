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
    //����
    servo_control_table(ServoPID.high,ServoPID.angle, &ServoPID.pwm_ph1, &ServoPID.pwm_ph4);
    //�޷�������
    if(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1)
    {
        zf_assert(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1);
        pwm_set_duty(SERVO_1, SERVO1_MID);
        pwm_set_duty(SERVO_2, SERVO2_MID);
        pwm_set_duty(SERVO_3, SERVO3_MID);
        pwm_set_duty(SERVO_4, SERVO4_MID);
    }
    //������
    pwm_set_duty(SERVO_1, SERVO1_MID - ServoPID.pwm_ph4);
    pwm_set_duty(SERVO_2, SERVO2_MID + ServoPID.pwm_ph4);
    pwm_set_duty(SERVO_3, SERVO3_MID + ServoPID.pwm_ph1);
    pwm_set_duty(SERVO_4, SERVO4_MID - ServoPID.pwm_ph1);
}
void pid0_stand_b_init(void)//����pid����
{
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pd�ĽǶȻ�
     * float p                  ���������                       180*0.6=108
     * float i                  ���������                       0
     * float d                  ΢�������                       5500*0.6=3300
     * float maxI               �������޷�                       0
     * float maxOut             ����޷�                         3000
     * */
     PID_Init(&pid0_stand_b.jiaodu_left,108,0,3300,0,3000);//�ǶȻ�������ʼ��,��е��ֵ-1.2
     PID_Clear(&pid0_stand_b.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pi���ٶȻ�
     * float p                  ���������                       18
     * float i                  ���������                       0.09
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       1000
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid0_stand_b.sudu,19,0.095,0,1000,2500);//�ٶȻ�������ʼ��
     PID_Clear(&pid0_stand_b.sudu);//����ٶȻ�pid����ʷ����
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
     PID_Init(&pid0_stand_c.jiaosudu_left,0.55,0.04,0.02,1000,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid0_stand_c.jiaosudu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pd�ĽǶȻ�
     * float p                  ���������                       -84
     * float i                  ���������                       0
     * float d                  ΢�������                       -240
     * float maxI               �������޷�                       0
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid0_stand_c.jiaodu_left,-84,0,-240,0,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid0_stand_c.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid0_standվ��pi���ٶȻ�
     * float p                  ���������                       0.2
     * float i                  ���������                       0.004
     * float d                  ΢�������                       0.08
     * float maxI               �������޷�                       8
     * float maxOut             ����޷�                         20
     * */
     PID_Init(&pid0_stand_c.sudu,0.2,0.004,0.08,8,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid0_stand_c.sudu);//����ٶȻ�pid����ʷ����
}
void pid1_walk_init(void)//pid����
{
    pid1_walk.run_speed=100;//�ٶȳ�ʼ��
    pid1_walk.mechanical_neutral_point=-3;//��е��ֵ��ʼ��
    pid1_walk.turn_value=800;//ת�����ȳ�ʼ��
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�Ľ��ٶȻ�
     * float p                  ���������                       0.45
     * float i                  ���������                       0.04
     * float d                  ΢�������                       0.02
     * float maxI               �������޷�                       1000
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid1_walk.jiaosudu_left,0.45,0.04,0.02,1000,2000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaosudu_left);//��ս��ٶȻ�pid����ʷ����
     PID_Init(&pid1_walk.jiaosudu_right,0.45,0.04,0.02,1000,2000);//���ٶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaosudu_right);//��ս��ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd�ĽǶȻ�
     * float p                  ���������                       -84
     * float i                  ���������                       0
     * float d                  ΢�������                       -240
     * float maxI               �������޷�                       0
     * float maxOut             ����޷�                         2000
     * */
     PID_Init(&pid1_walk.jiaodu_left,-84,0,-500,0,2000);//�ǶȻ�������ʼ��,��е��ֵ-3
     PID_Clear(&pid1_walk.jiaodu_left);//��սǶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pi���ٶȻ�
     * float p                  ���������                       0.2
     * float i                  ���������                       0.004
     * float d                  ΢�������                       0.08
     * float maxI               �������޷�                       8
     * float maxOut             ����޷�                         20
     * */
     PID_Init(&pid1_walk.sudu,0.2,0.004,0.08,8,20);//�ٶȻ�������ʼ��
     PID_Clear(&pid1_walk.sudu);//����ٶȻ�pid����ʷ����
    /* CascadePID *pid          pid�ṹ��                      pid1_walkվ��pd��ת��
     * float p                  ���������                       0
     * float i                  ���������                       0
     * float d                  ΢�������                       0
     * float maxI               �������޷�                       200
     * float maxOut             ����޷�                         300
     * */
     PID_Init(&pid1_walk.zhuanxiang,0.5,0.001,1,500,1000);//ת�򻷲�����ʼ��
     PID_SetErrLpfRatio(&pid1_walk.zhuanxiang,0.7);//���ٶȵ�ͨ�˲�����Ȼ̫��
     PID_Clear(&pid1_walk.zhuanxiang);//����ٶȻ�pid����ʷ����
}
void pid_init_all(void){
    pid0_stand_b_init();//����pid����
    pid0_stand_c_init();//����pid����
    pid1_walk_init();//����pid����
}
