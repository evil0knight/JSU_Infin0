/****************PID����****************/
#include "zf_common_headfile.h"

CascadePID pid2_flexible;//��������Ӧpid
CascadePID pid0_stand_c;//վ������pid
CascadePID pid1_walk;//����pid
CascadePID pid3_jump;//��Ծpid
Servo_PID ServoPID;
mode_choice mode;//С���ж�ģʽѡ��
//��ʼ��pid����
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->deadzone=0;
	pid->errLpfRatio=1;
}
/*����pid���㣬λ��ʽ
 * reference                    �ο���Ŀ��
 * feedback                     ��������״
 */
void position_pid(PID *pid,float reference,float feedback)
{
	//��������
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//���������������errorֱ����0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//��ͨ�˲�
	pid->error=pid->error*pid->errLpfRatio+pid->lastError*(1-pid->errLpfRatio);
	//����΢��
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//�������
	pid->output+=pid->error*pid->kp;
	//�������
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//�����޷�
	pid->output+=pid->integral;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
// ����ʽPID���ƺ���
void incremental_pid_left(PID *pid,float reference,float feedback) {
    float out_increment = 0;               // ����ʽPID��������
    static float ek_l = 0, ek1_l = 0, ek2_l = 0; // ǰ���������
    // �������ֵ
    ek2_l = ek1_l;                           // �������ϴ����
    ek1_l = ek_l;                            // �����ϴ����
    ek_l = reference-feedback;               // ���㵱ǰ���
    // ����PID����
    out_increment = (pid->kp * (ek_l - ek1_l) + pid->ki * ek_l + pid->kd * (ek_l - 2 * ek1_l + ek2_l));
    // �ۼ��������
    pid->output+= out_increment;
    // ����޷�����ֹ�������ֵ
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
void incremental_pid_right(PID *pid,float reference,float feedback) {
    float out_increment = 0;                 // ����ʽPID��������
    static float ek_r = 0, ek1_r = 0, ek2_r = 0; // ǰ���������
    // �������ֵ
    ek2_r = ek1_r;                           // �������ϴ����
    ek1_r = ek_r;                            // �����ϴ����
    ek_r = reference-feedback;               // ���㵱ǰ���
    // ����PID����
    out_increment = (pid->kp * (ek_r - ek1_r) + pid->ki * ek_r + pid->kd * (ek_r - 2 * ek1_r + ek2_r));
    // �ۼ��������
    pid->output+= out_increment;
    // ����޷�����ֹ�������ֵ
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
void incremental_pid_servo(PID *pid,float reference,float feedback) {
    float out_increment = 0;                 // ����ʽPID��������
    static float ek_s = 0, ek1_s = 0, ek2_s = 0; // ǰ���������
    // �������ֵ
    ek2_s = ek1_s;                           // �������ϴ����
    ek1_s = ek_s;                            // �����ϴ����
    ek_s = reference-feedback;               // ���㵱ǰ���
    // ����PID����
    out_increment = (pid->kp * (ek_s - ek1_s) + pid->ki * ek_s + pid->kd * (ek_s - 2 * ek1_s + ek2_s));
    // �ۼ��������
    pid->output+= out_increment;
    // ����޷�����ֹ�������ֵ
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
//����pid����
//void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
//{
//	position_pid(&pid->outer,angleRef,angleFdb);//�����⻷(�ǶȻ�)
//	position_pid(&pid->inner,pid->outer.output,speedFdb);//�����ڻ�(�ٶȻ�)
//	pid->output=pid->inner.output;
//}
//���һ��pid����ʷ����
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//�����趨pid����޷�
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//����PID����
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}

//����LPF�����ĵ�ͨ�˲�������
void PID_SetErrLpfRatio(PID *pid,float ratio)
{
	pid->errLpfRatio=ratio;
}

//�����ǽǶ�ת��
float Angle_Converter(void)
{
    float angle;
    if(euler_angle.roll>=0){
        angle=180-euler_angle.roll;
    }
    else {
        angle=-(180+euler_angle.roll);
    }
    return angle;
}