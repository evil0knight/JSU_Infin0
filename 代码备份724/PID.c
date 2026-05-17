/****************PID运算****************/
#include "zf_common_headfile.h"

CascadePID pid2_flexible;//地形自适应pid
CascadePID pid0_stand_c;//站立串级pid
CascadePID pid1_walk;//行走pid
CascadePID pid3_jump;//跳跃pid
Servo_PID ServoPID;
mode_choice mode;//小车行动模式选择
//初始化pid参数
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
/*单级pid计算，位置式
 * reference                    参考，目标
 * feedback                     反馈，现状
 */
void position_pid(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//若误差在死区内则error直接置0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//低通滤波
	pid->error=pid->error*pid->errLpfRatio+pid->lastError*(1-pid->errLpfRatio);
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
// 增量式PID控制函数
void incremental_pid_left(PID *pid,float reference,float feedback) {
    float out_increment = 0;               // 增量式PID单次增量
    static float ek_l = 0, ek1_l = 0, ek2_l = 0; // 前后三次误差
    // 保存误差值
    ek2_l = ek1_l;                           // 保存上上次误差
    ek1_l = ek_l;                            // 保存上次误差
    ek_l = reference-feedback;               // 计算当前误差
    // 计算PID增量
    out_increment = (pid->kp * (ek_l - ek1_l) + pid->ki * ek_l + pid->kd * (ek_l - 2 * ek1_l + ek2_l));
    // 累加增量输出
    pid->output+= out_increment;
    // 输出限幅，防止超出最大值
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
void incremental_pid_right(PID *pid,float reference,float feedback) {
    float out_increment = 0;                 // 增量式PID单次增量
    static float ek_r = 0, ek1_r = 0, ek2_r = 0; // 前后三次误差
    // 保存误差值
    ek2_r = ek1_r;                           // 保存上上次误差
    ek1_r = ek_r;                            // 保存上次误差
    ek_r = reference-feedback;               // 计算当前误差
    // 计算PID增量
    out_increment = (pid->kp * (ek_r - ek1_r) + pid->ki * ek_r + pid->kd * (ek_r - 2 * ek1_r + ek2_r));
    // 累加增量输出
    pid->output+= out_increment;
    // 输出限幅，防止超出最大值
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
void incremental_pid_servo(PID *pid,float reference,float feedback) {
    float out_increment = 0;                 // 增量式PID单次增量
    static float ek_s = 0, ek1_s = 0, ek2_s = 0; // 前后三次误差
    // 保存误差值
    ek2_s = ek1_s;                           // 保存上上次误差
    ek1_s = ek_s;                            // 保存上次误差
    ek_s = reference-feedback;               // 计算当前误差
    // 计算PID增量
    out_increment = (pid->kp * (ek_s - ek1_s) + pid->ki * ek_s + pid->kd * (ek_s - 2 * ek1_s + ek2_s));
    // 累加增量输出
    pid->output+= out_increment;
    // 输出限幅，防止超出最大值
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}
//串级pid计算
//void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
//{
//	position_pid(&pid->outer,angleRef,angleFdb);//计算外环(角度环)
//	position_pid(&pid->inner,pid->outer.output,speedFdb);//计算内环(速度环)
//	pid->output=pid->inner.output;
//}
//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}

//设置LPF（误差的低通滤波）比例
void PID_SetErrLpfRatio(PID *pid,float ratio)
{
	pid->errLpfRatio=ratio;
}

//俯仰角角度转化
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