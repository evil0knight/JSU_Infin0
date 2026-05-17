/*
 * guandao.c
 *
 *  Created on: 2023年8月14日
 *      Author: 11150
 */

#include "guandao.h"
#include "zf_common_headfile.h"
#include "gyro.h"
#include "my_math.h"

#define dt2 0.01//积分时间
extern float yaw_angle1;//偏航角
extern float bmq;//速度，就是驱动轮的速度环获取的编码器值，逻辑上确保前进时bmq为正
extern float angle3;//目标
extern float angle4;//目前
extern float angle5;//目标-目前
float piao_angle;//航向角
double x_distance=0;
double y_distance=0;//小车所在点坐标
double x_xiangdui;//巡点时，车与目标点的x距离
double y_xiangdui;//巡点时，车与目标点的y距离
int flagpoint;//打点标志位，为1打点，无需清零，自动清零
float target_distance;//巡点时的距离目标点的距离
float targets[200][2] ; // 示例目标点列表，之前测377，最多208个点，待商榷
extern int mode;//模式
int num_targets = sizeof(targets) / sizeof(targets[0]);
int current_target = 0,define_target = 0; // current_target当前目标点的索引，define_target储存的点索引


void point_get(void)//模式1：遥控器采点
{
//    system_delay_ms(10); // 假设每次循环间隔0.01秒,
    float dist;
    //float diss=0;
    dist = bmq * 1.0 * dt2;//一步长内移动距离
    // diss+=dist*0.01f;
//    printf("%f\n",dist);
    Get_Angle_Z();//获取0.01s中偏转角，调用z轴获取，需确保其他地方无z轴获取，不然出错
    angle4 = -yaw_angle1+90;//当前小车在起点惯性系方向（陀螺仪获得

    x_distance = x_distance + dist * func_cos(angle4 / 180 * 3.14159f);
    y_distance = y_distance + dist * func_sin(angle4 / 180 * 3.14159f);


    if ( flagpoint== 1)//遥控器打点，或者自动打点
    {

            targets[define_target][0] = x_distance;
            targets[define_target][1] = y_distance;
//            define_target++;
            printf("x=%f",targets[define_target][0]);
            printf("y=%f\n",targets[define_target][1]);
            define_target++;


    }
    flagpoint=0;//清零
}
void erase()//擦除函数，必须在打点和巡点之间调用一次，打点和巡点时不可以调用
{
    yaw_angle1=0;//偏航角置零(模式1与模式2之间操作)+
    x_distance=0;
    y_distance=0;
    current_target=0;

}

int judge_distance=55;//距离判断条件，由于编码器不同，这个距离自己确定
void pianjiao_get()//模式2：偏差角获取,当到达最后目标点时自动停车
{


    //yaw_angle1=0;//偏航角置零(模式1与模式2之间操作)
    //x_distance=0;
    //y_distance=0;x,y坐标置零(模式1与模式2之间操作)
    //system_delay_ms(10);
    Get_Angle_Z();//z轴角度获取
    angle4 = -yaw_angle1+90;//当前小车在起点惯性系方向（陀螺仪获得）
    float dist;
        //float diss=0;
        dist = bmq * 1.0 * dt2;//一步长内移动距离
    x_distance = x_distance + dist * func_cos(angle4 / 180 * 3.14159f);
    y_distance = y_distance + dist * func_sin(angle4 / 180 * 3.14159f);

    x_xiangdui=targets[current_target][0]-x_distance;
    y_xiangdui=targets[current_target][1]-y_distance;//计算目标点与所在点的相对距离
    angle3 = piao_angle;//目标方向（由当前坐标与目标坐标计算求得）

    if(x_xiangdui<0&&y_xiangdui>0)//判断目标点相对小车所在点象限
        piao_angle=atan(y_xiangdui/x_xiangdui) / 3.14159f * 180+180;//第二象限
    else if(x_xiangdui<0&&y_xiangdui<0)
        piao_angle=atan(y_xiangdui/x_xiangdui) / 3.14159f * 180-180;//第三象限
    else if(x_xiangdui > 0)
        piao_angle = atan(y_xiangdui / x_xiangdui) / 3.14159f * 180;//第一，四象限
    else if(x_xiangdui == 0&&y_xiangdui > 0)
        piao_angle = 90;
    else if(x_xiangdui == 0&&y_xiangdui < 0)
            piao_angle = -90;
    else if(y_xiangdui == 0&&x_xiangdui > 0)
                piao_angle = 0;
    else if(y_xiangdui == 0&&x_xiangdui < 0)
                    piao_angle = 180;

               //返回航向角
    angle5 = angle4 - angle3;//偏差角
    printf("angle5=%f\n",angle5);
    printf("current_target=%d/n",current_target);
    if(angle5 > 180)
        angle5 = angle5 - 360;
    else if(angle5 <= -180)
        angle5 = 360 + angle5;
    target_distance=func_sqrt((x_distance-targets[current_target][0])*(x_distance-targets[current_target][0])+(y_distance-targets[current_target][1])*(y_distance-targets[current_target][1]));
    if(current_target==(define_target-2)|(current_target-1))
    {
        judge_distance=50;//靠近车库改变判断条件，让回库更精准，编码器不同，需自己判断
    }

    if(target_distance<judge_distance)//跟随巡点判断
    {
        current_target++;
    }
    if(current_target==define_target+1)//停车
    {
        mode=3;//我们在3模式停车并调用擦除函数
    }
    if(current_target==define_target)//停车，写两个停车，这里有小问题，一个停车有时停不了
    {
        mode=3;
    }
}

