#include "zf_common_headfile.h"

void flexible_control_table(float high_left,float high_right)
{
    if(high_left==walk_leg_high){
        PID_Init(&pid2_flexible.jiaosudu_left,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaosudu_left);//清空角速度环pid的历史数据

        PID_Init(&pid2_flexible.jiaodu_left,-60,0,-350,2000,3000);//角度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaodu_left);//清空角度环pid的历史数据

        PID_Init(&pid2_flexible.sudu_left,0.14,0.002,0,15,20);//速度环参数初始化
        //PID_Clear(&pid2_flexible.sudu_left);//清空速度环pid的历史数据
    }
    else if(high_left==10.3){
        PID_Init(&pid2_flexible.jiaosudu_left,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaosudu_left);//清空角速度环pid的历史

        PID_Init(&pid2_flexible.jiaodu_left,-60,0,-350,2000,3000);//角度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaodu_left);//清空角度环pid的历史

        PID_Init(&pid2_flexible.sudu_left,0.14,0.002,0,15,20);//速度环参数初始化
        //PID_Clear(&pid2_flexible.sudu_left);//清空速度环pid的历史数据
    }
    if(high_right==walk_leg_high){
        PID_Init(&pid2_flexible.jiaosudu_right,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaosudu_right);//清空角速度环pid的历史数据

        PID_Init(&pid2_flexible.jiaodu_right,-60,0,-350,2000,3000);//角度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaodu_right);//清空角度环pid的历史

        PID_Init(&pid2_flexible.sudu_right,0.14,0.002,0,15,20);//速度环参数初始化
        //PID_Clear(&pid2_flexible.sudu_right);//清空速度环pid的历史数据
        }
    else if(high_right==10.3){
        PID_Init(&pid2_flexible.jiaosudu_right,0.552,0.06,0.03,1500,3000);//角速度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaosudu_right);//清空角速度环pid的历史数据

        PID_Init(&pid2_flexible.jiaodu_right,-60,0,-350,2000,3000);//角度环参数初始化,机械中值-3
        //PID_Clear(&pid2_flexible.jiaodu_right);//清空角度环pid的历史

        PID_Init(&pid2_flexible.sudu_right,0.14,0.002,0,15,20);//速度环参数初始化
        //PID_Clear(&pid2_flexible.sudu_right);//清空速度环pid的历史数据
    }
}

