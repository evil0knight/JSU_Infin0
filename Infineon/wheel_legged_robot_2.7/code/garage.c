/*
 * garage.c
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */
//车库检测
#include "math.h"
#include "headfile.h"
#define M_PI  3.14159265358979323846   // pi

enum garage_type_e garage_type = GARAGE_NONE;

float (*garage_rpts)[2];
int garage_rpts_num;

float calculate_vector_angle(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;

    float vector_length = Q_sqrt(dx * dx + dy * dy);
    float angle_radians = acos(dx / vector_length);
    float angle_degrees = angle_radians * 180 / M_PI;

    return angle_degrees;
}

void CheckGarage()
{
    //变量标志位
    int banmaxian_hangshu = 0;//斑马线行数

    //从下往上扫描
        for (int y = BEGINH_L + 10; y >= BEGINH_L - 10; y--)
        {
            int banmaxian_kuandu=0;
            //int banmaxian_hangshu=0;
            int banmaxian_geshu=0;
             //从右往左扫描
            for (int x =130; x >=20; x--)
            {
                int baidian_heng=0;
                //扫描到黑色，就进判断
                if (GET_PIX_1C(mt9v03x_image_copy[0], y, x + 2) > FIX_BINTHRESHOLD && GET_PIX_1C(mt9v03x_image_copy[0], y, x + 1) > FIX_BINTHRESHOLD
                && GET_PIX_1C(mt9v03x_image_copy[0], y, x) < FIX_BINTHRESHOLD && GET_PIX_1C(mt9v03x_image_copy[0], y, x - 1) < FIX_BINTHRESHOLD)
                {
                    for (int a = x; a > x - 30; a--)//从黑色点向左侧扫描
                    {
                        //找到白色点
                        if(GET_PIX_1C(mt9v03x_image_copy[0], y, a) > FIX_BINTHRESHOLD && GET_PIX_1C(mt9v03x_image_copy[0], y, a - 1) > FIX_BINTHRESHOLD)
                        {
                            //记录白色点的位置，跳出循环
                            baidian_heng=a;
                            break;
                        }
                    }//斑马线宽度等于黑白点的差
                    banmaxian_kuandu=x-baidian_heng;


                }
                else
                {   //斑马线的宽度在4~8之间认为它成立为斑马线黑色块
                    if (banmaxian_kuandu >= 2 && banmaxian_kuandu <= 6)
                    {
                        //斑马线黑色块++
                        banmaxian_geshu++;
                        //斑马线色块宽度清零，进行下一个黑色块的扫描计算
                        banmaxian_kuandu = 0;
                    }
                    else
                    {
                        //如果不满足对黑色块的认为要求就直接清零，去计算下一个黑色块
                        banmaxian_kuandu = 0;
//                        banmaxian_geshu = 0;
                    }
                }
            }
            //如果色块的个数在6~9之间则认为这一行的斑马线满足要求，在去扫下一行
            if (banmaxian_geshu >= 6 ){banmaxian_hangshu++;}
        }
        //如果有大于等于4行的有效斑马线
        if(banmaxian_hangshu>=2)
        {
            //斑马线标准位置1
            garage_type = GARAGE_FOUND;
        }
        else{garage_type = GARAGE_NONE;}
}

void RunGarage()
{
    if (garage_type == GARAGE_FOUND) {
        state_type = STOP_STATE;
        printf("识别到车库\r\n");
        //garage_type = GARAGE_NONE; // TFIXME 原来是 garage_type == GARAGE_NONE，确认更改后无问题
    }
}
