#include <Transform_Table.h>
#include <math.h>
#include "matrix.h"
#include <Kalman_fusion_of_imu963ra.h>
#include <Base.h>

//Focal Length:          fc = [ 65.24390   65.20765 ] +/- [ 0.99112   0.98226 ]
//Principal point:       cc = [ 103.60943   54.73447 ] +/- [ 0.43560   0.48324 ]
//Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
//Distortion:            kc = [ 0.00808   -0.00754   -0.00106   0.00239  0.00000 ] +/- [ 0.00687   0.00467   0.00148   0.00209  0.00000 ]
//Pixel error:          err = [ 0.23679   0.22753 ]

float K[3][3] =   {{65.24390,0,103.60943},
                    {0,65.20765,54.73447},
                    {0,         0  ,  1.0000}};
float kc[5] = {0.00808, -0.00754, -0.00106, 0.00239, 0.00000};

float inv[2] = {0};

extern imu963ra_struct imu;
extern wheel_coordinate_struct wheel_L;
extern wheel_coordinate_struct wheel_R;

float pitch = 0.0f;
float c = 0.0f;
float h = 0.0f;
float delt_Y = 0.0f;


//自适应逆透视相机外参更新
void external_reference_update(void)
{
    //角度转弧度
    pitch = (imu.pitch) / 180.0f * My_PI;
    float roll = imu.roll / 180.0f * My_PI;
    float c_roll = 48.0f / 180.0f * My_PI + roll;   //镜头光轴与水平方向夹角+俯仰角度（向下为正）
    c = fabsf(asinf(cosf(My_PI / 2.0f - c_roll) * cosf(pitch)));  //考虑滚转角的实际镜头光轴与水平方向夹角

    if(input.flag != 5)
    {
        h = ((34.0f - (128.0f - (wheel_L.x + wheel_R.x)/2.0f) * sinf(roll) + (231.0f + wheel_L.y) * cosf(roll)) * cosf(pitch)) / 1000.0f;
    }
    else
    {
        if(wheel_L.y >= wheel_R.y)  //镜头实际高度
        {
            h = ((34.0f - (128.0f - (wheel_L.x + wheel_R.x)/2.0f) * sinf(roll) + (231.0f + wheel_L.y) * cosf(roll)) * cosf(pitch) + WHEEL_RADIUS / 2.0f * sinf(-pitch)) / 1000.0f;
        }
        else
        {
            h = ((34.0f - (128.0f - (wheel_L.x + wheel_R.x)/2.0f) * sinf(roll) + (231.0f + wheel_R.y) * cosf(roll)) * cosf(pitch) + WHEEL_RADIUS / 2.0f * sinf(-pitch)) / 1000.0f;
        }
    }

    delt_Y = h * tanf(pitch);    //镜头与底盘中心的水平偏差
}

//自适应逆透视
void adaptive_reverse_perspective(int u, int v)
{
    float x = ((float)u - K[0][2]) / K[0][0];   //转换为Principal point坐标系的坐标
    float y = ((float)v - K[1][2]) / K[1][1];
    //去畸变
    float r2 = x*x+y*y;
    float x_raw = x * (1 + kc[0] * r2 + kc[1]*r2*r2 + kc[4]*r2*r2*r2) + 2 * kc[2] * x * y + kc[3] * (r2 + 2 * x * x);
    float y_raw = y * (1 + kc[0] * r2 + kc[1]*r2*r2 + kc[4]*r2*r2*r2) + kc[2] * (r2 + 2 * y * y) + 2 * kc[3] * x * y;
    //处理滚转角度，转换为X轴与地平线平行的坐标系坐标
    float x_corr = x_raw * cosf(pitch) + y_raw * sinf(pitch);
    float y_corr = y_raw * cosf(pitch) - x_raw * sinf(pitch);
    //转换为像素坐标
    float u_corr = K[0][0] * x_corr + K[0][2];
    float v_corr = K[1][1] * y_corr + K[1][2];
    //计算世界坐标系坐标（相机投影点为原点）
    float gama = atanf((v_corr -K[1][2]) / K[1][1]);
    float l = h / sinf(c + gama);
    inv[0] = h / tanf(c + gama);
    inv[1] = (u_corr - K[0][2]) * l / K[0][0] + delt_Y;
}
