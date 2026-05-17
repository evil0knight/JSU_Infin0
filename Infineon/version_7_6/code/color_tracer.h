#ifndef color_tracer_H
#define color_tracer_H

#include "zf_common_headfile.h"
// 色彩识别代码参考博客
//  https://blog.csdn.net/niruxi0401/article/details/119685347

#define IMG_X               0               // 图片x坐标
#define IMG_Y               0               // 图片y坐标
#define IMG_W               SCC8660_W - 1   // 图片宽度
#define IMG_H               SCC8660_H - 1   // 图片高度

#define ALLOW_FAIL_PER      10              // 容错率，没1<<ALLOW_FAIL_PER个点允许出现一个错误点，容错率越大越容易识别，但错误率越大
#define ITERATE_NUM         5               // 迭代次数，迭代次数越多识别越精确，但计算量越大

#define CONDI_H_RANGE       20              // 设定色块标定的颜色范围，在set_color_target_condi函数中使用
#define CONDI_S_RANGE       60              // 设定色块标定的对比度范围，在set_color_target_condi函数中使用
#define CONDI_L_RANGE       40              // 设定色块标定的亮度范围，在set_color_target_condi函数中使用

typedef struct{
    unsigned char           h_min;          // 目标最小色调
    unsigned char           h_max;          // 目标最大色调

    unsigned char           s_min;          // 目标最小饱和度
    unsigned char           s_max;          // 目标最大饱和度

    unsigned char           l_min;          // 目标最小亮度
    unsigned char           l_max;          // 目标最大亮度

    unsigned int            width_min;      // 目标最小宽度
    unsigned int            hight_min;      // 目标最小高度

    unsigned int            width_max;      // 目标最大宽度
    unsigned int            hight_max;      // 目标最大高度
}target_condi_struct;                       // 判定为的目标条件

typedef struct{
    unsigned int            x;              // 目标的x坐标
    unsigned int            y;              // 目标的y坐标
    unsigned int            w;              // 目标的宽度
    unsigned int            h;              // 目标的高度
}result_struct;                             // 识别结果

typedef struct{
    unsigned char           red;            // [0,255]
    unsigned char           green;          // [0,255]
    unsigned char           blue;           // [0,255]
}color_rgb_struct;                          // RGB格式颜色

typedef struct{
    unsigned char           hue;            // [0,240]
    unsigned char           saturation;     // [0,240]
    unsigned char           luminance;      // [0,240]
}color_hsl_struct;                          // HSL格式颜色

typedef struct{
    unsigned int            x_start;        // 区域x起始位置
    unsigned int            x_end;          // 区域x结束位置
    unsigned int            y_start;        // 区域y起始位置
    unsigned int            y_end;          // 区域y结束位置
}search_area_struct;                        // 区域

extern target_condi_struct target_color_condi;  // 色块颜色阈值信息
extern target_condi_struct target_color_condi1;
extern result_struct target_pos_out;            // 目标位置信息
extern result_struct target_pos_out1;
// 色块追踪函数
int     color_trace             (const target_condi_struct *condition,result_struct *resu);
// 设置色块目标阈值函数
void    set_color_target_condi  (uint16 rgb565_data, target_condi_struct* condition);
#endif

