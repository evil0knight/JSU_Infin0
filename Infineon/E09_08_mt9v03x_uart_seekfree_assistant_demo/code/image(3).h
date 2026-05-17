#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"
//#include "control.h"

#define image_h 120// 图像高度（像素）
#define image_w 188// 图像宽度（像素）


#define white_pixel 255// 二值图白色像素值
#define black_pixel 0// 二值图黑色像素值

#define bin_jump_num    1// 二值化跳变阈值（像素）
#define border_max  image_w-2 // 边界最大值（防止越界）
#define border_min  1   // 边界最小值（防止越界）

typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // 未检测到单桥状态
    SINGLE_BRIDGE_ACTIVE       // 检测到单桥状态
} SingleBridgeState;

extern uint8 original_image[image_h][image_w];// 原始图像缓冲区
extern uint8 bin_image[image_h][image_w];// 二值化图像缓冲区

// 边界数组（x1=左边界，x2=中心线，x3=右边界）
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

extern float border;// 边界偏移量（用于转向控制）
extern float turn_value;
extern SingleBridgeState BridgeState;// 桥梁状态标记
extern int jump_position;// 跳变点位置标记
extern int stop_position;// 停车点位置标记
extern int buzzer;// 蜂鸣器状态标记
extern int string_not;// 无有效赛道标记
extern int island;
extern int sum_island;
extern int cross_sum;
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

extern void image_process(void); // 图像主处理函数
void get_turn_value(float kp,float kp2,float kd,float gkd);// 计算转角值
#endif /*_IMAGE_H*/
