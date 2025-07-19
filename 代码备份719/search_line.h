#include "zf_common_headfile.h"
/*
 * 取消使用search_line
 * */
#ifndef SEARCH_LINE_H
#define SEARCH_LINE_H

#define SEARCH_IMAGE_W  MT9V03X_W                   //搜线宽度
#define SEARCH_IMAGE_H  MT9V03X_H                   //搜线高度
#define BLACKPOINT      50                          //黑点值
#define WHITEMAXMUL     13                          //白点最大值基于参考点的放大倍数  10为不放大
#define WHITEMINMUL     7                           //白点最小值基于参考点的放大倍数  10为不放大

#define REFRENCEROW     5                           //参考点统计行数
#define SEARCHRANGE     10                          //搜线半径
#define STOPROW         0                           //搜线停止行
#define CONTRASTOFFSET  3                           //搜线对比偏移


extern uint8 reference_point;                       //动态参考点
extern uint8 reference_col  ;                       //动态参考列
extern uint8 white_max_point;                             //动态白点最大值
extern uint8 white_min_point;                             //动态白点最小值
extern uint8 reference_contrast_ratio;                    //参考对比度
extern uint8 reference_col_line[SEARCH_IMAGE_H];          //参考列绘制
extern uint8 remote_distance[SEARCH_IMAGE_W];             //白点远端距离
extern uint8 left_edge_line[SEARCH_IMAGE_H];              //左右边界
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint32 if_count;

void get_reference_point        (const uint8 *image);       //获取参考点位
void search_reference_col       (const uint8 *image);       //搜索图像参考列
void search_line                (const uint8 *image);       //搜索赛道边界

#endif /*SEARCH_LINE_H*/
