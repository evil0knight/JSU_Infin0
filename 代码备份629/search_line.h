#include "zf_common_headfile.h"
/*
 * ȡ��ʹ��search_line
 * */
#ifndef SEARCH_LINE_H
#define SEARCH_LINE_H

#define SEARCH_IMAGE_W  MT9V03X_W                   //���߿��
#define SEARCH_IMAGE_H  MT9V03X_H                   //���߸߶�
#define BLACKPOINT      50                          //�ڵ�ֵ
#define WHITEMAXMUL     13                          //�׵����ֵ���ڲο���ķŴ���  10Ϊ���Ŵ�
#define WHITEMINMUL     7                           //�׵���Сֵ���ڲο���ķŴ���  10Ϊ���Ŵ�

#define REFRENCEROW     5                           //�ο���ͳ������
#define SEARCHRANGE     10                          //���߰뾶
#define STOPROW         0                           //����ֹͣ��
#define CONTRASTOFFSET  3                           //���߶Ա�ƫ��


extern uint8 reference_point;                       //��̬�ο���
extern uint8 reference_col  ;                       //��̬�ο���
extern uint8 white_max_point;                             //��̬�׵����ֵ
extern uint8 white_min_point;                             //��̬�׵���Сֵ
extern uint8 reference_contrast_ratio;                    //�ο��Աȶ�
extern uint8 reference_col_line[SEARCH_IMAGE_H];          //�ο��л���
extern uint8 remote_distance[SEARCH_IMAGE_W];             //�׵�Զ�˾���
extern uint8 left_edge_line[SEARCH_IMAGE_H];              //���ұ߽�
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint32 if_count;

void get_reference_point        (const uint8 *image);       //��ȡ�ο���λ
void search_reference_col       (const uint8 *image);       //����ͼ��ο���
void search_line                (const uint8 *image);       //���������߽�

#endif /*SEARCH_LINE_H*/
