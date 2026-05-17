/*
 * handle_img.h
 *
 *  Created on: 2024Äê10ÔÂ7ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_HANDLE_IMG_H_
#define USER_CAMERA_HANDLE_IMG_H_
#include "zf_common_headfile.h"
void GetImgBin_Average(uint8 img_gray[], float *threshold);
void SearchLineAdaptive_Left(uint8 img_gray[], int32 block_size, int32 down_value, int32 h, int32 w, int32 pts[][2], int32* line_num);
void SearchLineAdaptive_Right(uint8 img_gray[], int32 block_size, int32 down_value, int32 h, int32 w, int32 pts[][2], int32* line_num);
void GetLinesFilter(float pts_in[][2], int32 pts_in_count, float pts_out[][2], int32 kernelsize);
void GetLinesResample(float pts_in[][2], int32 num1, float pts_out[][2], int32* num2, float dist);
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
void GetMidLine_Left(float pts_left[][2], int32 pts_left_count, float mid_left[][2], int32 approx_num, float dist);
void GetMidLine_Right(float pts_right[][2], int32 pts_right_count, float mid_right[][2], int32 approx_num, float dist);
int is_curve(float angle[], int n, float threshold) ;



#endif /* USER_CAMERA_HANDLE_IMG_H_ */
