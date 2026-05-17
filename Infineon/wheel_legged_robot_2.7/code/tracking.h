/*
 * tracking.h
 *
 *  Created on: 2024Äê10ÔÂ12ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_TRACKING_H_
#define USER_CAMERA_TRACKING_H_

#include "zf_common_headfile.h"

extern float (*mid_track)[2];
extern int32 mid_track_count;
extern float pure_angle;
extern float pure_angle_half;
extern float dx_near;

extern int8 turn_flag ;
//extern float curvature;
extern float turn_threshold;

void aim_distance_select(void);
void tracking_line(void);
void ElementJudge(void);
void ElementRun(void);
void MidLineTrack(void);

void state_clear();
void center_point(float points[][2], float n, float point[2]);
float calculate_distance(float line_point[2], float slope, float point[2]);
void tracking_init(void);
float calculate_curvature_now(float x[], float y[]);
extern float standardized_curvature_now;
extern float standardized_curvature_ave;
extern float lateral_deviation;
extern float preview_distance;
extern float K_v_sq;
extern float K_v;
extern float K_curvature;
extern float K_lateral_deviation;
extern float K_lateral_deviation_straight;
extern float eva_speed;
extern int is_supplement_line;
#endif /* USER_CAMERA_TRACKING_H_ */
