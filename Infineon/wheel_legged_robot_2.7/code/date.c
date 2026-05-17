/*
 * date.c
 *
 *  Created on: 2024Äê10ÔÂ6ÈÕ
 *      Author: A
 */

#include "date.h"

uint8_t (*Img_Gray)[MT9V03X_W];

int32_t pts_left[PT_MAXLEN][2];
int32_t pts_right[PT_MAXLEN][2];
int32_t pts_bridge_left[PT_MAXLEN][2];
int32_t pts_bridge_right[PT_MAXLEN][2];
int32_t pts_left_count;
int32_t pts_right_count;
int32_t pts_bridge_left_count;
int32_t pts_bridge_right_count;
int32_t pts_left_curvature[PT_MAXLEN][2];
int32_t pts_right_curvature[PT_MAXLEN][2];
int32_t pts_left_count_curvature;
int32_t pts_right_count_curvature;
int32_t mid_s_count;
int32_t pts_far_left[PT_MAXLEN][2];
int32_t pts_far_right[PT_MAXLEN][2];
int32_t pts_far_left_count;
int32_t pts_far_right_count;

float pts_inv_l[PT_MAXLEN][2];
float pts_inv_r[PT_MAXLEN][2];
int32_t pts_inv_l_count;
int32_t pts_inv_r_count;
float pts_far_inv_l[PT_MAXLEN][2];
float pts_far_inv_r[PT_MAXLEN][2];
int32_t pts_far_inv_l_count;
int32_t pts_far_inv_r_count;
float pts_bridge_inv_l[PT_MAXLEN][2];
float pts_bridge_inv_r[PT_MAXLEN][2];
int32_t pts_bridge_inv_l_count;
int32_t pts_bridge_inv_r_count;

float pts_filter_l[PT_MAXLEN][2];
float pts_filter_r[PT_MAXLEN][2];
int32_t pts_filter_l_count;
int32_t pts_filter_r_count;
float pts_far_filter_l[PT_MAXLEN][2];
float pts_far_filter_r[PT_MAXLEN][2];
int32_t pts_far_filter_l_count;
int32_t pts_far_filter_r_count;
float pts_bridge_filter_l[PT_MAXLEN][2];
float pts_bridge_filter_r[PT_MAXLEN][2];
int32_t pts_bridge_filter_l_count;
int32_t pts_bridge_filter_r_count;

float pts_resample_left[PT_MAXLEN][2];
float pts_resample_right[PT_MAXLEN][2];
int32_t pts_resample_left_count;
int32_t pts_resample_right_count;
int32_t Lpt0_found_count;
int32_t Lpt1_found_count;
float pts_far_resample_left[PT_MAXLEN][2];
float pts_far_resample_right[PT_MAXLEN][2];
int32_t pts_far_resample_left_count;
int32_t pts_far_resample_right_count;
float pts_bridge_resample_left[PT_MAXLEN][2];
float pts_bridge_resample_right[PT_MAXLEN][2];
int32 pts_bridge_resample_left_count;
int32 pts_bridge_resample_right_count;

float mid_left[PT_MAXLEN][2];
float mid_right[PT_MAXLEN][2];
float mid_left_barrier[PT_MAXLEN][2];
float mid_right_barrier[PT_MAXLEN][2];
float mid_barrier_besides[PT_MAXLEN][2];

float mid_s[PT_MAXLEN][2];

int32_t mid_left_count;
int32_t mid_right_count;
int32_t mid_left_barrier_count;
int32_t mid_right_barrier_count;
int32_t mid_barrier_besides_count;

float angle_new_left[PT_MAXLEN];
float angle_new_right[PT_MAXLEN];
float angle_new_left_barrier[PT_MAXLEN];
float angle_new_right_barrier[PT_MAXLEN];
int angle_new_left_num;
int angle_new_right_num;
int angle_new_left_num_barrier;
int angle_new_right_num_barrier;
float far_angle_new_left[PT_MAXLEN];
float far_angle_new_right[PT_MAXLEN];
int far_angle_new_left_num;
int far_angle_new_right_num;
float bridge_angle_new_left[PT_MAXLEN];
float bridge_angle_new_right[PT_MAXLEN];
int bridge_angle_new_left_num;
int bridge_angle_new_right_num;
float bridge_angle_left[PT_MAXLEN];
float bridge_angle_right[PT_MAXLEN];
int bridge_angle_left_num;
int bridge_angle_right_num;

uint8_t mt9v03x_image_copy[MT9V03X_H][MT9V03X_W];
uint8_t mt9v03x_image_mirror_copy[MT9V03X_H][MT9V03X_W];

float angle_left[PT_MAXLEN];
float angle_right[PT_MAXLEN];
float angle_left_barrier[PT_MAXLEN];
float angle_right_barrier[PT_MAXLEN];
int angle_left_num;
int angle_right_num;
int angle_left_barrier_num;
int angle_right_barrier_num;
float far_angle_left[PT_MAXLEN];
float far_angle_right[PT_MAXLEN];
int far_angle_left_num;
int far_angle_right_num;
int far_x1_LAST;
int far_x2_LAST;

int Lpt0_rpts0s_id;
int Lpt1_rpts1s_id;
int far_Lpt0_rpts0s_id;
int far_Lpt1_rpts1s_id;
int bridge_Lpt0_rpts0s_id;
int bridge_Lpt1_rpts1s_id;
int Lpt0_rpts0s[2];
int Lpt1_rpts1s[2];
int Lpt0_rpts0s_id_barrier;
int Lpt1_rpts1s_id_barrier;
int Lpt0_rpts0s_barrier[2];
int Lpt1_rpts1s_barrier[2];
bool Lpt0_found;
bool Lpt1_found;
bool far_Lpt0_found;
bool far_Lpt1_found;
bool bridge_Lpt0_found;
bool bridge_Lpt1_found;
bool Lpt0_found_barrier;
bool Lpt1_found_barrier;
bool Lpt0_found_barrier_in;
bool Lpt1_found_barrier_in;
int Lpt1[2];
int Lpt0[2];
int far_Lpt1[2];
int far_Lpt0[2];

int Lpt_in0_rpts0s_id;
int Lpt_in1_rpts1s_id;
bool Lpt_in0_found;
bool Lpt_in1_found;
int Lpt_in1[2];
int Lpt_in0[2];
int Lpt0_found_barrier_in_id;
int Lpt1_found_barrier_in_id;


bool is_straight0;
bool is_straight1;
bool is_far_straight0;
bool is_far_straight1;

bool is_turn0_l;
bool is_turn1_l;
bool is_turn0_r;
bool is_turn1_r;

float rptsn[PT_MAXLEN][2];
int32_t rptsn_num;
float aim_distance;
float aim_judge_far=0.3f;
float FIX_BINTHRESHOLD = 110;

track_type_e track_type = TRACK_RIGHT;

int begin_flag = 0;
