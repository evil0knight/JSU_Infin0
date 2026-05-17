/*
 * extern.h
 *
 *  Created on: 2023Äê6ÔÂ7ÈÕ
 *      Author: 11150
 */

#ifndef CODE_EXTERN_H_
#define CODE_EXTERN_H_


#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

void road_off(void);
void find_point(uint8 mode, uint8 start_row, uint8 end_row);
uint8_t judge_loss_line(uint8 mode);
uint8_t judge_straight_line(uint8 mode);
void patch_line(int16 x1, int16 y1,int16 x2,int16 y2);

void get_angle(uint8 mode);
void get_distance(void);

void    draw_line(void);
void  draw_line_two(void);
void draw_line_200(void);
void draw_line_200_two(void);

extern float gz;
extern float gy;
extern float yaw_angle;
extern float pitch_angle;

extern float speed;
extern float distance;

extern uint8 icm20602_yaw_state;
extern uint8 icm20602_pitch_state;
extern uint8 distance_state;

void pit_handle(void);


#endif /* CODE_EXTERN_H_ */
