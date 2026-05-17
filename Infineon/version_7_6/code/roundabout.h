/*
 * roundabout.h
 *
 *  Created on: 2023Äê7ÔÂ20ÈÕ
 *      Author: 11150
 */

#ifndef CODE_ROUNDABOUT_H_
#define CODE_ROUNDABOUT_H_
#include "zf_common_typedef.h"
void diuxian(void);
uint8 judge_elements(void);
void racetrack_process(uint8 start_row, uint8 end_row, uint8 step);

void find_roundabout_midpoint(uint8 mode ,uint8 start_row, uint8 end_row, uint8 step);


extern uint8    left_loss_line_flag;
extern uint8    right_loss_line_flag;
extern uint8    left_straight_line_flag;
extern uint8    right_straight_line_flag;

extern uint8 roundabout_state;
extern uint8 roundabout_midpoint_state;

extern uint8    roundabout_L_flag;
extern uint8    roundabout_R_flag;

extern uint8 roundabout_midpoint_up_num;
extern uint8 roundabout_midpoint_down_num;

void protects(void);


#endif /* CODE_ROUNDABOUT_H_ */
