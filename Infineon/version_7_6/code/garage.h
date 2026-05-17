/*
 * garage.h
 *
 *  Created on: 2023Äê6ÔÂ18ÈÕ
 *      Author: 11150
 */

#ifndef CODE_GARAGE_H_
#define CODE_GARAGE_H_
#include "zf_common_typedef.h"

void garage(int choice);
uint8 judge_garage(void);
uint8 judge_garage_two(void);

void speed_control(int error_m);
extern uint8 garage_state;
extern uint8 garage_count;
extern uint8 garage_left_flag;
extern uint8 garage_right_flag;

extern uint8 garage_wide;
extern uint8 garage_num;
extern uint8 garage_row;


void chuku(void);

#endif /* CODE_GARAGE_H_ */
