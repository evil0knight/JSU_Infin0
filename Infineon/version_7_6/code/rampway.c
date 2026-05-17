/*
 * rampway.c
 *
 *  Created on: 2023Äê6ÔÂ18ÈÕ
 *      Author: 11150
 */
#include "extern.h"

#include "rampway.h"
#include "zf_common_headfile.h"

#include"roadblock.h"
#include"garage.h"

inflection_crossroads_point left_guai_crossroads[2],right_guai_crossroads[2];



uint8 rampway_state;
int rampway_Pixle_num;
uint8 rampway_Pixle_flag;


extern float speed_alone;

extern float SpeedI;
extern float SpeedP;




uint8 crossroads_flag;
uint8 crossroads_state;





























