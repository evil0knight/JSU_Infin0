/*
 * roundabout.c
 *
 *  Created on: 2023Äê7ÔÂ20ÈÕ
 *      Author: 11150
 */




#include "roundabout.h"
#include "extern.h"

#include "roadblock.h"

#include "rampway.h"
//#include "zf_common_headfile.h"
//#include "roundabout.h"
uint8   left_loss_line_flag;
uint8   right_loss_line_flag;
uint8   left_straight_line_flag;
uint8   right_straight_line_flag;

uint8 roundabout_state=10;
uint8 roundabout_midpoint_state;

uint8   roundabout_L_flag;
uint8   roundabout_R_flag;

uint8 roundabout_midpoint_up_num;
uint8 roundabout_midpoint_down_num;







int bbb=1;
void protects(void)
{
    if(bbb==1)
    {



        distance_state=1;

if(distance>500)
    {roundabout_state=0;
distance_state=0;
        distance=0;
        bbb=0;
    }


    }

}
















