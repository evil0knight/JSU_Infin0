/*
 * lost.c
 *
 *  Created on: 2024Äê11ÔÂ25ÈÕ
 *      Author: A
 */

#include "headfile.h"

enum lost_type_e lost_type=LOST_NONE;

void CheckLost(){
    if(circle_type != CIRCLE_LEFT_OUT || circle_type != CIRCLE_RIGHT_OUT || circle_type != CIRCLE_LEFT_IN || circle_type != CIRCLE_RIGHT_IN){
        if(cross_type!=CROSS_BEGIN &&cross_type!= CROSS_IN){
            if(pts_left_count<=3 && pts_right_count<=3 && pts_far_left_count<=3 && pts_far_right_count<=3){
                lost_type=LOST_YES;
            }
            else
                lost_type=LOST_NONE;
        }
    }
}
