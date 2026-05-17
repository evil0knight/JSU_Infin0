/*
 * Inverse_kinematics.h
 *
 *  Created on: 2024Äê11ÔÂ16ÈÕ
 *      Author: 17104
 */

#ifndef CODE_INVERSE_KINEMATICS_H_
#define CODE_INVERSE_KINEMATICS_H_

#include "zf_common_headfile.h"

typedef struct
{
    float x;
    float y;

    float L1;
    float L2;
    float L3;
    float L4;
    float L5;

    float front_radian;
    float rear_radian;

    float front_angle;
    float rear_angle;
}wheel_coordinate_struct;

void wheel_coordinate_struct_init(wheel_coordinate_struct * wheel, float x, float y);
void Inverse_kinematics(wheel_coordinate_struct * wheel);

#endif /* CODE_INVERSE_KINEMATICS_H_ */
