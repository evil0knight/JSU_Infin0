/*
 * state.h
 *
 *  Created on: 2024Äê10ÔÂ7ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_STATE_H_
#define USER_CAMERA_STATE_H_

enum state_type_e {
    COMMON_STATE,
    TURN_STATE,
    STRAIGHT_STATE,
    CIRCLE_STATE,
    CIRCLE_RUNNING_STATE,
    BARRIER_STATE,
    LOST_STATE,
    STOP_STATE
};
extern enum state_type_e last_state;
extern enum state_type_e state_type;
extern float cricle_aim;
extern float cross_aim;
extern float common_aim;
extern float straight_aim;
extern float turn_aim;
extern float mid_aim;
extern float barrier_aim;
extern float barrier_offset;




#endif /* USER_CAMERA_STATE_H_ */
