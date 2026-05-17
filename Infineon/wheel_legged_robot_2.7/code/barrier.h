/*
 * barrier.h
 *
 *  Created on: 2024Äê10ÔÂ12ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_BARRIER_H_
#define USER_CAMERA_BARRIER_H_

#define BEGINH_BRIDGE_R (30)
#define BEGINH_BRIDGE_L (100)

extern int32 bridge_flag;

enum barrier_type_e {
  BARRIER_NONE,
  BARRIER_BRIDGE_IN,
  BARRIER_BESIDES_LEFT_BEGIN,
  BARRIER_BESIDES_RIGHT_BEGIN,
  BARRIER_BESIDES_LEFT_RUNNING,
  BARRIER_BESIDES_RIGHT_RUNNING,
  BARRIER_JUMP_BEGIN,
  BARRIER_JUMP_IN,
  BARRIER_JUMP_OUT,
  BARRIER_LEFT_OUT,
  BARRIER_RIGHT_OUT,
};

enum s_type_e {
  S_NONE,
  S_BEGIN,
  S_RUNNING,
  S_OUT,
};

enum shield_type_e {
  SHIELD_NONE,
  SHIELD_BEGIN,
};

extern enum barrier_type_e barrier_type;
extern enum s_type_e s_type;
extern enum shield_type_e shield_type;
extern float (*mid_track_s)[2];
extern int32_t mid_track_count_s;
extern float transection_distance;
extern int temp_min_s;
extern int temp_max_s;
extern float barrirer_time;
void CheckBarrier(void);
void Check_s(void);
void RunBarrier(void);

#endif /* USER_CAMERA_BARRIER_H_ */
