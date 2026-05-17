/*
 * Base.h
 *
 *  Created on: 2024Äê11ÔÂ24ÈÕ
 *      Author: 17104
 */

#ifndef CODE_BASE_H_
#define CODE_BASE_H_

#include "AAA_Blessed_by_the_Buddha_there_will_never_be_any_bugs_AAA.h"
#include "zf_common_headfile.h"
#include <steering_engine.h>
#include <Inverse_kinematics.h>
#include <Kalman_fusion_of_imu963ra.h>
//#include <universal_filter.h>
#include "KalmanFilter.h"
#include "foc.h"
#include "headfile.h"
#include "state.h"
#include "LQR.h"
#include <menu.h>
#include "logger.h"
#include "PID.h"

#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_4 0.78539816340f
#define _PI_6 0.52359877559f

#define acceleration_acceleration 1.2
#define braking_acceleration 0.1
//#define steering_acceleration (3.1)
#define steering_acceleration 3.5
#define MAX_TORQUE_DIFFERENCE 1.5f

extern imu963ra_struct imu;
extern float speed[2];
extern enum state_type_e state_type;

extern Matrix COMMON_Q;
extern Matrix COMMON_R;
extern Matrix COMMON_K;
extern Matrix STABLE_K;
extern float    KL[7];
extern float    KR[7];
extern float    Ka[7];

typedef struct{
    float Car_zero;
    float max_speed;
    float horizontal_E;
    int flag;
    int prev_flag;
    int track;
    float batter_voltage;
    float actual_target_speed;
}Base_input_data;

extern wheel_coordinate_struct wheel_L;
extern wheel_coordinate_struct wheel_R;

extern float Instantaneous_speed;
extern Base_input_data input;
void Base_init(float Car_zero, float max_speed);
void Base_run(void);
void steering_engine_update(void);
void brushless_motor_update(void);
void base_get_speed(void);
void detect_terrain(float now_gy);

extern float standardized_curvature_now;
extern float standardized_curvature_ave;
extern float actual_max_speed;
extern float eva_speed;
extern float max_speed;
extern float delta_y;
extern float torque_compensation;
extern float Uq[2];
extern float alpha;
extern float begin_yaw;
extern int slope_status;
extern float now_gx;
extern float now_gy;
extern float now_gz;
//extern float resultant_acceleration;
//extern directory all_dir[dir_num];


#endif /* CODE_BASE_H_ */
