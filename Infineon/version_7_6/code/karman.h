/*
 * karman.h
 *
 *  Created on: 2022Äê11ÔÂ23ÈÕ
 *      Author: 11150
 */

#ifndef CODE_KARMAN_H_
#define CODE_KARMAN_H_



float KalmanFilter(float Accel,float Gyro);
float KalmanFilter1(float Accel1,float Gyro1);
float angle_calc(float angle_m, float gyro_m);
float angle_calc1(float angle_m, float gyro_m);

#endif /* CODE_KARMAN_H_ */
