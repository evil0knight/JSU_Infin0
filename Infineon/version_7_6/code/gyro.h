/*
 * gyro.h
 *
 *  Created on: 2022Äê11ÔÂ22ÈÕ
 *      Author: 11150
 */

#ifndef CODE_GYRO_H_
#define CODE_GYRO_H_
void jiaodu(void);

float Get_Gyro_Fy(void);
float Get_Gyro_Fg(void);
float Get_Gyro_Z(void);
float Get_Attitude_Fy(void);
float Get_Attitude_Fg(void);
float Turn_loop_right(int Bias,int sped);//ÓÒÑ¹Íä
float Turn_loop_left(int Bias,int sped);//×óÑ¹Íä
float bend(float Bias,float speed);//Ñ¹Íä


void zero_get_init(void);
void Get_Angle_Z(void);

#endif /* CODE_GYRO_H_ */
