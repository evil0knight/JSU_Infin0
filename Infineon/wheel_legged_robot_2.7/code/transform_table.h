/*
 * transform_table.h
 *
 *  Created on: 2024Äê10ÔÂ7ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_TRANSFORM_TABLE_H_
#define USER_CAMERA_TRANSFORM_TABLE_H_

//extern  float M[3][3] ;
//extern  const float InverseMapH[120][188] ;
//extern  const float InverseMapW[120][188] ;
//extern float inv[2];
extern float K[3][3];
extern float inv[2];
//void transform(float X, float Y, int* x, int* y);
//void transform_imu(float M_tran[3][3],float YAW);

void external_reference_update(void);
void adaptive_reverse_perspective(int u, int v);

#endif /* USER_CAMERA_TRANSFORM_TABLE_H_ */
