/*
 * KalmanFilter.h
 *
 *  Created on: 2024年10月19日
 *      Author: 17104
 */

#ifndef CODE_KALMANFILTER_H_
#define CODE_KALMANFILTER_H_


#include "zf_common_headfile.h"

// 卡尔曼滤波结构体
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 状态估计值
    float p; // 状态估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;


void kalman_init(KalmanFilter *kf, float initial_x, float initial_p, float q, float r);
float kalman_update(KalmanFilter *kf, float measurement);
void kalman_change(KalmanFilter *kf,float q, float r);


#endif /* CODE_KALMANFILTER_H_ */
