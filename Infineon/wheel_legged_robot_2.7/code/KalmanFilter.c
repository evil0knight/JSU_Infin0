/*
 * KalmanFilter.c
 *
 *  Created on: 2024年10月19日
 *      Author: 17104
 */

#include "KalmanFilter.h"

//#include <stdio.h>

//一维卡尔曼结构体声明示例
//KalmanFilter K;

//初始化一维卡尔曼滤波器
//示例:kalman_init(&K, initial_x, initial_p, q, r);
void kalman_init(KalmanFilter *kf, float initial_x, float initial_p, float q, float r) {
    kf->x = initial_x;
    kf->p = initial_p;
    kf->q = q;
    kf->r = r;
}

//一维卡尔曼滤波更新
//示例:float res = kalman_update(&K, Photoelectric_sensor_Refresh(L2_Port));
float kalman_update(KalmanFilter *kf, float measurement) {
    // 预测步骤
    // 在这个例子中，我们假设状态转移矩阵F是1（即状态不变），控制向量B和控制输入u都是0（即没有控制输入）
    // 因此，预测的状态就是当前的状态估计值x
    // 预测误差协方差P_predict = P + Q
    kf->p += kf->q;

    // 更新步骤
    // 计算卡尔曼增益K = P_predict / (P_predict + R)
    kf->k = kf->p / (kf->p + kf->r);

    // 更新状态估计值x_update = x + K * (measurement - x)
    kf->x += kf->k * (measurement - kf->x);

    // 更新预测误差协方差P_update = (1 - K) * P_predict
    kf->p *= (1 - kf->k);

    return kf->x; // 返回更新后的状态估计值
}

void kalman_change(KalmanFilter *kf,float q, float r)
{
    kf->q = q;
    kf->r = r;
}
