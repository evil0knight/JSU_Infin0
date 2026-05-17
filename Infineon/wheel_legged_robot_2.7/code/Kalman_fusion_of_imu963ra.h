/*
 * Kalman_fusion_of_imu660ra.h
 *
 *  Created on: 2024年11月22日
 *      Author: 17104
 */

#ifndef CODE_KALMAN_FUSION_OF_IMU660RA_H_
#define CODE_KALMAN_FUSION_OF_IMU660RA_H_

#include "zf_common_headfile.h"

#define My_PI   3.141592653f
#define My_PI_2 1.570796327f
#define MAX_SPEED 20.0f             //最大合法速度
#define SLIP_THRESHOLD 0.2f         //滑移率打滑判断阈值
#define CURVE_THRESHOLD_YAW 0.3f    //转向偏航角速度阈值 rad/s
#define CURVE_THRESHOLD_ACC 1.0f    //转向横向加速度阈值 m/s^2
#define SPEED_REVERSE_THRESH 0.1f   //速度反转认定阈值
#define ACC_THRESH  5.0f            //最大合法加速度 m/s^2
#define MAX_READ_VALUE 10.0f        //最大读取合法值
typedef struct
{
    float roll;//欧拉角
    float pitch;
    float yaw;

    float gx;//角速度
    float gy;
    float gz;
    float ax;//加速度
    float ay;
    float az;
    float mx;//磁力
    float my;
    float mz;

    float Xk_[3];// = {0};//先验估计
    float Xk[3];// = {0};//后验估计
    float Uk[3];// = {0};//系统输入
    float Zk[3];// = {0};//测量状态
    float Pk[3];// = {1,1,1};//后验估计误差协方差
    float Pk_[3];// = {0};//先验估计误差协方差
    float K[3];// = {0};//卡尔曼增益
    float Q[3];// = {1,1,1};//系统噪声协方差
    float R[3];// = {1,1,1};//测量噪声协方差

    float ax_linear; // 去除重力后的线性加速度(m/s^2)
    float ay_linear;
    float az_linear;

    float T;// = 0.002f;//离散时间
    float resultant_acceleration;
}imu963ra_struct;
extern imu963ra_struct imu;

void imu963ra_kalman_filter_init(imu963ra_struct * imu, float q, float r, float T);
void imu963ra_kalman_filter_update(imu963ra_struct * imu);

// 卡尔曼滤波器结构体定义
typedef struct {
    float Xk_[2];    // 先验估计：[位移, 速度]
    float Xk[2];     // 后验估计
    float Pk_[2][2]; // 先验误差协方差矩阵
    float Pk[2][2];  // 后验误差协方差矩阵
    float K[2];      // 卡尔曼增益
    float Q[2][2];   // 过程噪声协方差
    float R;         // 测量噪声方差
    float T;         // 采样周期

    float original_R;
    float original_Q[2][2];
    float direction_factor;
    float slip_factor; // 动态滑移系数 [0~1]
    float slip_ratio;
    int is_slip;

    float est_displacement;
    float est_velocity;
} KF_Velocity;
extern KF_Velocity vel_kf;

void imu963ra_menc15a_kalman_filter_init(KF_Velocity* kf, float q_pos, float q_vel, float r, float T);
void imu963ra_menc15a_kalman_filter_Update(KF_Velocity* kf, float measured_speed, float linear_accel);
void handle_slip_condition(KF_Velocity* kf, float measured_speed, float yaw_rate, float lat_accel);
int validate_acceleration(float imu_accel, float speed_est, float measured_speed, float dt);

#endif /* CODE_KALMAN_FUSION_OF_IMU660RA_H_ */
