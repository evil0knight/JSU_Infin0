/*
 * KF_imu_encoder_fusion.h
 *
 *  Created on: 2025年4月26日
 *      Author: 17104
 */

#ifndef CODE_KF_IMU_ENCODER_FUSION_H_
#define CODE_KF_IMU_ENCODER_FUSION_H_

#include "zf_common_headfile.h"
#include "matrix.h"
#include "LQR.h"
#include "foc.h"

//#define My_PI 3.14159265f
#define ACC_THRESHOLD 0.3f  // 加速度阈值，用于判断离地
#define GYRO_THRESHOLD 0.1f // 陀螺仪阈值，用于判断运动状态

// 状态向量维度
#define STATE_DIM 9
// 测量向量维度
#define MEASURE_DIM 6

// 状态向量索引
#define VX_IDX 0
#define VY_IDX 1
#define VZ_IDX 2
#define ROLL_IDX 3
#define PITCH_IDX 4
#define YAW_IDX 5
#define ROLL_RATE_IDX 6
#define PITCH_RATE_IDX 7
#define YAW_RATE_IDX 8

// 测量向量索引
#define ENCODER_VEL_IDX 0
#define AX_IDX 1
#define AY_IDX 2
#define AZ_IDX 3
#define GX_IDX 4
#define GY_IDX 5

//typedef struct {
//    // IMU数据
//    float gx, gy, gz;    // 角速度(rad/s)
//    float ax, ay, az;     // 加速度(m/s^2)
//    float mx, my, mz;     // 磁力计数据
//
//    // 编码器数据
//    float speed[2];       // 左右轮速度(m/s)
//
//    // 状态向量 [vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
//    matrix_struct Xk;      // 后验估计 (STATE_DIM x 1)
//    matrix_struct Xk_;     // 先验估计 (STATE_DIM x 1)
//
//    // 协方差矩阵
//    matrix_struct Pk;      // 后验估计误差协方差 (STATE_DIM x STATE_DIM)
//    matrix_struct Pk_;     // 先验估计误差协方差 (STATE_DIM x STATE_DIM)
//
//    // 卡尔曼增益
//    matrix_struct K;       // (STATE_DIM x MEASURE_DIM)
//
//    // 噪声协方差
//    matrix_struct Q;       // 过程噪声协方差 (STATE_DIM x STATE_DIM)
//    matrix_struct R;       // 测量噪声协方差 (MEASURE_DIM x MEASURE_DIM)
//
//    // 状态转移矩阵和测量矩阵
//    matrix_struct F;       // 状态转移矩阵 (STATE_DIM x STATE_DIM)
//    matrix_struct H;       // 测量矩阵 (MEASURE_DIM x STATE_DIM)
//
//    float T;              // 采样时间(s)
//
//    // 输出状态
//    float velocity;       // 融合后的速度(m/s)
//    float yaw;           // 融合后的偏航角(rad)
//    float pitch;         // 融合后的俯仰角(rad)
//    float roll;          // 融合后的滚转角(rad)
//    float yaw_rate;      // 偏航角速度(rad/s)
//    float pitch_rate;    // 俯仰角速度(rad/s)
//    float roll_rate;     // 滚转角速度(rad/s)
//    float yaw_acc;       // 偏航角加速度(rad/s^2)
//    float pitch_acc;     // 俯仰角加速度(rad/s^2)
//    float roll_acc;      // 滚转角加速度(rad/s^2)
//    int wheel_lifted;    // 轮子是否离地(0:否,1:是)
//
//    // 用于计算角加速度的上一次角速度
//    float prev_roll_rate;
//    float prev_pitch_rate;
//    float prev_yaw_rate;
//} robot_state_struct;
typedef struct {
    // IMU数据
    float gx, gy, gz;    // 角速度(rad/s)
    float ax, ay, az;     // 加速度(m/s^2)

    // 编码器数据
    float speed[2];       // 左右轮速度(m/s)

    // 状态向量 [vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
    float Xk[STATE_DIM];      // 后验估计
    float Xk_[STATE_DIM];     // 先验估计

    // 协方差矩阵
    float Pk[STATE_DIM][STATE_DIM];      // 后验估计误差协方差
    float Pk_[STATE_DIM][STATE_DIM];     // 先验估计误差协方差

    // 卡尔曼增益
    float K[STATE_DIM][MEASURE_DIM];

    // 噪声协方差
    float Q[STATE_DIM][STATE_DIM];       // 过程噪声协方差
    float R[MEASURE_DIM][MEASURE_DIM];   // 测量噪声协方差

    float T;              // 采样时间(s)

    // 输出状态
    float velocity;       // 融合后的速度(m/s)
    float yaw;           // 融合后的偏航角(rad)
    float pitch;         // 融合后的俯仰角(rad)
    float roll;          // 融合后的滚转角(rad)
    float yaw_rate;      // 偏航角速度(rad/s)
    float pitch_rate;    // 俯仰角速度(rad/s)
    float roll_rate;     // 滚转角速度(rad/s)
    float yaw_acc;       // 偏航角加速度(rad/s^2)
    float pitch_acc;     // 俯仰角加速度(rad/s^2)
    float roll_acc;      // 滚转角加速度(rad/s^2)
    int wheel_lifted;    // 轮子是否离地(0:否,1:是)

    // 用于计算角加速度的上一次角速度
    float prev_roll_rate;
    float prev_pitch_rate;
    float prev_yaw_rate;
} robot_state_struct;
extern robot_state_struct robot;//robot->pitch

void robot_state_init(robot_state_struct *robot, float q, float r, float T);    // 初始化函数
void robot_state_update(robot_state_struct *robot);                             // 状态更新函数
// 简化矩阵乘法 - 专门为STATE_DIM x STATE_DIM矩阵优化
void matrix_multiply_9x9(const float a[9][9], const float b[9][9], float result[9][9]);
// 简化矩阵乘法 - STATE_DIM x MEASURE_DIM
void matrix_multiply_9x6(const float a[9][9], const float b[9][6], float result[9][6]);
// 简化矩阵乘法 - MEASURE_DIM x STATE_DIM
void matrix_multiply_6x9(const float a[6][9], const float b[9][9], float result[6][9]);
// 简化矩阵乘法 - STATE_DIM x MEASURE_DIM * MEASURE_DIM x 1
void matrix_multiply_9x6_6x1(const float a[9][6], const float b[6], float result[9]);
// 简化矩阵转置 - STATE_DIM x STATE_DIM
void matrix_transpose_9x9(const float a[9][9], float result[9][9]);
// 简化矩阵转置 - MEASURE_DIM x STATE_DIM
void matrix_transpose_6x9(const float a[6][9], float result[9][6]);
// 简化矩阵加法 - STATE_DIM x STATE_DIM
void matrix_add_9x9(const float a[9][9], const float b[9][9], float result[9][9]);
// 简化矩阵减法 - STATE_DIM x STATE_DIM
void matrix_sub_9x9(const float a[9][9], const float b[9][9], float result[9][9]);
// 简化矩阵求逆 - 针对对称正定矩阵优化 (6x6)
void matrix_inv_6x6(const float a[6][6], float result[6][6]);
//// 示例使用
//int main() {
//    robot_state_struct robot;
//
//    // 初始化，设置过程噪声和测量噪声，采样时间0.01s
//    robot_state_init(&robot, 0.1f, 0.5f, 0.01f);
//
//    // 模拟数据更新循环
//    for(int i = 0; i < 100; i++) {
//        // 模拟更新IMU和编码器数据
//        robot.gx = 0.1f * sinf(i * 0.1f);
//        robot.gy = 0.05f * cosf(i * 0.1f);
//        robot.gz = 0.2f;  // 模拟偏航角速度
//
//        robot.ax = 0.1f * sinf(i * 0.2f);
//        robot.ay = 0.1f * cosf(i * 0.2f);
//        robot.az = GRAVITY + ((i > 50 && i < 70) ? 2.0f : 0.0f);  // 模拟离地情况
//
//        robot.speed[0] = 0.5f + 0.1f * sinf(i * 0.1f);
//        robot.speed[1] = 0.5f + 0.1f * cosf(i * 0.1f);
//
//        // 更新状态估计
//        robot_state_update(&robot);
//
//        // 打印结果
//        printf("Step %d:\n", i);
//        printf("  Velocity: %.3f m/s\n", robot.velocity);
//        printf("  Roll: %.3f rad (%.1f°)\n", robot.roll, robot.roll * 180.0f / My_PI);
//        printf("  Pitch: %.3f rad (%.1f°)\n", robot.pitch, robot.pitch * 180.0f / My_PI);
//        printf("  Yaw: %.3f rad (%.1f°)\n", robot.yaw, robot.yaw * 180.0f / My_PI);
//        printf("  Roll rate: %.3f rad/s\n", robot.roll_rate);
//        printf("  Pitch rate: %.3f rad/s\n", robot.pitch_rate);
//        printf("  Yaw rate: %.3f rad/s\n", robot.yaw_rate);
//        printf("  Roll acc: %.3f rad/s²\n", robot.roll_acc);
//        printf("  Pitch acc: %.3f rad/s²\n", robot.pitch_acc);
//        printf("  Yaw acc: %.3f rad/s²\n", robot.yaw_acc);
//        printf("  Wheel lifted: %d\n\n", robot.wheel_lifted);
//    }
//
//    return 0;
//}



#endif /* CODE_KF_IMU_ENCODER_FUSION_H_ */
