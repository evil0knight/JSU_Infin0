/*
 * KF_imu_encoder_fusion.c
 *
 *  Created on: 2025年4月26日
 *      Author: 17104
 */

// 初始化函数

#include "KF_imu_encoder_fusion.h"

robot_state_struct robot;

// 简化矩阵乘法 - 专门为STATE_DIM x STATE_DIM矩阵优化
void matrix_multiply_9x9(const float a[9][9], const float b[9][9], float result[9][9]) {
    for(int i = 0; i < 9; i++) {
        for(int j = 0; j < 9; j++) {
            result[i][j] = 0.0f;
            for(int k = 0; k < 9; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 简化矩阵乘法 - STATE_DIM x MEASURE_DIM
void matrix_multiply_9x6(const float a[9][9], const float b[9][6], float result[9][6]) {
    for(int i = 0; i < 9; i++) {
        for(int j = 0; j < 6; j++) {
            result[i][j] = 0.0f;
            for(int k = 0; k < 9; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 简化矩阵乘法 - MEASURE_DIM x STATE_DIM
void matrix_multiply_6x9(const float a[6][9], const float b[9][9], float result[6][9]) {
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 9; j++) {
            result[i][j] = 0.0f;
            for(int k = 0; k < 9; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 简化矩阵乘法 - STATE_DIM x MEASURE_DIM * MEASURE_DIM x 1
void matrix_multiply_9x6_6x1(const float a[9][6], const float b[6], float result[9]) {
    for(int i = 0; i < 9; i++) {
        result[i] = 0.0f;
        for(int k = 0; k < 6; k++) {
            result[i] += a[i][k] * b[k];
        }
    }
}

// 简化矩阵转置 - STATE_DIM x STATE_DIM
void matrix_transpose_9x9(const float a[9][9], float result[9][9]) {
    for(int i = 0; i < 9; i++) {
        for(int j = 0; j < 9; j++) {
            result[j][i] = a[i][j];
        }
    }
}

// 简化矩阵转置 - MEASURE_DIM x STATE_DIM
void matrix_transpose_6x9(const float a[6][9], float result[9][6]) {
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 9; j++) {
            result[j][i] = a[i][j];
        }
    }
}

// 简化矩阵加法 - STATE_DIM x STATE_DIM
void matrix_add_9x9(const float a[9][9], const float b[9][9], float result[9][9]) {
    for(int i = 0; i < 9; i++) {
        for(int j = 0; j < 9; j++) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

// 简化矩阵减法 - STATE_DIM x STATE_DIM
void matrix_sub_9x9(const float a[9][9], const float b[9][9], float result[9][9]) {
    for(int i = 0; i < 9; i++) {
        for(int j = 0; j < 9; j++) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

// 简化矩阵求逆 - 针对对称正定矩阵优化 (6x6)
void matrix_inv_6x6(const float a[6][6], float result[6][6]) {
    // 这里实现一个简化的矩阵求逆算法
    // 实际应用中可能需要更稳健的实现
//    float det = 0.0f;
    // 计算行列式(简化版)

    // 如果是对角矩阵，直接取倒数
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            result[i][j] = (i == j) ? (1.0f / a[i][i]) : 0.0f;
        }
    }
}

void robot_state_init(robot_state_struct *robot, float q, float r, float T) {
    imu963ra_init();
    menc15a_init();

    // 首先获取初始IMU数据用于校准
    imu963ra_get_acc();
    imu963ra_get_gyro();

    // 获取初始加速度数据并转换为m/s²
    float ax = imu963ra_acc_transition(imu963ra_acc_x) * GRAVITY;
    float ay = imu963ra_acc_transition(imu963ra_acc_y) * GRAVITY;
    float az = imu963ra_acc_transition(imu963ra_acc_z) * GRAVITY;

    // 使用重力矢量计算初始姿态角
    // Roll角(绕X轴旋转): atan2(ay, az)
    // Pitch角(绕Y轴旋转): atan2(-ax, sqrt(ay*ay + az*az))
    float initial_roll = atan2f(ay, az);
    float initial_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 初始化IMU和编码器数据
    robot->gx = robot->gy = robot->gz = 0.0f;
    robot->ax = ax;
    robot->ay = ay;
    robot->az = az;
    robot->speed[0] = robot->speed[1] = 0.0f;

    // 初始化状态向量 - 使用重力校准后的初始姿态
    for(int i = 0; i < STATE_DIM; i++) {
        robot->Xk[i] = 0.0f;
        robot->Xk_[i] = 0.0f;
    }
    robot->Xk[ROLL_IDX] = initial_roll;
    robot->Xk[PITCH_IDX] = initial_pitch;
    robot->Xk_[ROLL_IDX] = initial_roll;
    robot->Xk_[PITCH_IDX] = initial_pitch;

    // 初始化协方差矩阵
    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < STATE_DIM; j++) {
            robot->Pk[i][j] = (i == j) ? 1.0f : 0.0f;
            robot->Pk_[i][j] = 0.0f;
        }
    }

    // 姿态角的初始不确定性可以设置得小一些，因为我们已经校准
    robot->Pk[ROLL_IDX][ROLL_IDX] = 0.01f;
    robot->Pk[PITCH_IDX][PITCH_IDX] = 0.01f;
    robot->Pk_[ROLL_IDX][ROLL_IDX] = 0.01f;
    robot->Pk_[PITCH_IDX][PITCH_IDX] = 0.01f;

    // 其余初始化保持不变...
    // ... [保持原有的卡尔曼增益、噪声协方差等初始化代码]

    robot->T = T;
    robot->velocity = 0.0f;
    robot->yaw = 0.0f;
    robot->pitch = initial_pitch;
    robot->roll = initial_roll;
    robot->yaw_rate = 0.0f;
    robot->pitch_rate = 0.0f;
    robot->roll_rate = 0.0f;
    robot->yaw_acc = 0.0f;
    robot->pitch_acc = 0.0f;
    robot->roll_acc = 0.0f;
    robot->wheel_lifted = 0;

    robot->prev_roll_rate = 0.0f;
    robot->prev_pitch_rate = 0.0f;
    robot->prev_yaw_rate = 0.0f;

    printf("Initial calibration: roll=%f, pitch=%f\n", initial_roll, initial_pitch);
}

void robot_state_update(robot_state_struct *robot) {
    // 获取传感器数据
    imu963ra_get_acc();
    imu963ra_get_gyro();
    robot->gx = imu963ra_gyro_transition(imu963ra_gyro_x) * _PI / 180.0f;
    robot->gy = imu963ra_gyro_transition(imu963ra_gyro_y) * _PI / 180.0f;
    robot->gz = imu963ra_gyro_transition(imu963ra_gyro_z) * _PI / 180.0f;
    robot->ax = imu963ra_acc_transition(imu963ra_acc_x) * GRAVITY;
    robot->ay = imu963ra_acc_transition(imu963ra_acc_y) * GRAVITY;
    robot->az = imu963ra_acc_transition(imu963ra_acc_z) * GRAVITY;

//    printf("%f,%f,%f\n",robot -> gx,robot -> gy,robot -> gz);
//    printf("%f,%f,%f\n",robot -> ax,robot -> ay,robot -> az);
    robot->speed[0] = ((float)menc15a_absolute_offset_data[menc15a_1_module] * _2PI / 32768.0f * 1000.0f / pit_time0_ms + robot->gx) * WHEEL_RADIUS / 2.0f;
    robot->speed[1] = ((float)menc15a_absolute_offset_data[menc15a_2_module] * _2PI / 32768.0f * 1000.0f / pit_time0_ms + robot->gx) * WHEEL_RADIUS / -2.0f;
//    printf("%f,%f\n",robot -> speed[0],robot -> speed[1]);
    // 1. 预测步骤 (状态预测)
    // 简化状态转移模型
    robot->Xk_[VX_IDX] = robot->Xk[VX_IDX];
    robot->Xk_[VY_IDX] = robot->Xk[VY_IDX];
    robot->Xk_[VZ_IDX] = robot->Xk[VZ_IDX];
    float dt = robot->T;

    // 使用改进的欧拉角微分方程
    robot->Xk_[ROLL_IDX] = robot->Xk[ROLL_IDX] + dt * (
        robot->gx +
        sinf(robot->Xk[ROLL_IDX]) * tanf(robot->Xk[PITCH_IDX]) * robot->gy +
        cosf(robot->Xk[ROLL_IDX]) * tanf(robot->Xk[PITCH_IDX]) * robot->gz
    );

    robot->Xk_[PITCH_IDX] = robot->Xk[PITCH_IDX] + dt * (
        cosf(robot->Xk[ROLL_IDX]) * robot->gy -
        sinf(robot->Xk[ROLL_IDX]) * robot->gz
    );

    robot->Xk_[YAW_IDX] = robot->Xk[YAW_IDX] + dt * (
        sinf(robot->Xk[ROLL_IDX]) / cosf(robot->Xk[PITCH_IDX]) * robot->gy +
        cosf(robot->Xk[ROLL_IDX]) / cosf(robot->Xk[PITCH_IDX]) * robot->gz
    );

    robot->Xk_[ROLL_RATE_IDX] = robot->gx;
    robot->Xk_[PITCH_RATE_IDX] = robot->gy;
    robot->Xk_[YAW_RATE_IDX] = robot->gz;

    // 2. 预测步骤 (协方差预测)
    // Pk_ = F * Pk * F' + Q
    // 简化处理，直接加上过程噪声
    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < STATE_DIM; j++) {
            robot->Pk_[i][j] = robot->Pk[i][j] + robot->Q[i][j];
        }
    }

    // 3. 更新步骤 (计算卡尔曼增益)
    // 计算 H * Pk_ * H' + R
    float HPHT[MEASURE_DIM][MEASURE_DIM];
    for(int i = 0; i < MEASURE_DIM; i++) {
        for(int j = 0; j < MEASURE_DIM; j++) {
            for(int k = 0; k < STATE_DIM; k++) {
                // 简化H矩阵 - 只有对角线元素
                if((i == ENCODER_VEL_IDX && k == VX_IDX) ||
                   (i == AX_IDX && k == AX_IDX) ||
                   (i == AY_IDX && k == AY_IDX) ||
                   (i == AZ_IDX && k == AZ_IDX) ||
                   (i == GX_IDX && k == ROLL_RATE_IDX) ||
                   (i == GY_IDX && k == PITCH_RATE_IDX)) {
                    for(int l = 0; l < STATE_DIM; l++) {
                        // 简化H矩阵转置
                        if((j == ENCODER_VEL_IDX && l == VX_IDX) ||
                           (j == AX_IDX && l == AX_IDX) ||
                           (j == AY_IDX && l == AY_IDX) ||
                           (j == AZ_IDX && l == AZ_IDX) ||
                           (j == GX_IDX && l == ROLL_RATE_IDX) ||
                           (j == GY_IDX && l == PITCH_RATE_IDX)) {
                            HPHT[i][j] += robot->Pk_[k][l];
                        }
                    }
                }
            }
            HPHT[i][j] += robot->R[i][j];
        }
    }

    // 计算 inv(HPHT + R)
    float HPHT_inv[MEASURE_DIM][MEASURE_DIM];
    matrix_inv_6x6(HPHT, HPHT_inv);

    // 计算 K = Pk_ * H' * HPHT_inv
    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < MEASURE_DIM; j++) {
            robot->K[i][j] = 0.0f;
            for(int k = 0; k < STATE_DIM; k++) {
                // 简化H矩阵转置
                if((j == ENCODER_VEL_IDX && k == VX_IDX) ||
                   (j == AX_IDX && k == AX_IDX) ||
                   (j == AY_IDX && k == AY_IDX) ||
                   (j == AZ_IDX && k == AZ_IDX) ||
                   (j == GX_IDX && k == ROLL_RATE_IDX) ||
                   (j == GY_IDX && k == PITCH_RATE_IDX)) {
                    robot->K[i][j] += robot->Pk_[i][k];
                }
            }
            float sum = 0.0f;
            for(int k = 0; k < MEASURE_DIM; k++) {
                sum += robot->K[i][k] * HPHT_inv[k][j];
            }
            robot->K[i][j] = sum;
        }
    }

    // 4. 更新步骤 (状态更新)
    // 测量值Zk
    float Zk[MEASURE_DIM] = {
        (robot->speed[0] + robot->speed[1]) / 2.0f,  // 编码器速度

        // 基于当前姿态预测的重力分量
        sinf(robot->Xk_[PITCH_IDX]),                         // 预测的ax/g
        -sinf(robot->Xk_[ROLL_IDX]) * cosf(robot->Xk_[PITCH_IDX]), // 预测的ay/g
        cosf(robot->Xk_[ROLL_IDX]) * cosf(robot->Xk_[PITCH_IDX]),  // 预测的az/g

        robot->gx,                                   // 陀螺仪x
        robot->gy                                    // 陀螺仪y
    };

    // 计算加速度残差时使用归一化的测量值
    float innovation[MEASURE_DIM];
    innovation[ENCODER_VEL_IDX] = Zk[ENCODER_VEL_IDX] - robot->Xk_[VX_IDX];
    innovation[AX_IDX] = (robot->ax/GRAVITY) - Zk[AX_IDX];
    innovation[AY_IDX] = (robot->ay/GRAVITY) - Zk[AY_IDX];
    innovation[AZ_IDX] = (robot->az/GRAVITY) - Zk[AZ_IDX];
    innovation[GX_IDX] = robot->gx - robot->Xk_[ROLL_RATE_IDX];
    innovation[GY_IDX] = robot->gy - robot->Xk_[PITCH_RATE_IDX];

    // Xk = Xk_ + K * innovation
    for(int i = 0; i < STATE_DIM; i++) {
        robot->Xk[i] = robot->Xk_[i];
        for(int j = 0; j < MEASURE_DIM; j++) {
            robot->Xk[i] += robot->K[i][j] * innovation[j];
        }
    }

    // 5. 更新步骤 (协方差更新)
    // Pk = (I - K * H) * Pk_
    // 简化处理，直接使用 Pk = (I - K * H) * Pk_
    float temp[STATE_DIM][STATE_DIM];
    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < STATE_DIM; j++) {
            temp[i][j] = (i == j) ? 1.0f : 0.0f;
            for(int k = 0; k < MEASURE_DIM; k++) {
                // 简化H矩阵
                if((k == ENCODER_VEL_IDX && j == VX_IDX) ||
                   (k == AX_IDX && j == AX_IDX) ||
                   (k == AY_IDX && j == AY_IDX) ||
                   (k == AZ_IDX && j == AZ_IDX) ||
                   (k == GX_IDX && j == ROLL_RATE_IDX) ||
                   (k == GY_IDX && j == PITCH_RATE_IDX)) {
                    temp[i][j] -= robot->K[i][k];
                }
            }
        }
    }

    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < STATE_DIM; j++) {
            robot->Pk[i][j] = 0.0f;
            for(int k = 0; k < STATE_DIM; k++) {
                robot->Pk[i][j] += temp[i][k] * robot->Pk_[k][j];
            }
        }
    }

    // 更新后姿态角归一化
    robot->Xk[ROLL_IDX] = atan2f(sinf(robot->Xk[ROLL_IDX]), cosf(robot->Xk[ROLL_IDX]));
    robot->Xk[PITCH_IDX] = atan2f(sinf(robot->Xk[PITCH_IDX]), cosf(robot->Xk[PITCH_IDX]));
    robot->Xk[YAW_IDX] = atan2f(sinf(robot->Xk[YAW_IDX]), cosf(robot->Xk[YAW_IDX]));

    // 更新输出状态
    robot->velocity = sqrtf(robot->Xk[VX_IDX] * robot->Xk[VX_IDX] +
                          robot->Xk[VY_IDX] * robot->Xk[VY_IDX]);
    robot->roll = robot->Xk[ROLL_IDX];
    robot->pitch = robot->Xk[PITCH_IDX];
    robot->yaw = robot->Xk[YAW_IDX];
    robot->roll_rate = robot->Xk[ROLL_RATE_IDX];
    robot->pitch_rate = robot->Xk[PITCH_RATE_IDX];
    robot->yaw_rate = robot->Xk[YAW_RATE_IDX];

    // 计算角加速度
    robot->roll_acc = (robot->roll_rate - robot->prev_roll_rate) / robot->T;
    robot->pitch_acc = (robot->pitch_rate - robot->prev_pitch_rate) / robot->T;
    robot->yaw_acc = (robot->yaw_rate - robot->prev_yaw_rate) / robot->T;

    robot->prev_roll_rate = robot->roll_rate;
    robot->prev_pitch_rate = robot->pitch_rate;
    robot->prev_yaw_rate = robot->yaw_rate;

    // 判断轮子是否离地
    float vertical_acc = fabsf(sqrtf(robot->ax * robot->ax + robot->ay * robot->ay + robot->az * robot->az) - GRAVITY);
    float angular_motion = sqrtf(robot->gx * robot->gx + robot->gy * robot->gy + robot->gz * robot->gz);

    robot->wheel_lifted = (vertical_acc > ACC_THRESHOLD) || (angular_motion > GYRO_THRESHOLD) ? 1 : 0;
//    printf("%f,%f,%f\n",robot -> roll_acc,robot -> pitch_acc,robot -> yaw_acc);
//    printf("%f,%f,%f\n",robot -> roll_rate,robot -> pitch_rate,robot -> yaw_rate);
    printf("%f,%f,%f,%f,%d\n", robot->roll, robot->pitch, robot->yaw, robot->velocity, robot->wheel_lifted);
}

