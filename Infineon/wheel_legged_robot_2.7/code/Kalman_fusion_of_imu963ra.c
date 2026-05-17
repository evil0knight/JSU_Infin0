/*
 * Kalman_fusion_of_imu660ra.c
 *
 *  Created on: 2024年11月22日
 *      Author: 17104
 */

#include<Kalman_fusion_of_imu963ra.h>
#include <Base.h>

//imu963ra结构体声明示例：
imu963ra_struct imu;

//imu963ra结构体初始化
//示例：imu963ra_kalman_filter_init(&imu, 1, 1, 0.002f);//结构体地址，系统噪声协方差，测量噪声协方差，离散时间
void imu963ra_kalman_filter_init(imu963ra_struct * imu, float q, float r, float T)
{
    imu660ra_init();

    imu -> roll = 0;//欧拉角
    imu -> pitch = 0;
    imu -> yaw = 0;

    imu -> gx = 0;//角速度
    imu -> gy = 0;
    imu -> gz = 0;
    imu -> ax = 0;//加速度
    imu -> ay = 0;
    imu -> az = 0;
    imu -> mx = 0;///磁力
    imu -> my = 0;
    imu -> mz = 0;

    int i;
    for (i = 0; i < 3;i++)
    {
        imu -> Xk_[i] = 0;//先验估计
        imu -> Xk[i] = 0;//后验估计
        imu -> Uk[i] = 0;//系统输入
        imu -> Zk[i] = 0;//测量状态
        imu -> Pk[i] = 1;//后验估计误差协方差
        imu -> Pk_[i] = 0;//先验估计误差协方差
        imu -> K[i] = 0;//卡尔曼增益
        imu -> Q[i] = q;//系统噪声协方差
        imu -> R[i] = r;//测量噪声协方差
    }

    imu -> T = T;//离散时间
    imu -> resultant_acceleration = 0;
}

//imu963ra卡尔曼融合滤波更新，六轴（陀螺仪加速度计）
//示例：imu963ra_kalman_filter_update(&imu);
void imu963ra_kalman_filter_update(imu963ra_struct * imu)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();
//    printf("%f,%f,%f\n",imu -> gx,imu -> gy,imu -> gz);
//    printf("%f,%f,%f\n",imu -> ax,imu -> ay,imu -> az);
//    printf("%f,%f,%f\n",imu -> roll,imu -> pitch,imu -> yaw);
    imu -> gx = imu660ra_gyro_transition(imu660ra_gyro_x) * My_PI / 180.f;//角速度，单位为 rad/s
    imu -> gy = imu660ra_gyro_transition(imu660ra_gyro_y) * My_PI / 180.f;
    imu -> gz = imu660ra_gyro_transition(imu660ra_gyro_z) * My_PI / 180.f;
    imu -> ax = imu660ra_acc_transition(imu660ra_acc_x);//加速度，单位为 g(m/s^2)
    imu -> ay = imu660ra_acc_transition(imu660ra_acc_y);
    imu -> az = imu660ra_acc_transition(imu660ra_acc_z);

    if(fabsf(imu -> gx) > MAX_READ_VALUE || fabsf(imu -> gy) > MAX_READ_VALUE || fabsf(imu -> gz) > MAX_READ_VALUE || fabsf(imu -> ax) > MAX_READ_VALUE || fabsf(imu -> ay) > MAX_READ_VALUE || fabsf(imu -> az) > MAX_READ_VALUE)
    {return;}

    imu -> resultant_acceleration = imu -> ax * imu -> ax + imu -> ay * imu -> ay + imu -> az * imu -> az;
    if(imu -> resultant_acceleration > 0)
    {
        imu -> resultant_acceleration = sqrtf(imu -> resultant_acceleration);
    }
    else
    {
        imu -> resultant_acceleration = 0;
        return;
    }

    imu -> Uk[0] = imu -> gx + sin(imu -> Xk[0]) * tan(imu -> Xk[1]) * imu -> gy + cos(imu -> Xk[0]) * tan(imu -> Xk[1]) * imu -> gz;
    imu -> Uk[1] = cos(imu -> Xk[0]) * imu -> gy - sin(imu -> Xk[0]) * imu -> gz;
    imu -> Uk[2] = sin(imu -> Xk[0]) * imu -> gy / cos(imu -> Xk[1]) + cos(imu -> Xk[0]) * imu -> gz / cos(imu -> Xk[1]);
    //step1 - system input

    imu -> Xk_[0] = imu -> Xk[0] + imu -> T * imu -> Uk[0];
    imu -> Xk_[1] = imu -> Xk[1] + imu -> T * imu -> Uk[1];
    imu -> Xk_[2] = imu -> Xk[2] + imu -> T * imu -> Uk[2];
    //step2 - Prior estimation

    imu -> Pk_[0] = imu -> Pk[0] + imu -> Q[0];
    imu -> Pk_[1] = imu -> Pk[1] + imu -> Q[1];
    imu -> Pk_[2] = imu -> Pk[2] + imu -> Q[2];
    //step3 - Prior estimation error covariance

    imu -> K[0] = imu -> Pk_[0] / (imu -> Pk_[0] + imu -> R[0]);
    imu -> K[1] = imu -> Pk_[1] / (imu -> Pk_[1] + imu -> R[1]);
    imu -> K[2] = 0;
    //step4 - kalman gain

    imu -> Zk[0] = atan( imu -> ay / imu -> az );
    imu -> Zk[1] = -atan( imu -> ax / (sqrt( imu -> ay * imu -> ay + imu -> az * imu -> az )));
    imu -> Zk[2] = 0;
    //step5 - measure data

    imu -> Xk[0] = (1 - imu -> K[0]) * imu -> Xk_[0] + imu -> K[0] * imu -> Zk[0];
    imu -> Xk[1] = (1 - imu -> K[1]) * imu -> Xk_[1] + imu -> K[1] * imu -> Zk[1];
    imu -> Xk[2] = imu -> Xk_[2];// + pit_time0_ms * 0.00001052f;
    //step6 - Posterior estimation

    imu -> Pk[0] = (1 - imu -> K[0]) * imu -> Pk_[0];
    imu -> Pk[1] = (1 - imu -> K[1]) * imu -> Pk_[1];
    imu -> Pk[2] = imu -> Pk_[2];
    //step7 - Posterior estimation error covariance

    imu -> roll = imu -> Xk[0] / My_PI * 180.f;
    imu -> pitch = imu -> Xk[1] / My_PI * 180.f;
    imu -> yaw = imu -> Xk[2] / My_PI * 180.f;
    //calculate the angle,unit: degree

    // 新增重力补偿计算 ----------------------------------
//    const float GRAVITY = 9.80665f; // 标准重力加速度

    // 从卡尔曼滤波结果中获取当前姿态角（弧度）
    float roll = imu->Xk[0];
    float pitch = imu->Xk[1];

    // 计算三角函数值
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    // 计算重力在机体坐标系的分量（m/s^2）
    float gx_body = -GRAVITY * sin_pitch;
    float gy_body = GRAVITY * sin_roll * cos_pitch;
    float gz_body = GRAVITY * cos_roll * cos_pitch;

    // 加速度计原始数据转换（g转m/s^2）
    float ax_mps2 = imu->ax * GRAVITY;
    float ay_mps2 = imu->ay * GRAVITY;
    float az_mps2 = imu->az * GRAVITY;

    // 计算线性加速度
    imu->ax_linear = ax_mps2 - gx_body;
    imu->ay_linear = ay_mps2 - gy_body;
    imu->az_linear = az_mps2 - gz_body;
//    printf("%f,%f,%f\n",imu->ax_linear,imu->ay_linear,imu->az_linear);

}

KF_Velocity vel_kf;

// 滤波器初始化
//Q矩阵：反映加速度计的噪声特性，值越大表示对测量值的信任度越低。
//R值：反映磁编码器的测量噪声，值越大表示对预测值的信任度越高。
//初始协方差：根据系统初始不确定性调整，通常设为适度较大的值。
void imu963ra_menc15a_kalman_filter_init(KF_Velocity* kf, float q_pos, float q_vel, float r, float T) {

    kf->Xk_[0] = kf->Xk_[1] = 0;
    kf->Xk[0] = kf->Xk[1] = 0;

    // 初始化协方差矩阵
    kf->Pk[0][0] = 1.0f; kf->Pk[0][1] = 0;
    kf->Pk[1][0] = 0;     kf->Pk[1][1] = 1.0f;

    // 过程噪声矩阵
    kf->Q[0][0] = q_pos; kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;      kf->Q[1][1] = q_vel;

    kf->R = r;
    kf->T = T;

    kf->est_displacement = 0;
    kf->est_velocity = 0;

    kf->original_R = r;
    kf->original_Q[0][0] = q_pos;
    kf->original_Q[1][1] = q_vel;
    kf->direction_factor = 0;
    kf->slip_factor = 0;
    kf->slip_ratio = 0;
}

// 更新步骤（使用磁编码器速度）
void imu963ra_menc15a_kalman_filter_Update(KF_Velocity* kf, float measured_speed, float linear_accel)
{
    // 在重力补偿前增加有效性检查
//    if(fabs(imu.resultant_acceleration) < 0.1f) {
//        // 加速度计数据异常，使用纯轮速估计
//        kf->Xk[1] = measured_speed;
//    }
//    if(/*fabs(linear_accel) > 1.0f && */fabs(measured_speed - kf->Xk_[1]) > 0.2f)
//    {
//        kf->R = kf->original_R * 10000.0f;
//        kf->Q[0][0] = kf->original_Q[0][0] * 10000.0f;
//        kf->Q[1][1] = kf->original_Q[1][1];
//    }

    handle_slip_condition(kf, measured_speed, imu.gz, imu.ax_linear);
// 使用去除重力后的线性加速度进行预测
    const float B[2] = {0.5f * kf->T * kf->T, kf->T};

    // ... 其余协方差预测保持不变 ...
    // 状态转移矩阵A和输入矩阵B
    const float A[2][2] = {{1, kf->T},
                          {0, 1}};
//    // 状态预测
    kf->Xk_[0] = A[0][0]*kf->Xk[0] + A[0][1]*kf->Xk[1] + B[0]*linear_accel;
    kf->Xk_[1] = A[1][0]*kf->Xk[0] + A[1][1]*kf->Xk[1] + B[1]*linear_accel;
//
    // 协方差预测（正确矩阵乘法实现）
    float AP[2][2] = {
        {A[0][0]*kf->Pk[0][0] + A[0][1]*kf->Pk[1][0],
         A[0][0]*kf->Pk[0][1] + A[0][1]*kf->Pk[1][1]},
        {A[1][0]*kf->Pk[0][0] + A[1][1]*kf->Pk[1][0],
         A[1][0]*kf->Pk[0][1] + A[1][1]*kf->Pk[1][1]}
    };

    // 矩阵转置乘法
    kf->Pk_[0][0] = AP[0][0]*A[0][0] + AP[0][1]*A[0][1] + kf->Q[0][0];
    kf->Pk_[0][1] = AP[0][0]*A[1][0] + AP[0][1]*A[1][1] + kf->Q[0][1];
    kf->Pk_[1][0] = AP[1][0]*A[0][0] + AP[1][1]*A[0][1] + kf->Q[1][0];
    kf->Pk_[1][1] = AP[1][0]*A[1][0] + AP[1][1]*A[1][1] + kf->Q[1][1];

    // 计算卡尔曼增益
    const float S = kf->Pk_[1][1] + kf->R;
    kf->K[0] = kf->Pk_[0][1] / S;
    kf->K[1] = kf->Pk_[1][1] / S;

    // 状态更新
    const float y = measured_speed - kf->Xk_[1];
    kf->Xk[0] += kf->K[0] * y;
    kf->Xk[1] += kf->K[1] * y;

    // 更新协方差矩阵
    const float P00 = kf->Pk_[0][0] - kf->K[0] * kf->Pk_[1][0];
    const float P01 = kf->Pk_[0][1] - kf->K[0] * kf->Pk_[1][1];
    const float P11 = kf->Pk_[1][1] - kf->K[1] * kf->Pk_[1][1];

    kf->Pk[0][0] = P00;
    kf->Pk[0][1] = P01;
    kf->Pk[1][0] = kf->Pk_[1][0] - kf->K[1] * kf->Pk_[1][0];
    kf->Pk[1][1] = P11;

    kf->est_displacement += kf->Xk[0];
    kf->est_velocity = kf->Xk[1];

    // 恢复默认R值
    kf->R = kf->original_R;
    kf->Q[0][0] = kf->original_Q[0][0];
    kf->Q[1][1] = kf->original_Q[1][1];

    // 状态约束（防止发散）
    kf->Xk[1] = fmaxf(fminf(kf->Xk[1], MAX_SPEED), -MAX_SPEED);

    //    printf("%f,%f\n",kf->est_displacement,kf->est_velocity);
//    printf("%f,%f,%f,%d\n",Instantaneous_speed,kf->est_velocity,kf->slip_ratio,kf->is_slip);
}

// 改进的打滑处理函数
void handle_slip_condition(KF_Velocity* kf, float measured_speed, float yaw_rate, float lat_accel)
{
    // 计算速度差异
    float speed_diff = fabsf(measured_speed - kf->Xk_[1]);
////    printf("%f\n",speed_diff);
    if(fabsf(standardized_curvature_ave) > 0.6) {
        // 根据向心加速度修正速度估计
        float radius = fabs(kf->Xk[1] / (yaw_rate + 0.001f));
        float centripetal_accel = kf->Xk[1] * yaw_rate;
        float accel_bias = centripetal_accel - lat_accel;

        // 注入补偿量（限幅处理）
        accel_bias = fmaxf(fminf(accel_bias, 2.0f), -2.0f);
        imu.ax_linear -= 0.4f * accel_bias;
    }

    // 滑移率计算
    // 方向一致性检测 -------------------------------------------------
    if (measured_speed * kf->Xk_[1] >= 0) {
        // 同向运动
        kf->direction_factor = 1.0f;
    } else {
        // 异向时计算过渡因子
        float reverse_ratio = fminf(speed_diff / SPEED_REVERSE_THRESH, 1.0f);
        kf->direction_factor = cosf(My_PI_2 * reverse_ratio);
    }
    // 修正基准速度计算 -----------------------------------------------
    float base_speed = fmaxf(fabsf(measured_speed), fabsf(kf->Xk_[1]));
    if (base_speed < 0.2f) {
        kf->slip_ratio = 0;
    } else {
        // 引入方向因子修正
        float valid_speed_diff = speed_diff * kf->direction_factor;
        kf->slip_ratio = valid_speed_diff / base_speed;
    }

    //打滑判断

    if(/*fabs(linear_accel) > 1.0f && */speed_diff > 0.3f)
    {
//        kf->R = kf->original_R * 10000.0f;
//        kf->Q[0][0] = kf->original_Q[0][0] * 10000.0f;
//        kf->Q[1][1] = kf->original_Q[1][1];
        kf->is_slip = 1;
    }
    else
    {
        if (!validate_acceleration(imu.ay_linear, kf->Xk_[1], measured_speed, kf->T))
        {
            kf->slip_ratio = 0; // 加速度不匹配时清零
        }
        kf->is_slip = 0;
    }

//    // 弯道检测
//    int is_curving = (fabsf(yaw_rate) > CURVE_THRESHOLD_YAW) &&
//                    (fabsf(lat_accel) > CURVE_THRESHOLD_ACC);
//    // 动态调节系数
//    float curve_factor = is_curving ? 1.5f : 1.0f;// 弯道时更敏感
    float curve_factor = fabsf(standardized_curvature_ave) > 0.4 ? 1.5f : 1.0f;// 弯道时更敏感
    // 动态调整参数（指数平滑）
    kf->slip_factor = 0.8f * kf->slip_factor + 0.2f * kf->slip_ratio;
    // 参数更新
    kf->R = kf->original_R * (1.0f + 10000.0f * kf->slip_factor) * curve_factor;
    kf->Q[0][0] = kf->original_Q[0][0] * (1.0f + 10000.0f * kf->slip_factor) * curve_factor;

    // 约束最大值
    kf->R = fminf(kf->R, kf->original_R * 10000.0f);
    kf->Q[0][0] = fminf(kf->Q[0][0], kf->original_Q[0][0] * 10000.0f);

    // 弯道特殊处理
//    if(is_curving) {
//        kf->Q[1][1] = kf->original_Q[1][1] * 2.0f; // 提高速度过程噪声
//    }
}
int validate_acceleration(float imu_accel, float speed_est, float measured_speed, float dt)
{
    float model_accel = (measured_speed - speed_est) / dt;
    return (fabsf(imu_accel - model_accel) < ACC_THRESH);
}
