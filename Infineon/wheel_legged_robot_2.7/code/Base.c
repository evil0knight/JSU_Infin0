/*
 * Base.c
 *
 *  Created on: 2024年11月24日
 *      Author: 17104
 */

#include <Base.h>

float Uq[2] = { 0 };
float speed[2] = { 0 };
float prev_pitch = 0.0f;
uint8 tx_buffer[16] = { 0 };

float    KL[7]={0};
float    KR[7]={0};
float    Ka[7]={0};
float KlegL[7]={0};
float KlegR[7]={0};

Matrix COMMON_Q;
Matrix COMMON_R;
Matrix COMMON_K;
Matrix STABLE_K;
Matrix HIGH_K;

float I_pitch = 0.0f;
float I_I_pitch = 0.0f;

float prev_horizontal_E = 0.0f;

extern imu963ra_struct imu;

steering_engine_struct steer_LF;
steering_engine_struct steer_LR;
steering_engine_struct steer_RF;
steering_engine_struct steer_RR;

KalmanFilter Lx;
KalmanFilter Ly;
KalmanFilter Rx;
KalmanFilter Ry;

KalmanFilter LM;
KalmanFilter RM;

//KalmanFilter legL;
//KalmanFilter legR;

//place_PID_struct leg;
increment_PID_struct leg;

Base_input_data input;

wheel_coordinate_struct wheel_L;
wheel_coordinate_struct wheel_R;

KalmanFilter gx;
KalmanFilter gy;
KalmanFilter gz;

KalmanFilter bat;

foc_struct motor_L;
foc_struct motor_R;

float Instantaneous_speed = 0;
float actual_max_speed = 0.0f;
float displacement = 0.0f;
float torque_compensation = 0.0f;

int start_flag = 0;
int take_off_flag = 0;
int32 speed_count = 2;
int timer = 0;
float eva_speed = 0.5f;
float max_speed = 0.5f;
float delta_y = 0.0f;
float prev_delta_y = 0.0f;
float alpha = 0.0f;

debounce_filter_struct slope_status_filter;
int slope_status = 0;
float leg_increase = 0;

float prev_gx = 0.0f;
float acc_gx = 0.0f;
float prev_gy = 0.0f;
float acc_gy = 0.0f;
float begin_yaw = -10.0f;
float count_yaw = 0.0f;
float E_yaw = 0.0f;
float now_gx = 0.0f;
float now_gy = 0.0f;
float now_gz = 0.0f;

//float resultant_acceleration = 0.0f;

void Base_init(float Car_zero, float max_speed)
{
    /*舵机初始化*/
    wheel_coordinate_struct_init(&wheel_L, Car_zero, 45.0f);
    wheel_coordinate_struct_init(&wheel_R, Car_zero, 45.0f);
    steering_engine_init(&steer_LF,ATOM2_CH6_P11_11,200,90.0,30.0,180.0,-4.0);
    steering_engine_init(&steer_LR,ATOM2_CH4_P11_9,200,90.0,0.0,150.0,7.0);
    steering_engine_init(&steer_RF,ATOM2_CH7_P11_12,200,90.0,0.0,150.0,2.5);
    steering_engine_init(&steer_RR,ATOM2_CH5_P11_10,200,90.0,30.0,180.0,-6.0);
    kalman_init(&Lx, 10, 0.1, 1, 250);
    kalman_init(&Ly, 45, 0.1, 1, 200);
    kalman_init(&Rx, 10, 0.1, 1, 250);
    kalman_init(&Ry, 45, 0.1, 1, 200);
    kalman_init(&LM, 0, 0.1, 1, 7);
    kalman_init(&RM, 0, 0.1, 1, 7);

    kalman_init(&gx, 0, 1, 2, 1);
    kalman_init(&gy, 0, 1, 1, 10);
    kalman_init(&gz, 0, 1, 2, 1);

    debounce_filter_init(&slope_status_filter, 2);

    foc_init(&motor_L,ATOM3_CH2_P33_6,ATOM3_CH3_P33_7,ATOM3_CH4_P33_8,menc15a_1_module,corotation,12.0,6.0,7,pit_time3_us);
    foc_init(&motor_R,ATOM1_CH1_P33_9,ATOM3_CH0_P33_10,ATOM1_CH2_P33_11,menc15a_2_module,reversal,12.0,6.0,7,pit_time3_us);
//    kalman_init(&legL, 45.0f, 1, 1, 30);
//    kalman_init(&legR, 45.0f, 1, 1, 30);
//    place_pid_init(&leg, -10.0f, 2.0f, -10.0f, 0.0f, 30.0f, 30.0f);
    increment_pid_init(&leg, 5.0f, 0.0f, 0.5f, 0.0f, 30.0f);//kp,ki,kd,target,limit
    /*底盘结构体初始化*/
    adc_init(ADC1_CH0_A8, ADC_10BIT);
    kalman_init(&bat, (float)adc_mean_filter_convert (ADC1_CH0_A8,100), 1, 1, 100000);
    input.batter_voltage = kalman_update(&bat, (float)adc_convert (ADC1_CH0_A8)) / 75 - 1.4333f;
    input.Car_zero = Car_zero;
    input.max_speed = max_speed;
    input.flag = 0;
    input.prev_flag = 0;
    input.horizontal_E = 0;
    input.track = 0;
    input.actual_target_speed = 0;
    uint32 Ts = system_getval_ms();
    float Q_data[7] ={ 0 };
    int j;
    for(j = 0; j < 7; j++)
    {
        Q_data[j] = get_float_by_id(41 + j);
//        printf("%f\n",Q_data[j]);

    }
    float R_data[5] = { 0 };
    for(j = 0; j < 5; j++)
    {
        R_data[j] = get_float_by_id(51 + j);
//        printf("%f\n",R_data[j]);

    }
    LQR_AB_init(&A, &B);
    LQR_QR_init(&COMMON_Q, &Q_data[0], &COMMON_R, &R_data[0]);
    int count = LQR_K_update(&COMMON_Q, &COMMON_R, &COMMON_K);
    get_controller_gains(&COMMON_K, KL, KR, Ka);
    for(j = 0; j < dir_num; j++)//遍历目录结构体数组，更新被选中的结构体的选择状态
    {
        if(strcmp(all_dir[j].name, "update_LQR") == 0)
        {
            all_dir[j].int_data = count;
            break;
        }
    }
    printf("time: %lu ms; count : %d\n", system_getval_ms() - Ts,count);

    Ts = system_getval_ms();


    Q_data[1] *= 1.0f;  //v
    Q_data[2] *= 1.5f;  //roll
    Q_data[3] *= 1.0f;  //gx
    Q_data[4] *= 1.0f;  //c
    Q_data[5] *= 1.8f;  //gz

    R_data[0] *= 1.0f;   //TL
    R_data[1] *= 1.0f;   //TR
//    R_data[2] *= 0.5;   //a
    LQR_QR_init(&COMMON_Q, &Q_data[0], &COMMON_R, &R_data[0]);
    count = LQR_K_update(&COMMON_Q, &COMMON_R, &STABLE_K);
    printf("time: %lu ms; count : %d\n", system_getval_ms() - Ts,count);


    Ts = system_getval_ms();
    for(j = 0; j < 7; j++)
    {
        Q_data[j] = get_float_by_id(41 + j);
//        printf("%f\n",Q_data[j]);
    }
    for(j = 0; j < 5; j++)
    {
        R_data[j] = get_float_by_id(51 + j);
//        printf("%f\n",R_data[j]);
    }
    Q_data[1] *= 1.0f;  //v
    Q_data[2] *= 1.0f;  //roll
    Q_data[3] *= 1.5f;  //gx
    Q_data[4] *= 1.0f;  //c
    Q_data[5] *= 1.0f;  //gz

    R_data[0] *= 1.0f;   //TL
    R_data[1] *= 1.0f;   //TR
    LQR_AB_init_high(&A, &B);
    LQR_QR_init(&COMMON_Q, &Q_data[0], &COMMON_R, &R_data[0]);
    count = LQR_K_update(&COMMON_Q, &COMMON_R, &HIGH_K);
    printf("time: %lu ms; count : %d\n", system_getval_ms() - Ts,count);
    matrix_show(&HIGH_K);
    matrix_show(&COMMON_K);
//    get_controller_gains(&STABLE_K, KL, KR, Ka);


//    int i;
//    for(i = 0; i < 7; i++)
//    {
//        printf("%.10f,%.10f,%.10f\n",KL[i],KR[i],Ka[i]);
//    }
}
//input.horizontal_E
void Base_run(void)
{

    if(Instantaneous_speed < 0.6f)// 低速时参数
    {
        kalman_change(&LM, 1, 7);
        kalman_change(&RM, 1, 7);
    }
    else// 高速时参数
    {
        kalman_change(&LM, 1, 15);
        kalman_change(&RM, 1, 15);
    }

    if((take_off_flag || vel_kf.is_slip) && input.flag != 5)//增益自适应调整逻辑，主要功能是根据车辆状态动态调整LQR控制器的增益参数
    {
        get_controller_gains(&STABLE_K, KL, KR, Ka);
        get_adaptive_controller_gains(&COMMON_K, &STABLE_K,  KL,  KR,  Ka, vel_kf.slip_ratio);
        Instantaneous_speed = vel_kf.est_velocity;
        input.actual_target_speed = Instantaneous_speed;
        KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2])*1.2f;
        KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2])*1.2f;
        KL[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KL[1]))*1.0f;
        KR[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KR[1]))*1.0f;
//        Ka[2] = sigmoid_adaptive_q_weight(imu.roll, Ka[2]);
    }
    else
    {
        get_controller_gains(&COMMON_K, KL, KR, Ka);
        KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
        KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);
        KL[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KL[1]))*1.0f;
        KR[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KR[1]))*1.0f;
    }

    if(fabsf(imu.roll) > 15.0f)//车辆大角度侧倾时的紧急稳定控制策略
    {
        get_controller_gains(&STABLE_K, KL, KR, Ka);
        KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2])*1.2f;
        KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2])*1.2f;
        KL[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KL[1]))*1.0f;
        KR[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KR[1]))*1.0f;

//        Ka[2] = sigmoid_adaptive_q_weight(imu.roll, Ka[2]);
    }

//电池电压检测加保护
    input.batter_voltage = kalman_update(&bat, (float)adc_convert (ADC1_CH0_A8)) * 0.0134883720930233f - 1.62883720930237f;//12.34-998,11.93-967//12.0-1012,11.9-1003,11.32-960
    motor_L.batter_voltage = input.batter_voltage;
    motor_R.batter_voltage = input.batter_voltage;
    if(input.batter_voltage < 11.4 && (input.flag ==1 || input.flag == 4) && !take_off_flag)
    {
        input.flag = 0;
        beel = 3000;
    }

//    printf("%f,%d\n",input.batter_voltage,adc_convert (ADC1_CH0_A8));

    now_gx = kalman_update(&gx, imu.gx);
    now_gy = kalman_update(&gy, imu.gy);
    now_gz = kalman_update(&gz, imu.gz);


    acc_gx = (now_gx - prev_gx) / pit_time0_ms * 1000.0f / 180.0f * _PI;
    prev_gx = now_gx;
    acc_gy = (now_gy - prev_gy) / pit_time0_ms * 1000.0f / 180.0f * _PI;
    prev_gy = now_gy;

//    d_gx = (now_gx - prev_gx) / pit_time0_ms * 1000.0f / 180.0f * _PI;
//    prev_gx = now_gx;
//    d_gy = (now_gy - prev_gy) / pit_time0_ms * 1000.0f / 180.0f * _PI;
//    prev_gy = now_gy;
//    printf("%f,%f\n",d_gx,d_gy);
//    if(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az > 0){resultant_acceleration = sqrtf(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az);}
//    else{resultant_acceleration = 0;}

//    speed[0] = foc_get_speed(&motor_L);
//    speed[1] = foc_get_speed(&motor_R);
////    printf("%f,%f\n",speed[0],speed[1]);
//    printf("%f,%f,%f\n",speed[0],speed[1],Instantaneous_speed);
//    Instantaneous_speed = ((speed[0] - speed[1]) / 2.0f + now_gx) * 68.0f / 2.0f / 1000.0f;
//    speed[0] = 0;
//    speed[1] = 0;
//    displacement += Instantaneous_speed * pit_time0_ms / 1000.0f;

    if(input.track){input.horizontal_E = pure_angle * 2;}
    else{input.horizontal_E = 0;}

//    if(barrier_type == BARRIER_BRIDGE_IN && input.flag == 1)
//    {
//        timer = 0;
//        actual_max_speed = 0.8f;
//        input.flag = 5;
//        I_pitch = 0.0f;
//        leg_increase = 0;
//        begin_yaw = fmodf(imu.yaw / 180.0f * _PI + pure_angle , _2PI);
//        begin_yaw = begin_yaw > _PI ? begin_yaw - _2PI : begin_yaw;
//        begin_yaw = begin_yaw < -_PI ? begin_yaw + _2PI : begin_yaw;
//        kalman_change(&Ly, 1, 50);
//        kalman_change(&Ry, 1, 50);
//        kalman_change(&LM, 1, 20);
//        kalman_change(&RM, 1, 20);
//    }
//    else if(barrier_type == BARRIER_NONE && input.flag == 5 && timer > 500)
//    {
//        input.flag = 1;
//        begin_yaw = -10;
//        timer = 0;
//        kalman_change(&Ly, 1, 200);
//        kalman_change(&Ry, 1, 200);
//    }

//    if(input.flag)
//    {
//        input.flag = 5;
////
//        kalman_change(&Ly, 1, 50);
//        kalman_change(&Ry, 1, 50);
//        kalman_change(&LM, 1, 20);
//        kalman_change(&RM, 1, 20);
//
//    }
//    弯道减速、压弯
    if(fabsf(standardized_curvature_ave) > 0.4 && Instantaneous_speed > 0.3f)//当车辆以一定速度进入足够弯的弯道时，启用弯道控制策略
    {
        actual_max_speed = sqrtf(steering_acceleration / fabsf(standardized_curvature_ave));//计算弯道最大安全速度
        actual_max_speed = actual_max_speed > input.max_speed ? input.max_speed : actual_max_speed;
//        float h = (prev_delta_y / 2 + 80.0f)/1000.0f;
        if(!vel_kf.is_slip && !take_off_flag && input.flag != 5)//弯道横向偏移量计算（预瞄控制）
        {
            float h = centroid_distance/1000.0f;//重心高度
            // 复杂动力学公式计算横向偏移
            delta_y = 1.0f * 160.0f * tanf(asinf((Instantaneous_speed*Instantaneous_speed*fabsf(standardized_curvature_ave)-Instantaneous_speed*fabsf(now_gz)-h*fabsf(acc_gx)*cosf(imu.roll / 180.0f * _PI))/(GRAVITY-fabsf(h*acc_gy))));
//            delta_y = 0.2f*Instantaneous_speed * Instantaneous_speed * fabsf(standardized_curvature_ave) / GRAVITY * 160.0f / 2.0f;
        }
        else{delta_y = 0.0f;}// 特殊工况不进行偏移
        //        torque_compensation = fabsf(1000*h*now_gz*sinf(imu.pitch/ 180.0f * _PI)/Instantaneous_speed);

//        delta_y = Instantaneous_speed * Instantaneous_speed * fabsf(standardized_curvature_ave) / 9.7997f * 160.0f / 2.0f;
//        _constrain(delta_y, (prev_delta_y - 30.0f) * pit_time0_ms / 1000.0f, (prev_delta_y + 30.0f) * pit_time0_ms / 1000.0f);

        delta_y = _constrain(delta_y, 0, 10.0f);
        prev_delta_y = delta_y;
    }
    else//直道处理逻辑
    {
        actual_max_speed = input.max_speed;
        delta_y = 0.0f;
        torque_compensation = 0.0f;
    }
//    加速度限制
//    if(input.actual_target_speed < actual_max_speed && imu.roll <= 15.0f && imu.roll >= -15.0f)// && imu.gx <= 0.3f )//&& fabsf(Instantaneous_speed - actual_max_speed) < acceleration_acceleration )
//    {
//        input.actual_target_speed += acceleration_acceleration * pit_time0_ms / 1000.0f * ((1.0 - fabsf(standardized_curvature_ave)) / 1.0);
//        if(input.actual_target_speed > actual_max_speed)
//        {
//            input.actual_target_speed = actual_max_speed;
//        }
//    }
//    else if(input.actual_target_speed > actual_max_speed  && imu.roll >= -20.0f)
//    {
//        input.actual_target_speed -= braking_acceleration * pit_time0_ms / 1000.0f;
//        if(input.actual_target_speed < actual_max_speed)
//        {
//            input.actual_target_speed = actual_max_speed;
//        }
//    }

    if(input.actual_target_speed < actual_max_speed && imu.roll <= 10.0f && imu.roll >= -10.0f && imu.gx <= 0.3f && fabsf(Instantaneous_speed - actual_max_speed) < acceleration_acceleration )
    {//加速条件判断
        //智能加速控制，弯道越急，加速度越小
        input.actual_target_speed = input.actual_target_speed + (float)acceleration_acceleration * (float)pit_time0_ms / 1000.0f * ((4.0 - fabsf(standardized_curvature_ave)) / 4.0);
        if(input.actual_target_speed > actual_max_speed)
        {
            input.actual_target_speed = actual_max_speed;//确保加速后不超过安全限速
        }
    }
    else if(input.actual_target_speed > actual_max_speed)//  && imu.roll >= -20.0f)
    {// 减速控制
        input.actual_target_speed = input.actual_target_speed - (float)braking_acceleration * (float)pit_time0_ms / 1000.0f;
        if(input.actual_target_speed < actual_max_speed)
        {
            input.actual_target_speed = actual_max_speed;
        }
    }
//    启动速度锁
//    if(Instantaneous_speed >= 0.85f) start_flag = 1;
    if(start_flag == 0)//车辆起步速度保护机制
    {
        input.actual_target_speed = (input.actual_target_speed < 0.6f ? 0.6f : input.actual_target_speed);
    }
//    else
//    {
//        input.actual_target_speed = (input.actual_target_speed < 0.8f ? 0.8f : input.actual_target_speed);
//    }

    //倾倒保护
    if(imu.roll > 60.0f || imu.roll < -70.0f || imu.pitch > 70.0f || imu.pitch < -70.0f)
    {
        input.flag = 0;
    }
    //测平均速率
    if(Instantaneous_speed >= 0.5f)
    {
        speed_count ++;
        eva_speed = eva_speed * (speed_count - 1) / speed_count + Instantaneous_speed / speed_count;
        if(Instantaneous_speed > max_speed)
        {
            max_speed = Instantaneous_speed;
        }
    }
//    printf("%.3f\n",eva_speed);

    if(take_off_flag)
    {
        take_off_flag +=pit_time0_ms;
        if(take_off_flag > 1000){take_off_flag = 0;}
    }

    //正常行驶识别到跳台还有0.16s时起跳
    if(transection_distance / Instantaneous_speed <= (input.batter_voltage > 12.0f ? 0.18f : 0.19f) && transection_distance != -1 && (input.flag == 1 || input.flag == 4) && take_off_flag == 0)
    {
        take_off_flag +=pit_time0_ms;
        timer = 0;
        input.flag = 2;
    }

    if(transection_distance  > 0.4 && transection_distance != -1 && input.flag == 1 && take_off_flag == 0)
    {
        if(count_yaw == 0.0f || begin_yaw == -10.0f)
        {
            begin_yaw = 0.0f;
        }
        float road_yaw = fmodf(imu.yaw / 180.0f * _PI - pure_angle , _2PI);
        road_yaw = road_yaw > _PI ? road_yaw - _2PI : road_yaw;
        road_yaw = road_yaw < -_PI ? road_yaw + _2PI : road_yaw;
        begin_yaw += road_yaw;
        count_yaw += 1.0f;
    }
    else if(transection_distance  <= 0.4 && transection_distance != -1 && input.flag == 1 && take_off_flag == 0)
    {
        if(begin_yaw == -10.0f)
        {
            begin_yaw = fmodf(imu.yaw / 180.0f * _PI - pure_angle , _2PI);
            begin_yaw = begin_yaw > _PI ? begin_yaw - _2PI : begin_yaw;
            begin_yaw = begin_yaw < -_PI ? begin_yaw + _2PI : begin_yaw;
        }
        else{begin_yaw /= count_yaw;}

        input.flag = 4;
        count_yaw = 0.0f;
    }

//    if(input.flag == 1)
//    {
//        timer += pit_time0_ms;
//        if(timer <= 3000)
//        {
//            if(count_yaw == 0.0f)
//            {
//                begin_yaw = 0.0f;
//            }
//            begin_yaw += fmodf(imu.yaw / 180.0f * _PI - pure_angle , _PI);
//            count_yaw += 1.0f;
//        }
//        else
//        {
//            if(begin_yaw == -10.0f){begin_yaw = fmodf(imu.yaw / 180.0f * _PI - pure_angle , _PI);}
//            else{begin_yaw /= count_yaw;}
//
//            input.flag = 4;
//            count_yaw = 0.0f;
//        }
//    }

    int i;
    float legL, legR, torque_compensation_L = 0,torque_compensation_R = 0;
    float m = 49.1f / 1000.0f;         //车轮质量kg
    float M = 1314.8f / 1000.0f;       //车体质量kg
    float l = centroid_distance / 1000.0f;           //质心距离底盘中心的距离m
    float length = 170.0f / 1000.0f;   //车体长度m
    float width = 100.0f / 1000.0f;     //车体宽度m
    float height = 90.0f / 1000.0f;    //车体高度m
    float Jp_ratio = 0;
    float Jc_ratio = 0;
//    input.actual_target_speed = 0;
//    input.horizontal_E = 0;
//    input.flag = 1;
//    printf("%f,%f\n",Instantaneous_speed,now_gx);
//    printf("%d,%d,%f,%f\n",input.flag,timer,Uq[0],Uq[1]);

    switch(input.flag)
    {
        case 0: //地面停车
        {
            Uq[0] = 0;
            Uq[1] = 0;

//            speed[0] = foc_openloop_update(&motor_L,Uq[0] * -1);
//            speed[1] = foc_openloop_update(&motor_R,Uq[1]);

            timer = 0;

            break;
        }
        case 1: //地面运行
        {
//            timer += pit_time0_ms;
//            if(timer > 8000)
//            {
//                timer = 0;
//                input.flag = 2;
//            }
//            for(i = 0; i < 7; i++)
//            {
//                KL[i] = LS_STRAIGHT_K.data[0][i];
//                KR[i] = LS_STRAIGHT_K.data[1][i];
//                Ka[i] = LS_STRAIGHT_K.data[2][i];
//            }
//            get_controller_gains(&LS_STRAIGHT_K, KL, KR, Ka);
            switch(state_type)
            {
                case STRAIGHT_STATE:

                    break;
                default:

                   break;

            }

//            float speed_E = input.target_speed - Instantaneous_speed;
//            if(speed_E > 0.1f) speed_E = 0.1f;
//            else if(speed_E < -0.1f) speed_E = -0.1f;

            // x: 当前状态; x_max: 状态约束边界; margin: 安全裕度; max_scale: 最大惩罚倍数（如10.0）
//            KL[2] = linear_adaptive_q_weight(imu.roll, 45.0f, 35.0f, 5.0f, KL[2]);
//            KR[2] = linear_adaptive_q_weight(imu.roll, 45.0f, 35.0f, 5.0f, KR[2]);
//            Ka[2] = linear_adaptive_q_weight(imu.roll, 45.0f, 35.0f, 5.0f, Ka[2]);
//            KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
//            KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);

//            Ka[2] = sigmoid_adaptive_q_weight(imu.roll, Ka[2]);
            if(delta_y == 0)
            {
                legL =  45.0f;
                legR =  45.0f;
            }else{
                if(standardized_curvature_ave > 0)
                {
                    legL = 45.0f + 0.75f * delta_y;
                    legR = 45.0f - 0.75f * delta_y;
//                    for(i = 2; i < 5; i++)
//                    {
//                        KL[i] *= (delta_y / 80.0f + 1.0f)*(delta_y / 80.0f + 1.0f)/1.071f;
////                        KR[i] *= (delta_y / 160.0f + 1.0f)*(delta_y / 160.0f + 1.0f)/1.071f;
//                        Ka[i] *= (delta_y / 160.0f + 1.0f)*(delta_y / 160.0f + 1.0f)/1.071f;
//                    }
//                    KL[4] *= 0.68f;
//                    KR[4] *= 0.68f;
//                    KL[5] *= 0.5f;
//                    KR[5] *= 0.5f;
                    torque_compensation_L = torque_compensation * -1;
                    torque_compensation_R = torque_compensation;
                }
                else
                {
                    legL =  45.0f - 0.75f * delta_y;
                    legR =  45.0f + 0.75f * delta_y;
//                    for(i = 2; i < 5; i++)
//                    {
////                        KL[i] *= (delta_y / 160.0f + 1.0f)*(delta_y / 160.0f + 1.0f)/1.071f;
//                        KR[i] *= (delta_y / 80.0f + 1.0f)*(delta_y / 80.0f + 1.0f)/1.071f;
//                        Ka[i] *= (delta_y / 160.0f + 1.0f)*(delta_y / 160.0f + 1.0f)/1.071f;
//                    }
//                    KL[4] *= 0.68f;
//                    KR[4] *= 0.68f;
//                    KL[5] *= 0.5f;
//                    KR[5] *= 0.5f;
                    torque_compensation_L = torque_compensation;
                    torque_compensation_R = torque_compensation * -1;
                }
            }

//            input.actual_target_speed = 1.5;
            float roll_compensation =  imu.roll > -5.0f ? 0.267533179f * Instantaneous_speed - alpha : 0;
//            float prev_alpha = alpha;
            alpha = (Ka[1] * (input.actual_target_speed - Instantaneous_speed) + Ka[2] * imu.roll / 180.0f * _PI + Ka[3] * now_gx);
//            float alpha_vel = (alpha - prev_alpha) / pit_time0_ms * 1000.0f;
//            Uq[0] /= cosf(fabsf(alpha));
//            Uq[1] /= cosf(fabsf(alpha));
//            m = 49.1f / 1000.0f;         //车轮质量kg
//            M = 1314.8f / 1000.0f;       //车体质量kg
//            l = centroid_distance / 1000.0f;           //质心距离底盘中心的距离m
//            length = 150.0f / 1000.0f;   //车体长度m
//            height = 140.0f / 1000.0f;    //车体高度m


            Jp_ratio = ((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l/ cosf(alpha)/ cosf(alpha))/((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l);  //车体绕质心俯仰旋转的转动惯量kg*m^2
            Jc_ratio = ((M - 2 * m) * (length * length + width * width) / 12.0f + (M - 2 * m) * l * l * tanf(alpha) * tanf(alpha))/((M - 2 * m) * (length * length + width * width) / 12.0f);
            Uq[0] = -1.7f * ((KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KL[3] * now_gx) *Jp_ratio + (-KL[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KL[5] * now_gz)*Jc_ratio + torque_compensation_L);//L
            Uq[1] = -1.7f * ((KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KR[3] * now_gx) *Jp_ratio + (-KR[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KR[5] * now_gz)*Jc_ratio + torque_compensation_R);

            float torque_difference = Uq[0] - Uq[1];
            float aphla_L_ratio = 1.0f, aphla_R_ratio = 1.0f;
            if(torque_difference > 0){
                aphla_L_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.4f + 1.0f;
                aphla_R_ratio = 2.0f - aphla_L_ratio;
            }
            else{
                aphla_R_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.4f + 1.0f;
                aphla_L_ratio = 2.0f - aphla_R_ratio;
            }

            if (fabsf(torque_difference) > MAX_TORQUE_DIFFERENCE)
            {
                if(Uq[0] > Uq[1])
                {
                    Uq[0] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE) / 2.0f;
                    Uq[1] = Uq[0] - MAX_TORQUE_DIFFERENCE;
                }
                else
                {
                    Uq[1] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE) / 2.0f;
                    Uq[0] = Uq[1] - MAX_TORQUE_DIFFERENCE;
                }
            }
            if(Uq[0] >= 0){Uq[0] += expf(Uq[0]*0.15) * expf(Uq[0]*0.15) - 1;}
            else{Uq[0] -= expf(Uq[0]*-0.15) * expf(Uq[0]*-0.15) - 1;}
            if(Uq[1] >= 0){Uq[1] += expf(Uq[1]*0.15) * expf(Uq[1]*0.15) - 1;}
            else{Uq[1] -= expf(Uq[1]*-0.15) * expf(Uq[1]*-0.15) - 1;}
            if(vel_kf.is_slip)
            {
                Uq[0] *= (1.0f - 0.5f * vel_kf.slip_ratio);
                Uq[1] *= (1.0f - 0.5f * vel_kf.slip_ratio);
            }
//            printf("%f,%f\n",Uq[0],Uq[1]);


//            if(take_off_flag){alpha *=0.8;}
//            if(wheel_L.y - wheel_R.y > -60.0f && imu.pitch < 0.0f)
//            {
//                I_pitch += imu.pitch / 180.0f * _PI;
//            }
//            else if(wheel_L.y - wheel_R.y < 60.0f && imu.pitch > 0.0f)
//            {
//                I_pitch += imu.pitch / 180.0f * _PI;
//            }
//            legL = -1 * KlegL[6] * I_pitch + 50.0f;
//            legR = -1 * KlegR[6] * I_pitch + 50.0f;

//            if(delta_y == 0)
//            {
//                legL =  45.0f;
//                legR =  45.0f;
//            }else{
//                if(standardized_curvature_ave > 0)
//                {
//                    legL =  fabsf(delta_y) + 45.0f;
//                    legR =  45.0f;
//                    Uq[0] *= (delta_y / 80.0f + 1.0f);
//                    Uq[1] *= (delta_y / 80.0f + 1.0f);
//                    alpha *= (delta_y / 80.0f + 1.0f);
//                }
//                else
//                {
//                    legL =  45.0f;
//                    legR =  fabsf(delta_y) + 45.0f;
//                    Uq[0] *= (delta_y / 80.0f + 1.0f);
//                    Uq[1] *= (delta_y / 80.0f + 1.0f);
//                    alpha *= (delta_y / 80.0f + 1.0f);
//                }
//            }

            legL = _constrain(legL,30.0f,130.0f);
            legR = _constrain(legR,30.0f,130.0f);
            wheel_L.y = kalman_update(&Ly, legL);
            wheel_R.y = kalman_update(&Ry, legR);
//            wheel_L.y = legL;
//            wheel_R.y = legR;
            if(alpha < _PI_4 && alpha > -1 * _PI_4)
            {
                wheel_L.x = centroid_distance * tan(alpha * aphla_L_ratio) + input.Car_zero;
                wheel_R.x = centroid_distance * tan(alpha * aphla_R_ratio) + input.Car_zero;
                wheel_L.x = _constrain(wheel_L.x,-70.0f,70.0f);
                wheel_R.x = _constrain(wheel_R.x,-70.0f,70.0f);
                wheel_L.x = kalman_update(&Lx, wheel_L.x);
                wheel_R.x = kalman_update(&Rx, wheel_R.x);
            }
//            if(input.batter_voltage < 11.4)
//            {
//                Uq[0] = 0;
//                Uq[1] = 0;
//                input.flag = 0;
//            }
//            speed[0] = foc_openloop_update(&motor_L,Uq[0] * -1);
//            speed[1] = foc_openloop_update(&motor_R,Uq[1]);
            break;
        }
        case 2: //起跳
        {
            timer += pit_time0_ms;
//            for(i = 0; i < 7; i++)
//            {
//                KL[i] = LS_STRAIGHT_K.data[0][i];
//                KR[i] = LS_STRAIGHT_K.data[1][i];
//                Ka[i] = LS_STRAIGHT_K.data[2][i];
//            }
//            get_controller_gains(&COMMON_K, KL, KR, Ka);
//            KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
//            KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);
//            Ka[2] = sigmoid_adaptive_q_weight(imu.roll, Ka[2]);
            float roll_compensation =  0.267533179f * Instantaneous_speed - alpha;
            alpha = (Ka[1] * (input.actual_target_speed - Instantaneous_speed) + Ka[2] * imu.roll / 180.0f * _PI + Ka[3] * now_gx);

            Jp_ratio = ((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l/ cosf(alpha)/ cosf(alpha))/((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l);  //车体绕质心俯仰旋转的转动惯量kg*m^2
//            Jc_ratio = ((M - 2 * m) * (length * length + width * width) / 12.0f + (M - 2 * m) * l * l * tanf(alpha) * tanf(alpha))/((M - 2 * m) * (length * length + width * width) / 12.0f);

            Uq[0] = -1.7f * ((KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KL[3] * now_gx) *Jp_ratio);//L
            Uq[1] = -1.7f * ((KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KR[3] * now_gx) *Jp_ratio);

            if(Uq[0] >= 0){Uq[0] += expf(Uq[0]*0.15) * expf(Uq[0]*0.15) - 1;}
            else{Uq[0] -= expf(Uq[0]*-0.15) * expf(Uq[0]*-0.15) - 1;}
            if(Uq[1] >= 0){Uq[1] += expf(Uq[1]*0.15) * expf(Uq[1]*0.15) - 1;}
            else{Uq[1] -= expf(Uq[1]*-0.15) * expf(Uq[1]*-0.15) - 1;}

//            alpha = (Ka[1] * (input.actual_target_speed - Instantaneous_speed) + Ka[2] * imu.roll / 180.0f * _PI + Ka[3] * now_gx);

//            if(wheel_L.y - wheel_R.y > -60.0f && imu.pitch < 0.0f)
//            {
//                I_pitch += imu.pitch / 180.0f * _PI;
//            }
//            else if(wheel_L.y - wheel_R.y < 60.0f && imu.pitch > 0.0f)
//            {
//                I_pitch += imu.pitch / 180.0f * _PI;
//            }
//            legL = -1 * KlegL[6] * I_pitch + 50.0f;
//            legR = -1 * KlegR[6] * I_pitch + 50.0f;

            if(alpha < _PI_4 && alpha > -1 * _PI_4)
            {
                wheel_L.x = centroid_distance * tan(alpha) + input.Car_zero - sinf(imu.roll/ 180.0f * _PI + alpha) * 135.0f - tanf(imu.roll/ 180.0f * _PI) * (centroid_distance - 45.0f);
                wheel_R.x = centroid_distance * tan(alpha) + input.Car_zero - sinf(imu.roll/ 180.0f * _PI + alpha) * 135.0f - tanf(imu.roll/ 180.0f * _PI) * (centroid_distance - 45.0f);
                wheel_L.x = _constrain(wheel_L.x,-70.0f,70.0f);
                wheel_R.x = _constrain(wheel_R.x,-70.0f,70.0f);
                wheel_L.x = kalman_update(&Lx, wheel_L.x);
                wheel_R.x = kalman_update(&Rx, wheel_R.x);
            }
//            if(input.batter_voltage < 11.4)
//            {
//                Uq[0] = 0;
//                Uq[1] = 0;
//            }
//            speed[0] = foc_openloop_update(&motor_L,Uq[0] * -1);
//            speed[1] = foc_openloop_update(&motor_R,Uq[1]);

            if(timer <= 80)
            {
                wheel_L.y = cosf(imu.roll/ 180.0f * _PI) * 135.0f;
                wheel_R.y = cosf(imu.roll/ 180.0f * _PI) * 135.0f;
            }
            else
            {
                wheel_L.y = 45.0f;
                wheel_R.y = 45.0f;
                input.flag = 3;
                timer = 0;
            }

            break;
        }
        case 3: //空中
        {
            timer += pit_time0_ms;

//            for(i = 0; i < 7; i++)
//            {
//                KL[i] = LS_STRAIGHT_K.data[0][i];
//                KR[i] = LS_STRAIGHT_K.data[1][i];
//            }
//            get_controller_gains(&LS_STRAIGHT_K, KL, KR, Ka);
//            KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
//            KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);

            Uq[0] = -1.7f * (KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * imu.roll / 180.0f * _PI + KL[3] * now_gx);// - KL[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KL[5] * now_gz);//L
            Uq[1] = -1.7f * (KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * imu.roll / 180.0f * _PI + KR[3] * now_gx);// - KR[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KR[5] * now_gz);

            if(Uq[0] >= 0){Uq[0] += expf(Uq[0]*0.15) * expf(Uq[0]*0.15) - 1;}
            else{Uq[0] -= expf(Uq[0]*-0.15) * expf(Uq[0]*-0.15) - 1;}
            if(Uq[1] >= 0){Uq[1] += expf(Uq[1]*0.15) * expf(Uq[1]*0.15) - 1;}
            else{Uq[1] -= expf(Uq[1]*-0.15) * expf(Uq[1]*-0.15) - 1;}

//            if(input.batter_voltage < 11.4)
//            {
//                Uq[0] = 0;
//                Uq[1] = 0;
//            }
//            speed[0] = foc_openloop_update(&motor_L,Uq[0] * -1);
//            speed[1] = foc_openloop_update(&motor_R,Uq[1]);

            wheel_L.x = input.Car_zero - sinf(imu.roll/ 180.0f * _PI + alpha) * centroid_distance - tanf(imu.roll/ 180.0f * _PI) * (centroid_distance - 45.0f);
            wheel_R.x = input.Car_zero - sinf(imu.roll/ 180.0f * _PI + alpha) * centroid_distance - tanf(imu.roll/ 180.0f * _PI) * (centroid_distance - 45.0f);
            wheel_L.y = 45.0f;
            wheel_R.y = 45.0f;
            wheel_L.x = kalman_update(&Lx, wheel_L.x);
            wheel_R.x = kalman_update(&Rx, wheel_R.x);

            if((timer > 300 || fabsf(imu.az_linear) > 2.0f * GRAVITY) && timer > 50)
            {
                input.flag = 4;
                timer = 0;
//                input.actual_target_speed *= 0.75f;
//                start_flag = 1;
            }
        }
            break;
        case 4: //地面运行
        {
            timer += pit_time0_ms;
            if ((timer > 300 && take_off_flag)||(timer > 100 && imu.resultant_acceleration > 3.0f && take_off_flag))
            {
                timer = 0;
                input.flag = 1;
                begin_yaw = -10.0f;
//                input.actual_target_speed = 0;
            }

            E_yaw = fmodf(begin_yaw - imu.yaw / 180.0f * _PI , _2PI);
            E_yaw = E_yaw > _PI ? E_yaw - _2PI : E_yaw;
            E_yaw = E_yaw < -_PI ? E_yaw + _2PI : E_yaw;

            input.horizontal_E = E_yaw;//fmodf(begin_yaw - imu.yaw / 180.0f * _PI, _PI);
//            get_controller_gains(&LS_STRAIGHT_K, KL, KR, Ka);

//            KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
//            KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);

//            input.actual_target_speed = 1.5;
            float roll_compensation =  imu.roll > -5.0f ? 0.267533179f * Instantaneous_speed - alpha : 0;
            alpha = (Ka[1] * (input.actual_target_speed - Instantaneous_speed) + Ka[2] * imu.roll / 180.0f * _PI + Ka[3] * now_gx);

            Jp_ratio = ((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l/ cosf(alpha)/ cosf(alpha))/((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l);  //车体绕质心俯仰旋转的转动惯量kg*m^2
            Jc_ratio = ((M - 2 * m) * (length * length + width * width) / 12.0f + (M - 2 * m) * l * l * tanf(alpha) * tanf(alpha))/((M - 2 * m) * (length * length + width * width) / 12.0f);
            Uq[0] = -1.7f * ((KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KL[3] * now_gx) *Jp_ratio + (-KL[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KL[5] * now_gz)*Jc_ratio + torque_compensation_L);//L
            Uq[1] = -1.7f * ((KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KR[3] * now_gx) *Jp_ratio + (-KR[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KR[5] * now_gz)*Jc_ratio + torque_compensation_R);

            float torque_difference = Uq[0] - Uq[1];
            float aphla_L_ratio = 1.0f, aphla_R_ratio = 1.0f;
            if(torque_difference > 0){
                aphla_L_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.3f + 1.0f;
                aphla_R_ratio = 2.0f - aphla_L_ratio;
            }
            else{
                aphla_R_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.3f + 1.0f;
                aphla_L_ratio = 2.0f - aphla_R_ratio;
            }

            if (fabsf(torque_difference) > MAX_TORQUE_DIFFERENCE)
            {
                if(Uq[0] > Uq[1])
                {
                    Uq[0] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE) / 2.0f;
                    Uq[1] = Uq[0] - MAX_TORQUE_DIFFERENCE;
                }
                else
                {
                    Uq[1] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE) / 2.0f;
                    Uq[0] = Uq[1] - MAX_TORQUE_DIFFERENCE;
                }
            }
//            Uq[0] = -2 * (KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * imu.roll / 180.0f * _PI + KL[3] * now_gx - KL[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KL[5] * now_gz + torque_compensation_L);//L
//            Uq[1] = -2 * (KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * imu.roll / 180.0f * _PI + KR[3] * now_gx - KR[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KR[5] * now_gz + torque_compensation_R);

            if(Uq[0] >= 0){Uq[0] += expf(Uq[0]*0.15) * expf(Uq[0]*0.15) - 1;}
            else{Uq[0] -= expf(Uq[0]*-0.15) * expf(Uq[0]*-0.15) - 1;}
            if(Uq[1] >= 0){Uq[1] += expf(Uq[1]*0.15) * expf(Uq[1]*0.15) - 1;}
            else{Uq[1] -= expf(Uq[1]*-0.15) * expf(Uq[1]*-0.15) - 1;}
//            printf("%f,%f\n",Uq[0],Uq[1]);


//            if(take_off_flag){alpha *=0.8;}
//            _constrain(legL,30.0f,130.0f);
//            _constrain(legR,30.0f,130.0f);
//            wheel_L.y = kalman_update(&Ly, legL);
//            wheel_R.y = kalman_update(&Ry, legR);

            wheel_L.y = 45.0f;
            wheel_R.y = 45.0f;

            if(alpha < _PI_4 && alpha > -1 * _PI_4)
            {
                wheel_L.x = centroid_distance * tan(alpha) + input.Car_zero;
                wheel_R.x = centroid_distance * tan(alpha) + input.Car_zero;
                wheel_L.x = _constrain(wheel_L.x,-70.0f,70.0f);
                wheel_R.x = _constrain(wheel_R.x,-70.0f,70.0f);
                wheel_L.x = kalman_update(&Lx, wheel_L.x);
                wheel_R.x = kalman_update(&Rx, wheel_R.x);
            }
//            speed[0] = foc_openloop_update(&motor_L,Uq[0] * -1);
//            speed[1] = foc_openloop_update(&motor_R,Uq[1]);
            break;
        }
        case 5: //地面单边桥运行
        {
            input.flag = 0;
            beel = 1000;
            break;
            timer += pit_time0_ms;
            if(timer <= 300)
            {
                leg_increase = 40.0f * timer / 300.0f ;
            }

            get_adaptive_controller_gains(&COMMON_K, &HIGH_K,  KL,  KR,  Ka, ((float)timer / 300.0f));
            KL[2] = sigmoid_adaptive_q_weight(imu.roll, KL[2]);
            KR[2] = sigmoid_adaptive_q_weight(imu.roll, KR[2]);
            KL[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KL[1]))*1.0f;
            KR[1] = sqrtf_pro(sigmoid_adaptive_q_weight((input.actual_target_speed - Instantaneous_speed), KR[1]))*1.0f;

//            I_pitch += imu.pitch / 180.0f * _PI;
            leg.now_data = imu.pitch / 180.0f * _PI;//实时数据初始化
//            place_pid_update(&leg,now_gy);
            if(abs(leg.output+10)>4)
            {
                leg.Kp = 2;
            }
            else
            {
                leg.Kp = 2;
            }
            increment_pid_update(&leg);
            //该标志位用于判断初始巡线点的位置 0为中心 1为右边 2为左边
//            bridge_flag = 1;

//            detect_terrain(now_gy);
            float ff_leg_comp = 0;
//            if(slope_status){beel = 2000;}
//            if(timer > 500)
//            {
//                if(slope_status == -1){
//                    ff_leg_comp = 1.0f * sinf(fabsf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy)) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output + ff_leg_comp;
//                }
//                else if(slope_status == -2){
//                    ff_leg_comp = -1.0f * sinf(fabsf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy)) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output + ff_leg_comp;
//                }
//                else if(slope_status == 1){
//                    ff_leg_comp = 1.0f * sinf(fabsf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy)) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output - ff_leg_comp;
//                }
//                else if(slope_status == 2){
//                    ff_leg_comp = -1.0f * sinf(fabsf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy)) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output - ff_leg_comp;
//                }
//            }

//            if(timer > 300)
//            {
//                if(bridge_flag == 2){
//                    ff_leg_comp = 1.0f * sinf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output + ff_leg_comp;
//                }
//                else if(bridge_flag == 1){
//                    ff_leg_comp = -1.0f * sinf(imu.pitch/ 180.0f * _PI + pit_time0_ms * now_gy) * WHEEL_RADIUS * 1000.0f;
//                    delta_y = leg.output + ff_leg_comp;
//                }
//                else {
//                    delta_y = 0;
//                }
//
//            }

            if(fabsf(imu.pitch) > 10.0f){ff_leg_comp = sinf(imu.pitch / 180.0f * _PI + pit_time0_ms * now_gy / 1000.0f) * WHEEL_RADIUS * 1000.0f;}
            if(ff_leg_comp * leg.output <= 0){
                ff_leg_comp *= _constrain(fabsf(imu.pitch)/2.5f,0,1.0f);
                ff_leg_comp = _constrain(ff_leg_comp,-20.0f,20.0f);
//                input.actual_target_speed = 0.3f;
//                leg.Ki = 2.0f;
            }
            else{
                ff_leg_comp *= _constrain(fabsf(imu.pitch)/2.5f,0,1.0f);
                ff_leg_comp = _constrain(ff_leg_comp,-20.0f,20.0f);
//                input.actual_target_speed = 0.0f;
//                leg.E_integral= 0.0f;
            }




            leg.output = _constrain(leg.output, -100.0f, 100.0f);
//            printf("%f\n",leg.output);
            delta_y = leg.output;//输出初始化

//            printf("%f\n",delta_y);
//            printf("%f,%f,%f,%f\n",leg.now_data,delta_y,leg.output,ff_leg_comp);
//            delta_y += 0.5f * WHEEL_RADIUS * 1000.0f * tanf(now_gy);
//            printf("%f,%f,%f\n",imu.pitch / 180.0f * _PI,I_pitch,leg.output);

            delta_y = _constrain(delta_y, -60.0f, 60.0f);

            delta_y = 0;
            torque_compensation_L = torque_compensation_R = 1.0f;
            if(delta_y >= 0)
            {
                legR =  45.0f + leg_increase;
                legL =  45.0f  + leg_increase - delta_y;
                if(fabsf(delta_y) > 5.0f)
                {
                    if(imu.pitch > 10.0f)
                    {
                        torque_compensation_L = 1.8f;
                        torque_compensation_R = 1.0f;
                    }
                    else if(imu.pitch <-10.0f)
                    {
                        torque_compensation_L = 0.5f;
                        torque_compensation_R = 1.2f;
                    }
                }
            }
            else
            {
                legR = 45.0f  + leg_increase + delta_y;
                legL = 45.0f + leg_increase;
                if(fabsf(delta_y) > 5.0f && imu.pitch <-10.0f)
                {
                    if(imu.pitch > 10.0f)
                    {
                        torque_compensation_L = 1.2f;
                        torque_compensation_R = 0.5f;
                    }
                    else if(imu.pitch <-10.0f)
                    {
                        torque_compensation_L = 1.0f;
                        torque_compensation_R = 1.8f;
                    }
                }

            }
            float max_l = l + leg_increase/ 1000.0f;
            l += ((legR + legL) / 2000.0f);

            begin_yaw = 0;

            E_yaw = fmodf(begin_yaw - imu.yaw / 180.0f * _PI , _2PI);
            E_yaw = E_yaw > _PI ? E_yaw - _2PI : E_yaw;
            E_yaw = E_yaw < -_PI ? E_yaw + _2PI : E_yaw;

            input.horizontal_E = E_yaw;

//            input.horizontal_E = 0;
            input.actual_target_speed = 0.0f;

            float roll_compensation =  imu.roll > -5.0f ? 0.267533179f * Instantaneous_speed - alpha : 0;
            alpha = (Ka[1] * (input.actual_target_speed - Instantaneous_speed) + Ka[2] * imu.roll / 180.0f * _PI + Ka[3] * now_gx);

            Jp_ratio = ((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l/ cosf(alpha)/ cosf(alpha))/((M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * max_l * max_l);  //车体绕质心俯仰旋转的转动惯量kg*m^2
            Jc_ratio = ((M - 2 * m) * (length * length + width * width) / 12.0f + (M - 2 * m) * l * l * tanf(alpha) * tanf(alpha))/((M - 2 * m) * (length * length + width * width) / 12.0f);
            Uq[0] = -1.8f * ((KL[1] * (input.actual_target_speed - Instantaneous_speed) + KL[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KL[3] * now_gx) * (Jp_ratio) * torque_compensation_L + (-KL[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KL[5] * now_gz)*Jc_ratio);//L
            Uq[1] = -1.8f * ((KR[1] * (input.actual_target_speed - Instantaneous_speed) + KR[2] * (imu.roll / 180.0f * _PI + 0.7f * roll_compensation) + KR[3] * now_gx) * (Jp_ratio) * torque_compensation_R + (-KR[4] * (input.horizontal_E)/* - imu.yaw/ 180.0f * _PI)*/ - KR[5] * now_gz)*Jc_ratio);

//            printf("%.10f,%.10f,%.10f,%.10f,%.10f\n",Uq[0],Uq[1],alpha,Jp_ratio,Jc_ratio);
            float torque_difference = Uq[0] - Uq[1];
//            float aphla_L_ratio = 1.0f, aphla_R_ratio = 1.0f;
//            if(torque_difference > 0){
//                aphla_L_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.4f + 1.0f;
//                aphla_R_ratio = 2.0f - aphla_L_ratio;
//            }
//            else{
//                aphla_R_ratio = fabsf(torque_difference) / MAX_TORQUE_DIFFERENCE * 0.4f + 1.0f;
//                aphla_L_ratio = 2.0f - aphla_R_ratio;
//            }

            if (fabsf(torque_difference) > MAX_TORQUE_DIFFERENCE * 2.0f)
            {
                if(Uq[0] > Uq[1])
                {
                    Uq[0] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE* 2.0f) / 2.0f;
                    Uq[1] = Uq[0] - MAX_TORQUE_DIFFERENCE* 2.0f;
                }
                else
                {
                    Uq[1] = (Uq[0] + Uq[1] + MAX_TORQUE_DIFFERENCE* 2.0f) / 2.0f;
                    Uq[0] = Uq[1] - MAX_TORQUE_DIFFERENCE* 2.0f;
                }
            }

            if(Uq[0] >= 0){Uq[0] += expf(Uq[0]*0.15) * expf(Uq[0]*0.15) - 1;}
            else{Uq[0] -= expf(Uq[0]*-0.15) * expf(Uq[0]*-0.15) - 1;}
            if(Uq[1] >= 0){Uq[1] += expf(Uq[1]*0.15) * expf(Uq[1]*0.15) - 1;}
            else{Uq[1] -= expf(Uq[1]*-0.15) * expf(Uq[1]*-0.15) - 1;}

            legL = _constrain(legL,30.0f,130.0f);
            legR = _constrain(legR,30.0f,130.0f);
//            kalman_change(&Ly, 1, 50);
//            kalman_change(&Ry, 1, 50);

            wheel_L.y = kalman_update(&Ly, legL);
            wheel_R.y = kalman_update(&Ry, legR);

            if(alpha < _PI_4 && alpha > -1 * _PI_4)
            {
                wheel_L.x = (centroid_distance + legL - 45.0f) * tan(alpha) + input.Car_zero;
                wheel_R.x = (centroid_distance + legR - 45.0f) * tan(alpha) + input.Car_zero;
                wheel_L.x = _constrain(wheel_L.x,-70.0f,70.0f);
                wheel_R.x = _constrain(wheel_R.x,-70.0f,70.0f);
                wheel_L.x = kalman_update(&Lx, wheel_L.x);
                wheel_R.x = kalman_update(&Rx, wheel_R.x);
            }

            break;
        }
    }
//    printf("%f,%f,%f,%f\n",speed[0],speed[1],Uq[0],Uq[1]);
    /*电机状态更新*/
//    speed[0] = 0;
//    speed[0] = 0;
// steering_engine_run(&steer_RR,90.0);

//    Uq[0]=0.3;
//    Uq[1]=0.3;
//    printf("%f,%f,%f,%f\n",speed[0],speed[1],Uq[0],Uq[1]);
//    printf("%f,%f\n",Uq[0],Uq [1]);
//    tx_buffer[0] = 0xFF;
//    tx_buffer[9] = 0xFE;
//    memcpy(tx_buffer + 1,(uint8*)Uq,sizeof(Uq));
//    uart_write_buffer(UART_1, &tx_buffer[0], 10);
    if(input.flag)
    {
        time_stamp += pit_time0_ms;
        if(logger_send_flag==0)
        {
            logger_send_flag = 1;
        }

    }

    else if(input.flag == 0 && input.prev_flag != 0)
    {
        if(logger_save_flag==0)
        {
            logger_save_flag = 1;
            time_stamp = 0;
        }
    }
    input.prev_flag = input.flag;

//    time_stamp += pit_time0_ms;
//    if(logger_send_flag==0)
//    {
//        logger_send_flag = 1;
//    }
}

void steering_engine_update(void)
{
//    wheel_L.x = 0;
//    wheel_R.x = 37;
//    wheel_L.x = kalman_update(&Lx, wheel_L.x);

    Inverse_kinematics(&wheel_L);
//     printf("%f,%f,%f,%f\n",wheel_L.front_angle,wheel_L.rear_angle,wheel_R.front_angle,wheel_R.rear_angle);
//    wheel_L.front_angle = kalman_update(&LF, wheel_L.front_angle);
//    wheel_L.rear_angle = kalman_update(&LR, wheel_L.rear_angle);

    float front_angle = 90.0 + wheel_L.front_angle;
    float rear_angle = -90.0 + wheel_L.rear_angle;

    if(front_angle >= 0.0f && front_angle <= 180.0f && rear_angle >= 0.0f && rear_angle <= 180.0f)
    {
        steering_engine_run(&steer_LF,front_angle);
        steering_engine_run(&steer_LR,rear_angle);
    }


    Inverse_kinematics(&wheel_R);
//    wheel_R.x = kalman_update(&Rx, wheel_R.x);
//    wheel_R.front_angle = kalman_update(&RF, wheel_R.front_angle);
//    wheel_R.rear_angle = kalman_update(&RR, wheel_R.rear_angle);

    front_angle = 90.0 - wheel_R.front_angle;
    rear_angle = 270.0 - wheel_R.rear_angle;

    if(front_angle >= 0.0f && front_angle <= 180.0f && rear_angle >= 0.0f && rear_angle <= 180.0f)
    {
        steering_engine_run(&steer_RF,front_angle);
        steering_engine_run(&steer_RR,rear_angle);
    }


//    steering_engine_run(&steer_RR,90.0);
//    steering_engine_run(&steer_RF,90.0);
//    steering_engine_run(&steer_LR,90.0);
//    steering_engine_run(&steer_LF,90.0);
}

void brushless_motor_update(void)
{

//    Uq[0] = _constrain(Uq[0],-5.0f,5.0f);
//    Uq[1] = _constrain(Uq[1],-5.0f,5.0f);
    if(vel_kf.is_slip){
        Uq[0] = _constrain(Uq[0],-5.0f,5.0f);
        Uq[1] = _constrain(Uq[1],-5.0f,5.0f);
    }
    else{
        Uq[0] = _constrain(Uq[0],-3.0f,3.0f);
        Uq[1] = _constrain(Uq[1],-3.0f,3.0f);
    }
    Uq[0] = kalman_update(&LM, Uq[0]);
    Uq[1] = kalman_update(&RM, Uq[1]);
//    Uq[0] = 0.6f;
//    Uq[1] = 0.6f;
    foc_openloop_update(&motor_L,Uq[0]);
    foc_openloop_update(&motor_R,Uq[1]);
}//pure_angle
void base_get_speed(void)
{
    speed[0] = foc_get_speed(&motor_L);
    speed[1] = foc_get_speed(&motor_R);
    speed[0] = _constrain(speed[0],-90.0f,90.0f);
    speed[1] = _constrain(speed[1],-90.0f,90.0f);

    Instantaneous_speed = ((speed[0] - speed[1]) / 2.0f + imu.gx) * WHEEL_DIAMETER / 2.0f;
//   printf("%f,%f,%f\n",speed[0],speed[1],Instantaneous_speed);
}

// 地形特征提取（检测单侧斜坡）
void detect_terrain(float now_gy)
{

    if(speed[0] > -speed[1] && now_gy < -0.2f)//左轮速大于右轮速且向右滚转
    {
        slope_status = -1;//左轮上坡
    }
    else if(speed[0] > -speed[1] && now_gy > 0.2f)//左轮速大于右轮速且向左滚转
    {
        slope_status = -2;//左轮下坡
    }
    else if(speed[0] < -speed[1] && now_gy > 0.2f)//左轮速小于右轮速且向左滚转
    {
        slope_status = 1;//右轮上坡
    }
    else if(speed[0] < -speed[1] && now_gy < -0.2f)//左轮速小于右轮速且向右滚转
    {
        slope_status = 2;//右轮下坡
    }
    else
    {
        slope_status = 0;//无状态
    }

    slope_status_filter.data = slope_status;
    debounce_filter_update(&slope_status_filter);
    slope_status = slope_status_filter.output;
}
