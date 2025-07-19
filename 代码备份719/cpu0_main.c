/*
 * 作者：眼膏
 * 核0用来运动控制，核1用来道路识别
 *
 * */
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
extern IfxCpu_mutexLock tu;
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
/*
 * motor_value.receive_left_speed_data              左电机转速%d
 * motor_value.receive_right_speed_data             右电机转速%d
 * euler_angle.pitch(理论)                          俯仰角，绕X轴（理论）
 * euler_angle.roll(理论)                           横滚角，绕Y轴（理论）
 * 由于陀螺仪放置问题，现在roll是pitch,原pitch是roll
 * euler_angle.pitch                                横滚角，绕Y轴（实际），直立是0，往4根舵机电源线倒最多-90，往舵机板信号线倒最多+90翻个面为0%f
 * euler_angle.roll                                 俯仰角，绕X轴（实际），直立是180，下载器一边是+180，报警器一边是-180，翻个面是0%f
 * imu660ra_gyro_x                                  x轴角速度，往下载器为正，往报警器为负，直立是0%d，0~+-1000，2000
 * imu660ra_gyro_y                                  y轴角速度，往4根舵机电源线为正，往舵机板信号线为负，直立是0%d
 * imu660ra_gyro_z                                  z轴角速度，0~1200，%d
 * */
uint32 system_image_time = 0;
// 在 isr.c 文件顶部添加这些静态变量
static float obstacle_prev_pitch = 0;            // 记录上次的pitch值
static unsigned char obstacle_left_climbing_flag = 0;
static unsigned char obstacle_right_climbing_flag = 0;
static float obstacle_original_left_kp = 0.18f;
static float obstacle_original_right_kp = 0.18f;
static unsigned char obstacle_system_initialized = 0;
int delayt=0;

/**
 * @brief 基于pitch角持续变化趋势的转速补偿控制
 * @param void
 * @return void
 * @note 结合pitch角持续变化趋势进行转速补偿：
 *       - 当euler_angle.pitch > 0且持续变大时，左腿转速增加adjustment_ratio倍
 *       - 当euler_angle.pitch > 0且持续变小时，左腿转速恢复原值
 *       - 当euler_angle.pitch < 0且持续变小时，右腿转速增加adjustment_ratio倍
 *       - 当euler_angle.pitch < 0且持续变大时，右腿转速恢复原值
 *       - 避免频繁的来回调整，只在持续趋势下执行操作
 */
void wheel_obstacle_height_control(int i)
{
    // 初始化原始参数
    //if(((motor_value.receive_left_speed_data<0||-motor_value.receive_right_speed_data<0)&&delayt==0)&&mode!=stopworking&&bridge_in_flag_num==0/*&&bridge_entry_cooldown==0*/){BridgeState=SINGLE_BRIDGE_ACTIVE;mode=flexible;ServoPID.angle_roll=0;}
    if(BridgeState==SINGLE_BRIDGE_NOT_ACTIVE||i==0)return;
    if(obstacle_system_initialized == 0)
    {
        obstacle_original_left_kp = pid2_flexible.sudu_left.kp;
        obstacle_original_right_kp = pid2_flexible.sudu_right.kp;
        obstacle_prev_pitch = euler_angle.pitch;
        obstacle_system_initialized = 1;
    }

    // 计算pitch角变化
    float pitch_change = euler_angle.pitch - obstacle_prev_pitch;
    
    const float pitch_threshold = 0.0f; // pitch角阈值
    const float change_threshold = 0.001f; // pitch变化阈值
    const float adjustment_ratio = 11.0f / 9.8f; // 转速调整比例
    
    static unsigned char left_adjust_flag = 0;
    static unsigned char right_adjust_flag = 0;
    static unsigned char left_increasing_trend = 0;  // 左腿对应的pitch增大趋势标志
    static unsigned char left_decreasing_trend = 0;  // 左腿对应的pitch减小趋势标志
    static unsigned char right_increasing_trend = 0; // 右腿对应的pitch增大趋势标志
    static unsigned char right_decreasing_trend = 0; // 右腿对应的pitch减小趋势标志
    
    // 左腿处理逻辑：当pitch > 0时
    if(euler_angle.pitch > pitch_threshold) 
    {
        if(pitch_change > change_threshold) // pitch在变大
        {
            left_increasing_trend = 1;
            left_decreasing_trend = 0;
            if(left_adjust_flag == 0)
            {
                pid2_flexible.sudu_left.kp *= adjustment_ratio;
                left_adjust_flag = 1;
            }
        }
        else if(pitch_change < -change_threshold) // pitch在变小
        {
            left_decreasing_trend = 1;
            left_increasing_trend = 0;
            if(left_adjust_flag == 1)
            {
                pid2_flexible.sudu_left.kp = obstacle_original_left_kp;
                left_adjust_flag = 0;
            }
        }
        // 如果变化很小，保持当前趋势标志不变
    }
    else if(euler_angle.pitch <= pitch_threshold && left_adjust_flag == 1) // pitch恢复到0以下
    {
        pid2_flexible.sudu_left.kp = obstacle_original_left_kp;
        left_adjust_flag = 0;
        left_increasing_trend = 0;
        left_decreasing_trend = 0;
    }
    
    // 右腿处理逻辑：当pitch < 0时
    if(euler_angle.pitch < -pitch_threshold) 
    {
        if(pitch_change < -change_threshold) // pitch在变小（更负）
        {
            right_decreasing_trend = 1;
            right_increasing_trend = 0;
            if(right_adjust_flag == 0)
            {
                pid2_flexible.sudu_right.kp *= adjustment_ratio;
                right_adjust_flag = 1;
            }
        }
        else if(pitch_change > change_threshold) // pitch在变大（变小负值）
        {
            right_increasing_trend = 1;
            right_decreasing_trend = 0;
            if(right_adjust_flag == 1)
            {
                pid2_flexible.sudu_right.kp = obstacle_original_right_kp;
                right_adjust_flag = 0;
            }
        }
        // 如果变化很小，保持当前趋势标志不变
    }
    else if(euler_angle.pitch >= -pitch_threshold && right_adjust_flag == 1) // pitch恢复到0以上
    {
        pid2_flexible.sudu_right.kp = obstacle_original_right_kp;
        right_adjust_flag = 0;
        right_increasing_trend = 0;
        right_decreasing_trend = 0;
    }
    
    // 更新历史pitch值
    obstacle_prev_pitch = euler_angle.pitch;
}

/**
 * @brief 智能横滚角补偿函数
 * @param void
 * @return void
 * @note 优先回平，再单侧补偿的智能平衡策略
 */
int flexible_leg_high=7.6;
void follow_pitch_compensation(int modechoice)
{
    //if(mode != flexible) return;
    //if(delayt==0&&(motor_value.receive_left_speed_data>50&&-motor_value.receive_right_speed_data>50))delayt=1;
    if(BridgeState==SINGLE_BRIDGE_NOT_ACTIVE)return;
    //turn_value=0;
    if(ServoPID.highleft<flexible_leg_high){ServoPID.highleft+=0.04;}
    if(ServoPID.highright<flexible_leg_high){ServoPID.highright+=0.04;}
    if(modechoice==0){ 
    if((euler_angle.pitch > 2 || euler_angle.pitch < -2))
    {   
        //float adjustment = 0.001f;
        float adjustment = 0.005f * (euler_angle.pitch > 0 ? euler_angle.pitch : -euler_angle.pitch); // 补偿量与角度大小成正比
        float height_diff = ServoPID.highleft - ServoPID.highright; // 左腿-右腿的高度差
        
        if(euler_angle.pitch > 0) // 车往右斜，右边低左边高
        {
            if(height_diff > 0.05f) // 左腿比右腿高超过阈值
            {
                // 优先减少左腿高度，趋向等高
                ServoPID.highleft -= adjustment;
            }
            else // 两腿基本等高或右腿已经比左腿高
            {
                // 增加右腿高度
                ServoPID.highright += adjustment;
            }
        }
        else if(euler_angle.pitch < 0) // 车往左斜，左边低右边高
        {
            if(height_diff < -0.05f) // 右腿比左腿高超过阈值
            {
                // 优先减少右腿高度，趋向等高
                ServoPID.highright -= adjustment;
            }
            else // 两腿基本等高或左腿已经比右腿高
            {
                // 增加左腿高度
                ServoPID.highleft += adjustment;
            }
        }
        
        // 高度限幅保护
        if(ServoPID.highleft > 8.0f)
            ServoPID.highleft -= 0.036f;
        if(ServoPID.highright > 8.0f)
            ServoPID.highright -= 0.036f;
        if(ServoPID.highleft < 2.8)
            ServoPID.highleft += 0.036f;
        if(ServoPID.highright < 2.8)
            ServoPID.highright += 0.036f;
    }
    
    }
    else if(modechoice==1) {
        if(ServoPID.highleft<flexible_leg_high){ServoPID.highleft+=0.015;ServoPID.highright+=0.015;}
             if((euler_angle.pitch > 2||euler_angle.pitch < -2)){
                 ServoPID.angle_pitch=ServoPID.angle_pitch+0.0005*euler_angle.pitch;
                 LIMIT(ServoPID.angle_pitch,-0.6,0.6);
             }
    }
    //leg_height_balance_control_wheel_speed();
}
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    imu660ra_init();//陀螺仪初始化,陀螺仪算角速度，卡尔曼滤波后算角度
    EKF_Init();//卡尔曼初始化
    small_driver_uart_init();//无刷驱动 参数初始化，给速度
    pid_init_all();
    servo_init();
    mode=stopworking;
    //BridgeState = SINGLE_BRIDGE_ACTIVE;
    //pid1_walk.run_speed=0;//速度初始化
    //BridgeState = SINGLE_BRIDGE_ACTIVE;
    mt9v03x_init();
    system_delay_ms(200);
    ips200_init(IPS200_TYPE_PARALLEL8);
    buzzer_init();
    my_key_init();

    //seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);//
    //wireless_uart_init();
    pit_ms_init(CCU60_CH1,1);//中断设置初始化
    pit_ms_init(CCU61_CH0,10);//中断设置初始化

    //system_start();

    //system_image_time = system_getval_ns();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    system_delay_ms(200);
    adjust_menu();
    while (TRUE)
    {
        //if(IfxCpu_acquireMutex(&tu))//双核传输
        {
            // if(annulus_L_memory_flag){
            //     //++annulus_L_memory_flag;
            //     if(annulus_L_memory_flag>=15){
            //        //annulus_L_memory_flag=0;
            //        annulus_L_memory=4;
            //     }
            // }
            if(bridge_in_flag==1||bridge_in_flag_num>0)
            {
                //++bridge_in_flag_num;
                pid1_walk.run_speed=0;//速度初始化
                turn_value=0;
                if(bridge_in_flag_num>=30)
                {
                bridge_in_flag_num=0;
                bridge_in_flag=0;
                mode_switch(flexible);
                BridgeState = SINGLE_BRIDGE_ACTIVE;
                //ServoPID.angle_roll=0;
                }
            }
            if(bridge_out_flag){
                //++bridge_out_flag_num;
                if(motor_value.receive_left_speed_data<3||-motor_value.receive_right_speed_data<3)bridge_out_flag_num=0;
                if(bridge_out_flag_num>=200&&(motor_value.receive_left_speed_data>30&&-motor_value.receive_right_speed_data>30)){
                    bridge_out_flag=0;
                    bridge_out_flag_num=0;
                    bridge_entry_cooldown=1;
                    BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;// 重置桥梁状态为未激活
                    ServoPID.angle_pitch=0;//4，4，0.17最大
                    mode_switch(walk);
                }
            }
            if(delayt){//跳跃后用
                //++delayt;
                if(delayt>=70)
                {
                    delayt=0;
                    mode=walk;
                    pid1_walk.run_speed=walk_speed;
                    jump_position_flag=0;
                }
            }
            //IfxCpu_releaseMutex(&tu);
        }
        //system_delay_ms(5);
    }
        // 此处编写需要循环执行的代码
}


#pragma section all restore
// **************************** 代码区域 ****************************

