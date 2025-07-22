/*
 * ���ߣ��۸�
 * ��0�����˶����ƣ���1������·ʶ��
 *
 * */
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
extern IfxCpu_mutexLock tu;
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
/*
 * motor_value.receive_left_speed_data              ����ת��%d
 * motor_value.receive_right_speed_data             �ҵ��ת��%d
 * euler_angle.pitch(����)                          �����ǣ���X�ᣨ���ۣ�
 * euler_angle.roll(����)                           ����ǣ���Y�ᣨ���ۣ�
 * ���������Ƿ������⣬����roll��pitch,ԭpitch��roll
 * euler_angle.pitch                                ����ǣ���Y�ᣨʵ�ʣ���ֱ����0����4�������Դ�ߵ����-90����������ź��ߵ����+90������Ϊ0%f
 * euler_angle.roll                                 �����ǣ���X�ᣨʵ�ʣ���ֱ����180��������һ����+180��������һ����-180����������0%f
 * imu660ra_gyro_x                                  x����ٶȣ���������Ϊ������������Ϊ����ֱ����0%d��0~+-1000��2000
 * imu660ra_gyro_y                                  y����ٶȣ���4�������Դ��Ϊ������������ź���Ϊ����ֱ����0%d
 * imu660ra_gyro_z                                  z����ٶȣ�0~1200��%d
 * */
uint32 system_image_time = 0;
// �� isr.c �ļ����������Щ��̬����
static float obstacle_prev_pitch = 0;            // ��¼�ϴε�pitchֵ
static unsigned char obstacle_left_climbing_flag = 0;
static unsigned char obstacle_right_climbing_flag = 0;
static float obstacle_original_left_kp = 0.18f;
static float obstacle_original_right_kp = 0.18f;
static unsigned char obstacle_system_initialized = 0;
int delayt=0;

/**
 * @brief �����ȸ߲��ת�ٲ�������
 * @param void
 * @return void
 * @note ���������ȸ߶Ȳ����ת�ٲ�����
 *       - �����ȶ����ȸ�ʱ������ת������adjustment_ratio��
 *       - �����ȶ����ȸ�ʱ������ת������adjustment_ratio��
 *       - ���ȸ߲�ص�ƽ��״̬ʱ���ָ�ԭ��ת��
 */
void wheel_obstacle_height_control(int i)
{
    // ��ʼ��ԭʼ����
    //if(((motor_value.receive_left_speed_data<0||-motor_value.receive_right_speed_data<0)&&delayt==0)&&mode!=stopworking&&bridge_in_flag_num==0/*&&bridge_entry_cooldown==0*/){BridgeState=SINGLE_BRIDGE_ACTIVE;mode=flexible;ServoPID.angle_roll=0;}
    if(BridgeState==SINGLE_BRIDGE_NOT_ACTIVE||i==0)return;
    if(obstacle_system_initialized == 0)
    {
        obstacle_original_left_kp = pid2_flexible.sudu_left.kp;
        obstacle_original_right_kp = pid2_flexible.sudu_right.kp;
        obstacle_system_initialized = 1;
    }

    // �����ȸ߲���ȸ߶� - ���ȸ߶ȣ�
    float height_diff = ServoPID.highleft - ServoPID.highright;
    
    const float height_threshold = 0.1f; // �ȸ߲���ֵ
    const float adjustment_ratio = 11.0f / 9.8f; // ת�ٵ�������
    
    static unsigned char left_adjust_flag = 0;
    static unsigned char right_adjust_flag = 0;
    
    // ���ȴ����߼��������ȶ����ȸ�ʱ��height_diff < 0��
    if(height_diff < -height_threshold) 
    {
        if(left_adjust_flag == 0)
        {
            pid2_flexible.sudu_left.kp *= adjustment_ratio;
            left_adjust_flag = 1;
        }
        // ͬʱ�ָ�����ת��
        if(right_adjust_flag == 1)
        {
            pid2_flexible.sudu_right.kp = obstacle_original_right_kp;
            right_adjust_flag = 0;
        }
    }
    // ���ȴ����߼��������ȶ����ȸ�ʱ��height_diff > 0��
    else if(height_diff > height_threshold) 
    {
        if(right_adjust_flag == 0)
        {
            pid2_flexible.sudu_right.kp *= adjustment_ratio;
            right_adjust_flag = 1;
        }
        // ͬʱ�ָ�����ת��
        if(left_adjust_flag == 1)
        {
            pid2_flexible.sudu_left.kp = obstacle_original_left_kp;
            left_adjust_flag = 0;
        }
    }
    // �ȸ߲�����ֵ��Χ�ڣ��ָ�����ת��
    else
    {
        if(left_adjust_flag == 1)
        {
            pid2_flexible.sudu_left.kp = obstacle_original_left_kp;
            left_adjust_flag = 0;
        }
        if(right_adjust_flag == 1)
        {
            pid2_flexible.sudu_right.kp = obstacle_original_right_kp;
            right_adjust_flag = 0;
        }
    }
}

/**
 * @brief ���ܺ���ǲ�������
 * @param void
 * @return void
 * @note ���Ȼ�ƽ���ٵ��ಹ��������ƽ�����
 */
int flexible_leg_high=7.6;
void follow_pitch_compensation(int modechoice)
{
    //if(mode != flexible) return;
    //if(delayt==0&&(motor_value.receive_left_speed_data>50&&-motor_value.receive_right_speed_data>50))delayt=1;
    if(BridgeState!=SINGLE_BRIDGE_ACTIVE&&bridge_in_flag==0)return;
    //turn_value=0;
    if(bridge_in_flag==1){
        if(ServoPID.highleft<flexible_leg_high){ServoPID.highleft+=0.04;}
        if(ServoPID.highright<flexible_leg_high){ServoPID.highright+=0.04;}
    }
    if(modechoice==0){ 
    if((euler_angle.pitch > 2 || euler_angle.pitch < -2))
    {   
        //float adjustment = 0.001f;
        float adjustment = 0.005f * (euler_angle.pitch > 0 ? euler_angle.pitch : -euler_angle.pitch); // ��������Ƕȴ�С������
        float height_diff = ServoPID.highleft - ServoPID.highright; // ����-���ȵĸ߶Ȳ�
        
        if(euler_angle.pitch > 0) // ������б���ұߵ���߸�
        {
            if(height_diff > 0.05f) // ���ȱ����ȸ߳�����ֵ
            {
                // ���ȼ������ȸ߶ȣ�����ȸ�
                ServoPID.highleft -= adjustment;
            }
            else // ���Ȼ����ȸ߻������Ѿ������ȸ�
            {
                // �������ȸ߶�
                ServoPID.highright += adjustment;
            }
        }
        else if(euler_angle.pitch < 0) // ������б����ߵ��ұ߸�
        {
            if(height_diff < -0.05f) // ���ȱ����ȸ߳�����ֵ
            {
                // ���ȼ������ȸ߶ȣ�����ȸ�
                ServoPID.highright -= adjustment;
            }
            else // ���Ȼ����ȸ߻������Ѿ������ȸ�
            {
                // �������ȸ߶�
                ServoPID.highleft += adjustment;
            }
        }
        
        // �߶��޷�����
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
             if((euler_angle.pitch > 1||euler_angle.pitch < -1)){
                 ServoPID.angle_pitch=ServoPID.angle_pitch+0.00055*euler_angle.pitch;
                 LIMIT(ServoPID.angle_pitch,-0.6,0.6);
             }
    }
    //leg_height_balance_control_wheel_speed();
}
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
    imu660ra_init();//�����ǳ�ʼ��,����������ٶȣ��������˲�����Ƕ�
    EKF_Init();//��������ʼ��
    small_driver_uart_init();//��ˢ���� ������ʼ�������ٶ�
    pid_init_all();
    servo_init();
    mode=stopworking;
    //BridgeState = SINGLE_BRIDGE_ACTIVE;
    //pid1_walk.run_speed=0;//�ٶȳ�ʼ��
    mt9v03x_init();
    system_delay_ms(200);
    ips200_init(IPS200_TYPE_PARALLEL8);
    buzzer_init();
    my_key_init();

    //seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);//
    //wireless_uart_init();
    pit_ms_init(CCU60_CH1,1);//�ж����ó�ʼ��
    pit_ms_init(CCU61_CH0,10);//�ж����ó�ʼ��

    //system_start();

    //system_image_time = system_getval_ns();
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    system_delay_ms(200);
    adjust_menu();
    while (TRUE)
    {
        //if(IfxCpu_acquireMutex(&tu))//˫�˴���
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
                bridge_in_flag_num=30;
                pid1_walk.run_speed=0;//�ٶȳ�ʼ��
                turn_value=0;
                if(bridge_in_flag_num>=30)
                {
                bridge_in_flag_num=0;
                bridge_in_flag=0;
                mode_switch(flexible);
                ServoPID.highleft=7.0f;
                ServoPID.highright=7.0f;
                BridgeState = SINGLE_BRIDGE_ACTIVE;
                //ServoPID.angle_roll=0;
                }
            }
            if(bridge_out_flag){
                BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;// ��������״̬Ϊδ����
                ServoPID.angle_pitch=0;//4��4��0.17���
                mode_switch(walk);
                ips200_full(RGB565_66CCFF);
                bridge_out_flag=0;
            }
            if(delayt){//��Ծ����
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
        // �˴���д��Ҫѭ��ִ�еĴ���
}


#pragma section all restore
// **************************** �������� ****************************

