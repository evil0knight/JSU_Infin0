/*
 * ���ߣ��۸�
 * */
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"
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
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
    imu660ra_init();//�����ǳ�ʼ��,����������ٶȣ��������˲�����Ƕ�
    EKF_Init();//��������ʼ��
    small_driver_uart_init();//��ˢ���� ������ʼ�������ٶ�
    pid_init_all();
    //servo_init();
    mode=stopworking;
    ips200_init                     (IPS200_TYPE_SPI);

    //seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);//
    //wireless_uart_init();
    pit_ms_init(CCU60_CH1,1);//�ж����ó�ʼ��
    //pit_ms_init(CCU61_CH0,20);//�ж����ó�ʼ��
    system_start();

    system_image_time = system_getval_ns();
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        show_pid_information();//չʾ��Ϣ
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
// **************************** �������� ****************************
