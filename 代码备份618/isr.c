/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.20
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

static unsigned char ms20=0;//�����жϣ�20msִ��һ��
static unsigned char ms10=0;//�����жϣ�10msִ��һ��
static unsigned char ms5=0;//�����жϣ�5msִ��һ��
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);



}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);// �����ж�Ƕ��
    ++ms5;
    ++ms10;
    ++ms20;
    EKF_UpData();//�������˲�������̬,��ȡ�Ƕ�,���ٶ�
    angle_roll=Angle_Converter();//�����ǽǶ�ת��
    //if(mode!=jump){
    //    ServoPID.angle_roll=ServoPID.angle_roll+0.001*angle_roll;
    //   LIMIT(ServoPID.angle_roll,-15,15);
    //}
   // if(mode==flexible&&ServoPID.highleft>=5.3){
   //     ServoPID.angle_pitch=ServoPID.angle_pitch+0.0001*euler_angle.pitch;
   //     LIMIT(ServoPID.angle_pitch,-0.4,0.4);
   // }


    switch(mode){
        //*********************************************************************************************************************
        case flexible:{//ģʽ0��վ��,����pid
            /*if(ServoPID.highleft<6.3){
                ServoPID.highleft+=0.003;
                ServoPID.highright+=0.003;
            }*/
            if(ms20==20){
                ms20=0;
                /*run_speedǰ���ٶȣ�init.h��*/
                position_pid(&pid2_flexible.sudu_left,pid2_flexible.run_speed,motor_value.receive_left_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                position_pid(&pid2_flexible.sudu_right,pid2_flexible.run_speed,-motor_value.receive_right_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                }
            if(ms10==10){
                ms10=0;
                /*mechanical_neutral_point��е��ֵ��init.h��*/
                position_pid(&pid2_flexible.jiaodu_left,pid2_flexible.mechanical_neutral_point-pid2_flexible.sudu_left.output,angle_roll);//�ǶȻ�����������x��Ƕ�
                position_pid(&pid2_flexible.jiaodu_right,pid2_flexible.mechanical_neutral_point-pid2_flexible.sudu_left.output,angle_roll);//�ǶȻ�����������x��Ƕ�
                position_pid(&pid2_flexible.zhuanxiang,pid2_flexible.turn_value,-imu660ra_gyro_z);//ת���󣬷�������z����ٶ�
                 }
            if(ms5==5){
                ms5=0;
                pid2_flexible.turn_value=turn_value;
                incremental_pid_left(&pid2_flexible.jiaosudu_left,pid2_flexible.jiaodu_left.output+pid2_flexible.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                incremental_pid_right(&pid2_flexible.jiaosudu_right,pid2_flexible.jiaodu_left.output-pid2_flexible.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                small_driver_set_duty((int16)pid2_flexible.jiaosudu_left.output,-(int16)pid2_flexible.jiaosudu_right.output);//վ������pid���õ��ת��
                 }
            break;
        }
        //**********************************************************************************************************************
        case stand_c:{//ģʽ0��վ��,����pid
             if(ms20==20){
                 ms20=0;
                 position_pid(&pid0_stand_c.sudu_left,0,(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data)/2);//�ٶȻ��󣬷������������ٶȣ�����С���ٶ�
                 ServoPID.angle_roll=-pid0_stand_c.sudu_left.output;
             }
             if(ms10==10){
                 ms10=0;
                 position_pid(&pid0_stand_c.jiaodu_left,0,angle_roll);//�ǶȻ��󣬷�������x��Ƕ�
              }
             if(ms5==5){
                 ms5=0;
                 position_pid(&pid0_stand_c.jiaosudu_left,pid0_stand_c.jiaodu_left.output,imu660ra_gyro_x);//���ٶȻ�
                 small_driver_set_duty((int16)pid0_stand_c.jiaosudu_left.output,(int16)pid0_stand_c.jiaosudu_left.output);//վ������pid���õ��ת��
              }
             break;
         }
        case stopworking:{
            break;
        }
        //************************************************************************************************************************
        case walk:{//ģʽ1������
             if(ms20==20){
                 ms20=0;
                 /*run_speedǰ���ٶȣ�init.h��*/
                 position_pid(&pid1_walk.sudu_right,pid1_walk.run_speed,-motor_value.receive_right_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                 position_pid(&pid1_walk.sudu_left,pid1_walk.run_speed,motor_value.receive_left_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                 ServoPID.angle_roll=(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2;
             }
             if(ms10==10){
                 ms10=0;
                 pid1_walk.turn_value=turn_value;
                 /*mechanical_neutral_point��е��ֵ��init.h��*/
                 position_pid(&pid1_walk.jiaodu_left,0,angle_roll);//�ǶȻ�����������x��Ƕ�
                 position_pid(&pid1_walk.zhuanxiang,pid1_walk.turn_value,-imu660ra_gyro_z);//ת���󣬷�������z����ٶ�
              }
             if(ms5==5){
                 ms5=0;
                 incremental_pid_left(&pid1_walk.jiaosudu_left,pid1_walk.jiaodu_left.output+pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                 incremental_pid_right(&pid1_walk.jiaosudu_right,pid1_walk.jiaodu_left.output-pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                 small_driver_set_duty((int16)pid1_walk.jiaosudu_left.output,-(int16)pid1_walk.jiaosudu_right.output);//վ������pid���õ��ת��
              }
             /*if((angle_roll>-0.5)&&(angle_roll<0.5)){
                 mode=flexible;//��վ����
             }*/
             break;
        }
        case jump:{//ģʽ0��վ��,����pid
            if(ms20==20){
                 ms20=0;
                 /*run_speedǰ���ٶȣ�init.h��*/
                 position_pid(&pid3_jump.sudu_right,pid3_jump.run_speed,-motor_value.receive_right_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                 position_pid(&pid3_jump.sudu_left,pid3_jump.run_speed,motor_value.receive_left_speed_data);//�ٶȻ����������������ٶȣ�����С���ٶ�
                 ServoPID.angle_roll=(-pid3_jump.sudu_left.output-pid3_jump.sudu_right.output)/2;
             }
             if(ms10==10){
                 ms10=0;
                 /*mechanical_neutral_point��е��ֵ��init.h��*/
                 position_pid(&pid3_jump.jiaodu_left,0,angle_roll);//�ǶȻ�����������x��Ƕ�
                 position_pid(&pid3_jump.zhuanxiang,pid3_jump.turn_value,-imu660ra_gyro_z);//ת���󣬷�������z����ٶ�
              }
             if(ms5==5){
                 ms5=0;
                 pid3_jump.turn_value=turn_value;
                 incremental_pid_left(&pid3_jump.jiaosudu_left,pid3_jump.jiaodu_left.output+pid3_jump.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                 incremental_pid_right(&pid3_jump.jiaosudu_right,pid3_jump.jiaodu_left.output-pid3_jump.zhuanxiang.output,imu660ra_gyro_x);//���ٶȻ�
                 small_driver_set_duty((int16)pid3_jump.jiaosudu_left.output,(int16)pid3_jump.jiaosudu_right.output);//վ������pid���õ��ת��
              }
             break;
        }
    }
    pit_clear_flag(CCU60_CH1);
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    //������Ƽ��㣺
    /*����Ǻ�angle����*/
    //incremental_pid_servo(&ServoPID.Servo,0,angle_roll);
    //ServoPID.angle=-ServoPID.Servo.output;
    //if(angle_roll>1)ServoPID.angle=ServoPID.angle+0.5;
    //if(angle_roll<-1)ServoPID.angle=ServoPID.angle-0.5;
    //LIMIT(ServoPID.angle,-15,15);
    //����
    servo_control_pitch(ServoPID.angle_pitch);
    servo_control_table(ServoPID.highrightre,-ServoPID.angle_roll, &ServoPID.pwm_ph4, &ServoPID.pwm_ph2);
    servo_control_table(ServoPID.highleftre,ServoPID.angle_roll, &ServoPID.pwm_ph1, &ServoPID.pwm_ph3);
    //�޷�������
    if(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1||10000 == ServoPID.pwm_ph2 || 10000 == ServoPID.pwm_ph3)
    {
        zf_assert(10000 == ServoPID.pwm_ph4 || 10000 == ServoPID.pwm_ph1||10000 == ServoPID.pwm_ph2 || 10000 == ServoPID.pwm_ph3);
        pwm_set_duty(SERVO_1, SERVO1_MID);
        pwm_set_duty(SERVO_2, SERVO2_MID);
        pwm_set_duty(SERVO_3, SERVO3_MID);
        pwm_set_duty(SERVO_4, SERVO4_MID);
    }
    //������
    pwm_set_duty(SERVO_1, SERVO1_MID - ServoPID.pwm_ph4);//ch1
    pwm_set_duty(SERVO_2, SERVO2_MID + ServoPID.pwm_ph3);//ch3
    pwm_set_duty(SERVO_3, SERVO3_MID + ServoPID.pwm_ph2);//ch2
    pwm_set_duty(SERVO_4, SERVO4_MID - ServoPID.pwm_ph1);//ch4
    //flexible_control_table(ServoPID.highleftre,ServoPID.highrightre);
    //gpio_set_level (P33_10, (unsigned char)buzzer);


    get_turn_value(20,0.15,200,0);//����Ӧ�÷��ں�1�е�
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 1, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    //ͼ��ƫ�ת��ֵ���㣺����ʱ���ں�0���ȵ�˫�˸�������ں�1���ж��
    //ʧ��


    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF ģ�� INT �����ж�

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler();                           // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                  // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
#endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_uart_handler();                          // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    wireless_module_uart_handler();                 // ����ģ��ͳһ�ص�����



}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    //gnss_uart_callback();                           // GNSS���ڻص�����
    uart_control_callback();                        // ��ˢ���� ���ڽ��ջص�����



}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
