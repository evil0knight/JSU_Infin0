/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

#define PWM_MIN 1000
#define PWM_MAX 9000

static unsigned char ms20=0;//用于中断，20ms执行一次
static unsigned char ms10=0;//用于中断，10ms执行一次
static unsigned char ms5=0;//用于中断，5ms执行一次
// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。



// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);



}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);// 开启中断嵌套
    ++ms5;
    ++ms10;
    ++ms20;
    EKF_UpData();//卡尔曼滤波更新姿态,获取角度,角速度
    angle_roll=Angle_Converter();//俯仰角角度转化
    //if(mode!=jump){
    //    ServoPID.angle_roll=ServoPID.angle_roll+0.001*angle_roll;
    //   LIMIT(ServoPID.angle_roll,-15,15);
    //}
    //if(mode==flexible&&ServoPID.highleft<5.9){ServoPID.highleft+=0.01;ServoPID.highright+=0.01;}
    
    // 调用跟随式横滚角补偿函数
    follow_pitch_compensation(5);
    wheel_obstacle_height_control(0);
    jump_process();
    switch(mode){
        //*********************************************************************************************************************
        case flexible:{//模式0，站立,并级pid
            if(ms20==20){
                 ms20=0;
                 /*run_speed前进速度，init.h中*/
                 position_pid(&pid1_walk.sudu_right,pid2_flexible.run_speed,-motor_value.receive_right_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 position_pid(&pid1_walk.sudu_left,pid2_flexible.run_speed,motor_value.receive_left_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 //ServoPID.angle_roll=(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2;
             }
             if(ms10==10){
                 ms10=0;
                 pid1_walk.turn_value=turn_value;
                 /*mechanical_neutral_point机械中值，init.h中*/
                 position_pid(&pid1_walk.jiaodu_left,(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2+pid2_flexible.mechanical_neutral_point,angle_roll);//角度环，反馈的是x轴角度
                 position_pid(&pid1_walk.zhuanxiang,pid1_walk.turn_value+140,-imu660ra_gyro_z);//转向环左，反馈的是z轴角速度
              }
             if(ms5==5){
                 ms5=0;
                 incremental_pid_left(&pid1_walk.jiaosudu_left,pid1_walk.jiaodu_left.output+pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 incremental_pid_right(&pid1_walk.jiaosudu_right,pid1_walk.jiaodu_left.output-pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 small_driver_set_duty((int16)pid1_walk.jiaosudu_left.output,-(int16)pid1_walk.jiaosudu_right.output);//站立串级pid设置电机转速
              }
             /*if((angle_roll>-0.5)&&(angle_roll<0.5)){
                 mode=flexible;//先站起来
             }*/
            break;
        }
        case stopworking:{
            ms5=0;
            ms10=0;
            ms20=0;
            break;
        }
        //************************************************************************************************************************
        case walk:{//模式1，行走
             if(ms20==20){
                 ms20=0;
                 /*run_speed前进速度，init.h中*/
                 position_pid(&pid1_walk.sudu_right,pid1_walk.run_speed,-motor_value.receive_right_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 position_pid(&pid1_walk.sudu_left,pid1_walk.run_speed,motor_value.receive_left_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 ServoPID.angle_roll=(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2;
             }
             if(ms10==10){
                 ms10=0;
                 pid1_walk.turn_value=turn_value;
                 /*mechanical_neutral_point机械中值，init.h中*/
                 position_pid(&pid1_walk.jiaodu_left,(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2,angle_roll);//角度环，反馈的是x轴角度
                 position_pid(&pid1_walk.zhuanxiang,pid1_walk.turn_value+140,-imu660ra_gyro_z);//转向环左，反馈的是z轴角速度
              }
             if(ms5==5){
                 ms5=0;
                 incremental_pid_left(&pid1_walk.jiaosudu_left,pid1_walk.jiaodu_left.output+pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 incremental_pid_right(&pid1_walk.jiaosudu_right,pid1_walk.jiaodu_left.output-pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 small_driver_set_duty((int16)pid1_walk.jiaosudu_left.output,-(int16)pid1_walk.jiaosudu_right.output);//站立串级pid设置电机转速
              }
             /*if((angle_roll>-0.5)&&(angle_roll<0.5)){
                 mode=flexible;//先站起来
             }*/
            break;
        }
        case jump:{//模式0，站立,并级pid
            if(ms20==20){
                 ms20=0;
                 /*run_speed前进速度，init.h中*/
                 position_pid(&pid1_walk.sudu_right,pid1_walk.run_speed,-motor_value.receive_right_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 position_pid(&pid1_walk.sudu_left,pid1_walk.run_speed,motor_value.receive_left_speed_data);//速度环，反馈的是左轮速度，代表小车速度
                 //ServoPID.angle_roll=(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2;
             }
             if(ms10==10){
                 ms10=0;
                 pid1_walk.turn_value=turn_value;
                 /*mechanical_neutral_point机械中值，init.h中*/
                 position_pid(&pid1_walk.jiaodu_left,(-pid1_walk.sudu_left.output-pid1_walk.sudu_right.output)/2+pid2_flexible.mechanical_neutral_point,angle_roll);//角度环，反馈的是x轴角度
                 position_pid(&pid1_walk.zhuanxiang,pid1_walk.turn_value+140,-imu660ra_gyro_z);//转向环左，反馈的是z轴角速度
              }
             if(ms5==5){
                 ms5=0;
                 incremental_pid_left(&pid1_walk.jiaosudu_left,pid1_walk.jiaodu_left.output+pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 incremental_pid_right(&pid1_walk.jiaosudu_right,pid1_walk.jiaodu_left.output-pid1_walk.zhuanxiang.output,imu660ra_gyro_x);//角速度环
                 small_driver_set_duty((int16)pid1_walk.jiaosudu_left.output,-(int16)pid1_walk.jiaosudu_right.output);//站立串级pid设置电机转速
              }
             /*if((angle_roll>-0.5)&&(angle_roll<0.5)){
                 mode=flexible;//先站起来
             }*/
             break;
        }
    }
    pit_clear_flag(CCU60_CH1);
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //舵机控制计算：
    /*舵机角和angle叠加*/
    //if(mode==flexible){
        //incremental_pid_servo(&ServoPID.Servo,0,angle_roll);
        //ServoPID.angle=-ServoPID.Servo.output;
        //if(angle_roll>1)ServoPID.angle=ServoPID.angle+0.5;
        //if(angle_roll<-1)ServoPID.angle=ServoPID.angle-0.5;
        //LIMIT(ServoPID.angle,-15,15);
    //}
    servo_control_pitch(ServoPID.angle_pitch);
    servo_control_table(ServoPID.highrightre,-ServoPID.angle_roll, &ServoPID.pwm_ph4, &ServoPID.pwm_ph2);
    servo_control_table(ServoPID.highleftre,ServoPID.angle_roll, &ServoPID.pwm_ph1, &ServoPID.pwm_ph3);
    // 计算最终PWM值并限幅
    uint16 servo1_pwm = SERVO1_MID - ServoPID.pwm_ph4;
    uint16 servo2_pwm = SERVO2_MID + ServoPID.pwm_ph3;
    uint16 servo3_pwm = SERVO3_MID + ServoPID.pwm_ph2;
    uint16 servo4_pwm = SERVO4_MID - ServoPID.pwm_ph1;

    // 对最终PWM值进行限幅（假设PWM范围是0-10000）
    servo1_pwm = (servo1_pwm > PWM_MAX) ? PWM_MAX : (servo1_pwm < PWM_MIN) ? PWM_MIN : servo1_pwm;
    servo2_pwm = (servo2_pwm > PWM_MAX) ? PWM_MAX : (servo2_pwm < PWM_MIN) ? PWM_MIN : servo2_pwm;
    servo3_pwm = (servo3_pwm > PWM_MAX) ? PWM_MAX : (servo3_pwm < PWM_MIN) ? PWM_MIN : servo3_pwm;
    servo4_pwm = (servo4_pwm > PWM_MAX) ? PWM_MAX : (servo4_pwm < PWM_MIN) ? PWM_MIN : servo4_pwm;

    //更新腿
    pwm_set_duty(SERVO_1, servo1_pwm);//ch1
    pwm_set_duty(SERVO_2, servo2_pwm);//ch3
    pwm_set_duty(SERVO_3, servo3_pwm);//ch2
    pwm_set_duty(SERVO_4, servo4_pwm);//ch4
    // ...existing code...
    //flexible_control_table(ServoPID.highleftre,ServoPID.highrightre);
    //gpio_set_level (P33_10, (unsigned char)buzzer)
    get_turn_value(70,0.5,250,0);//本来应该放在核1中的
    if(delayt||jump_position_flag == 1){//跳跃后用
        ++delayt;
    }
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(key_pit_flag){
    if(!gpio_get_level (button_down))
    {
        place_index -= 1;
    }
    else if(!gpio_get_level (button_up))
    {
        place_index+=1;
    }
    else if(!gpio_get_level (button_next))
    {
        value_index+=1;
    }
    else if(!gpio_get_level (button_back))
    {
        value_index-=1;
    }
    pit_clear_flag(CCU61_CH1);


    }
    




}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //gnss_uart_callback();                           // GNSS串口回调函数
    uart_control_callback();                        // 无刷驱动 串口接收回调函数



}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
