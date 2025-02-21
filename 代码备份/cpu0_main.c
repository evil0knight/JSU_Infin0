/*
 * 作者：眼膏
 * */
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"
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
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    imu660ra_init();//陀螺仪初始化,陀螺仪算角速度，卡尔曼滤波后算角度
    EKF_Init();//卡尔曼初始化
    small_driver_uart_init();//无刷驱动 参数初始化，给速度
    pid_init_all();
    //servo_init();
    mode=stopworking;
    ips200_init                     (IPS200_TYPE_SPI);

    //seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);//
    //wireless_uart_init();
    pit_ms_init(CCU60_CH1,1);//中断设置初始化
    //pit_ms_init(CCU61_CH0,20);//中断设置初始化
    system_start();

    system_image_time = system_getval_ns();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        show_pid_information();//展示信息
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
