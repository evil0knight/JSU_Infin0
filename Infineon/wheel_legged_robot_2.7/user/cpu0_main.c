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
* 文件名称          cpu0_main
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
#include "AAA_Blessed_by_the_Buddha_there_will_never_be_any_bugs_AAA.h"
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
#include <Kalman_fusion_of_imu963ra.h>
#include <Base.h>
#include <interactive_interface.h>
#include "wifi_lineless.h"

//extern imu963ra_struct imu;
// **************************** 代码区域 ****************************
float fuck = 0;
int type_flag = 0;
int type_count = 0;

int core0_main(void)
{
//                   _ooOoo_
//                  o8888888o
//                  88" . "88
//                  (| -_- |)
//                  O\  =  /O
//               ____/`---'\____
//             .'  \\|     |//  `.
//            /  \\|||  :  |||//  \
//           /  _||||| -:- |||||-  \
//           |   | \\\  -  /// |   |
//           | \_|  ''\---/''  |   |
//           \  .-\__  `-`  ___/-. /
//         ___`. .'  /--.--\  `. . __
//      ."" '<  `.___\_<|>_/___.'  >'"".
//     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//     \  \ `-.   \_ __\ /__ _/   .-` /  /
//======`-.____`-.___\_____/___.-`____.-'======
//                   `=---='
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//            佛祖保佑       永无BUG
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
//    bluetooth_ch9141_init();

    system_start();
    logger_init();
    menu_init();
//    ips200_init(IPS200_TYPE_SPI);
    imu963ra_kalman_filter_init(&imu, 1, 50, pit_time0_ms / 1000.0f);
    imu963ra_menc15a_kalman_filter_init(&vel_kf, 0.01, 0.1, 0.1, pit_time0_ms / 1000.0f);


    Base_init(14.5f,0.0f);
    system_delay_ms(10);


    mt9v03x_init();
//    wifi_init();
    tracking_init();

    system_delay_ms(10);
    pit_ms_init(CCU60_CH0, pit_time0_ms);
//    pit_ms_init(CCU60_CH1, pit_time1_ms);
    pit_ms_init(CCU61_CH0, pit_time2_ms);
    pit_us_init(CCU61_CH1, pit_time3_us);

    pwm_init(ATOM0_CH6_P20_0,3000,0);
    system_delay_ms(10);


    interactive_interface_init();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
//    LQR_K_update(&LS_STRAIGHT_Q, &LS_STRAIGHT_R, &LS_STRAIGHT_K);
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

//        gpio_init (P20_0, GPO, 1,GPO_PUSH_PULL);
//        gpio_set_level(P20_0,0);
//        printf("555\n");
         if(input.flag == 1 || image)
         {
//             printf("666\n");
             Atimer_start(1);
             if (mt9v03x_finish_flag) {
    //            memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    //            seekfree_assistant_camera_send();
//                 printf("666\n");
                memcpy(mt9v03x_image_copy[0], mt9v03x_image[0], (sizeof(mt9v03x_image_copy) / sizeof(uint8_t)));
//                adaptive_threshold_integral(IMAGE_H,188,mt9v03x_image_mirror_copy,mt9v03x_image_copy,15,0);
                mt9v03x_finish_flag = 0;
                state_type = COMMON_STATE;
                //            //GetImgBin_Average(mt9v03x_image_copy[0], &FIX_BINTHRESHOLD);//平均值法
                img_processing();
                get_corners();
                tracking_line();
                ElementJudge();
                aim_distance_select();
                ElementRun();
                MidLineTrack();

                Atimer_stop(1);
                image = 2;
//                ips200_show_gray_image(0,y_pixel*9, mt9v03x_image_copy[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, FIX_BINTHRESHOLD);
             }

         }
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    interactive_interface_run();


    base_get_speed();
    imu963ra_kalman_filter_update(&imu);
    imu963ra_menc15a_kalman_filter_Update(&vel_kf, Instantaneous_speed, imu.ay_linear);
//    Instantaneous_speed = vel_kf.est_velocity;

//    uint32 Ts = system_getval_us();
    Base_run();
//    printf("time: %lu us\n", system_getval_us() - Ts);
    steering_engine_update();
//    steering_engine_update();
}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    if(circle_type == CIRCLE_LEFT_BEGIN||circle_type == CIRCLE_RIGHT_BEGIN||circle_type == CIRCLE_LEFT_IN||circle_type == CIRCLE_RIGHT_IN||cross_type == CROSS_IN||cross_type == CROSS_BEGIN)
    {
        type_flag = 1;
    }
    if(type_flag == 1)
    {
        type_count++;
        if(type_count>10)
        {
            if(circle_type == CIRCLE_LEFT_BEGIN||circle_type == CIRCLE_RIGHT_BEGIN||circle_type == CIRCLE_LEFT_IN||circle_type == CIRCLE_RIGHT_IN||cross_type == CROSS_IN||cross_type == CROSS_BEGIN)
            {
                circle_type = CIRCLE_NONE;
                cross_type == CROSS_NONE;
                type_flag = 0;
                type_count = 0;

                beel = 3000;
            }
            else
            {
                type_flag = 0;
                type_count = 0;
            }
        }
    }


}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);

    pwm_set_duty(ATOM0_CH6_P20_0,beel);
    beel = 0;

}
IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);

    brushless_motor_update();



}
#pragma section all restore
// **************************** 代码区域 ****************************

