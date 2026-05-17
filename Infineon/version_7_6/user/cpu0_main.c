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
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"
#include "extern.h"
#include "karman.h"
#include "gyro.h"
#include "balance.h"
#include "guandao.h"
#include "rtk.h"
#include "garage.h"
#include "zf_device_lora3a22.h"
#include "zf_device_dot_matrix_screen.h"

#define PIT_NUM                 (CCU60_CH0 )                            // 使用的周期中断编号
#define PIT_period              (10 )                                   // 中断周期
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 英飞凌Tricore 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
//
// 接入凌瞳摄像头 对应主板摄像头接口 请注意线序
//      模块管脚            单片机管脚
//      TXD                 查看 zf_device_scc8660.h 中 SCC8660_COF_UART_TX 宏定义
//      RXD                 查看 zf_device_scc8660.h 中 SCC8660_COF_UART_RX 宏定义
//      PCLK                查看 zf_device_scc8660.h 中 SCC8660_PCLK_PIN 宏定义
//      VSY                 查看 zf_device_scc8660.h 中 SCC8660_VSYNC_PIN 宏定义
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

//      IPS200模块管脚            单片机管脚
//      双排排针 并口两寸屏 硬件引脚
//      RD                  查看 zf_device_ips200.h 中 IPS200_RD_PIN_PARALLEL8     宏定义 B0
//      WR                  查看 zf_device_ips200.h 中 IPS200_WR_PIN_PARALLEL8     宏定义 B1
//      RS                  查看 zf_device_ips200.h 中 IPS200_RS_PIN_PARALLEL8     宏定义 B2
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_PARALLEL8    宏定义 C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_PARALLEL8     宏定义 B3
//      BL                  查看 zf_device_ips200.h 中 IPS200_BL_PIN_PARALLEL8     宏定义 C18
//      D0-D7               查看 zf_device_ips200.h 中 IPS200_Dx_PIN_PARALLEL8     宏定义 B16/B17/B18/B19/D12/D13/D14/D15
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//      单排排针 SPI 两寸屏 硬件引脚
//      SCL                 查看 zf_device_ips200.h 中 IPS200_SCL_PIN_SPI  宏定义  B0
//      SDA                 查看 zf_device_ips200.h 中 IPS200_SDA_PIN_SPI  宏定义  B1
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_SPI  宏定义  B2
//      DC                  查看 zf_device_ips200.h 中 IPS200_DC_PIN_SPI   宏定义  C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_SPI   宏定义  B3
//      BL                  查看 zf_device_ips200.h 中 IPS200_BLk_PIN_SPI  宏定义  C18
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 将核心板插在主板上 插到底 摄像头接在主板的摄像头接口 注意线序
//
// 2.摄像头接在主板的摄像头接口 注意线序IPS 2.0模块插入主板屏幕接口
//
// 3.主板上电 或者核心板链接完毕后上电 核心板按下复位按键
//
// 4.屏幕会显示初始化信息然后显示摄像头图像
//
// 5.识别到黄色方框后屏幕中回自动框选黄色方框
//
// 6.更换框选颜色的方框，需要把要识别的颜色方框显示在屏幕显示的中间，然后按一下REFRESH_TARGET按键，就可以却换识别颜色方框
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

extern float speed_alone;//6000
float speed_now=0;
 float fy;
float fg;
extern int mode;
int tets;
int stop_flag=0;//停车标志位，0启动，1停车
float error_of_CameraOrBalance;
float bmq;
int mode2=0;
extern float angle5;
extern float error_of_camera;
extern int flagpoint;
extern double angle_4;
extern double angle_3;
extern int rtk_current_target;
// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初�?化默认调试串�?
    // 此�?编写用户代码 例�?外�?初�?化代码等
    ips200_init(IPS200_TYPE_PARALLEL8);
    PID_init();
    icm20602_init();
    zero_get_init();
    balanceinit();
//    flash_init();
    keyinit();
    lora3a22_init();
    pit_ms_init(PIT_NUM, PIT_period);                                    // 初始化 CCU6_0_CH0 为周期中断 10ms 周期
    pit_ms_init(CCU60_CH1, 2);


//    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
//
//    // 初始化逐飞助手示波器的结构体
//       seekfree_assistant_oscilloscope_struct oscilloscope_data;
//
//       oscilloscope_data.data[0] = bmq;
//
//       // 设置为4个通道，通道数量最大为8个
//       oscilloscope_data.channel_num = 4;



    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    while (TRUE)
   {


        if(mode==0|1|3|-2)//调试模式
        {
            stop_flag=1;

        }

        if(mode==2|mode==4|mode==-1)
        {
            stop_flag=0;
        }

        scankey();



}
}
extern float fy;

extern float fg;
 float angle_s=0;
 float output=0;
 float speed_pid=0;
 float angle_pid=0;
 float micrae=0;

 float turn_jiaodu=0;
 float turn_sudu=0;
 float pwm_alone=0;
 extern int stop_flag;

 IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
 {
     interrupt_global_enable(0);                     // 开�?���?���?
     pit_clear_flag(CCU60_CH1);
//
//     turn_jiaodu=0;
//     turn_sudu=0;
//
//
    static float Angle_Speed_pid=0;//角速度�?
    static float PWM_Y=0;
//
//
//    static float angle_s=0;
//    static float output=0;
//    static float speed_pid=0;
//    static float angle_pid=0;
//
//
//
//    static float turn_jiaodu=0;
//    static float turn_sudu=0;
     extern float angle_s;
     extern float turn_jiaodu;
     extern float angle_pid;
     extern float output;
     extern float speed_pid;
     extern float turn_sudu;
     extern float micrae;

//
     stop_car();//停车函数
     //ips200_show_int(190, 0,error_of_CameraOrBalance, 3);

     pwm_alone=Gyro_PID(Get_Gyro_Fg(),angle_s);//该值作为独轮pwm输入�?
//     pwm_alone=-700;//确定电机死区
     if(stop_flag==1)
     {
         pwm_alone=0;
     }
     motor_alone(pwm_alone);//-pwm_alone
//
//
     Angle_Speed_pid=Gyro_PIDL(Get_Gyro_Fy(),angle_pid);//角加速度环
     PWM_Y=turn_gryo(icm20602_gyro_z,turn_jiaodu);//转向加速度

     motorAB_set(Angle_Speed_pid-PWM_Y,-Angle_Speed_pid-PWM_Y);



    static unsigned int TIM_COUNT_5ms=0;
    TIM_COUNT_5ms++;
    static unsigned int TIM_COUNT_50ms=0;
    TIM_COUNT_50ms++;
    static unsigned int TIM_COUNT_25ms=0;
    TIM_COUNT_25ms++;




       if(TIM_COUNT_5ms>=5)
                    {
                            TIM_COUNT_5ms=0;

                            //独轮
                            fg=-KalmanFilter(Get_Attitude_Fy(),Get_Gyro_Fg()/65.5);//+zero_alone(speed_alone,-speed_now);//+0.88+0.3-2.2+1.3;//+zero_alone(-speed_now);//;-jueduizhi(micrae/2);
                            //Get_Attitude_Fy()
                           // fg=angle_calc(Get_Attitude_Fy(),Get_Gyro_Fg()/65.5)+0.88;//采用一阶低通滤�? 节省解算时间

                            angle_s=Angle_PID(fg,output,2);



                       //动量
                            if(error_of_CameraOrBalance>0)
                            {
                                micrae=Turn_loop_left(angle_4,speed_now);//压弯函数，后期增加
                            }
                            else
                            {
                                micrae=Turn_loop_right(angle_4,speed_now);//压弯函数，后期增加

                            }



                            fy=-KalmanFilter1(Get_Attitude_Fg(),Get_Gyro_Fy()/65.5)-micrae;
                            angle_pid=flywheel_speedL(fy,speed_pid);
                            if(error_of_camera&&mode==4)//摄像头数据优先
                            {
                                error_of_CameraOrBalance=error_of_camera;
                                error_of_camera=0;
                            }




                            if(fabs(angle_4-error_of_CameraOrBalance)<30)
                            {
                                if(fabs(angle_4-error_of_CameraOrBalance)<5)
                                {
                                    angle_4=error_of_CameraOrBalance;
                                }
                                else if(angle_4>error_of_CameraOrBalance)
                                {
                                    angle_4-=4;
                                }
                                else if(angle_4<error_of_CameraOrBalance)
                                {
                                    angle_4+=4;
                                }
                            }
                            else
                            {
                                if(angle_4>error_of_CameraOrBalance)
                                {
                                    angle_4-=10;
                                }
                                else if(angle_4<error_of_CameraOrBalance)
                                {
                                    angle_4+=10;
                                }
                            }


                            if(angle_4>20)//转向限幅
                                angle_4=20;
                            else if(angle_4<-20)
                                angle_4=-20;

                            turn_jiaodu=turn_angle(angle_4,turn_sudu);



                            // tets=turn_jiaodu;

                            pid.I_zc_speed = encoder_get_count(TIM5_ENCODER)-encoder_get_count(TIM2_ENCODER);

                                                 encoder_clear_count(TIM2_ENCODER);
                                                 encoder_clear_count(TIM5_ENCODER);
                        //    tets=turn_jiaodu;
                    }

                if(TIM_COUNT_50ms>=6)
                    {

                    TIM_COUNT_50ms=0;

                    speed_now= Speed_Measure();
                   // tets=speed_now;
                    bmq = speed_now;
                    speed_control(error_of_CameraOrBalance);//加减速控制
                    output=Speed_PID(-speed_now,speed_alone,-0.6);//小独�?500speed_alone5600  13000  -3500
//                    ips200_show_float(180, 205,output,2,3);

                      }


                if(TIM_COUNT_25ms>=25)
                      {
                      TIM_COUNT_25ms=0;
                        // out_piancha=out_Turn(error_of_CameraOrBalance);

                      speed_pid=fspeedL(pid.I_zc_speed);//动量�?
//                      ips200_show_float(180,245,speed_pid,2,3);

                      //�?���?
                      turn_sudu=turn_speed(pid.I_zc_speed);
//                      ips200_show_float(180,284,turn_sudu,2,3);
                      turn_sudu=0;
                      printf("mode=%d\n",mode);

                        }


                protect(fg,fy);

             }

int point_time=0;

 IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
 {
     interrupt_global_enable(0);                      // 开启中断嵌套
     pit_clear_flag(CCU60_CH0);

     lora3a22_response_time++;
     if (lora3a22_response_time > 500 / PIT_period)   //500ms 没有接受倒数据判断位发送端异常
     {
         lora3a22_state_flag = 0;                     //遥控器状态位清零
         lora3a22_response_time = 0;
     }

               if(mode==2)//2模式打点，自动打点，和手动打点
                     {
                   point_time++;
                   if(point_time > 500 / PIT_period)//0.5秒自动打点
                   {
                       flagpoint=1;
//                       gpio_toggle_level(P33_10);
                       point_time=0;//清零计时器
                   }

                   point_get();

                    }


             //
                     if(mode==3)//3模式清除
                     {
                         erase();//擦除最后一次得到的航向角

                     }
             //
                     if(mode==4)//巡点
                     {

                                 pianjiao_get();
                                 error_of_CameraOrBalance=angle5;
                     }


 }

#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
//
// 问题2：显示 reinit 字样
//      检查接线是否正常
//      主板供电是否使用电量充足的电池供电
//
// 问题2：显示图像杂乱 错位
//      检查摄像头信号线是否有松动
