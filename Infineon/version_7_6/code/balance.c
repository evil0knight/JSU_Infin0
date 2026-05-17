 /*
 * balance.c
 *
 *  Created on: 2022闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狅拷?12闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狅拷?7闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狅拷?
 *      Author: 11150
 */
//#include "zf_driver_encoder.h"
#include "balance.h"
//#include "math.h"
//#include "zf_driver_pwm.h"
//#include"zf_driver_encoder.h"
//#include "zf_device_icm20602.h"
//#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"


/* 这段代码的功能是配置电机控制相关的 GPIO 和 PWM 输出通道，
 * 同时初始化多个编码器来检测电机的转动。通过调整 PWM 占空比，
 * 电机的速度可以被控制，而编码器则提供实时的电机旋转信息。*/
void balanceinit(void)
{


    //电机控制
    gpio_init(P11_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);//初始化 GPIO 引脚 P11_2，设置为推挽输出，初始电平为高（用于控制电机）。
    //pwm_init(ATOM0_CH2_P21_4, 50, 1000);
    //  gpio_init(P21_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(ATOM0_CH5_P02_5, 12500, 0);//pwm 17000，初始化 PWM 输出通道，设定频率为 12,500 Hz，初始占空比为 0（即没有输出）
    // pwm_init(ATOM0_CH4_P02_4, 50, 1000);
    gpio_init(P02_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);//：初始化 GPIO 引脚 P02_4，设置为推挽输出，初始电平为高（可能用于电机控制）




    //pwm_init( ATOM1_CH0_P21_2, 12500, 0);//pwm
    //pwm_init( ATOM1_CH1_P21_3, 12500, 0);//pwm


    gpio_init(P33_10, GPO, GPIO_HIGH, GPO_PUSH_PULL);//初始化 GPIO 引脚 P33_10，设置为推挽输出，初始电平为高。
    gpio_set_level(P33_10, 0);//将 P33_10 引脚的电平设置为低（可能用于某种状态控制）。




    //闂佸搫鍟版慨鎾春濞戙垺鍋ㄩ柛顭戝亝缁拷2
    gpio_init(P11_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    //pwm_init(ATOM0_CH5_P02_5, 50, 1000);
                   //gpio_init(P02_5, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(ATOM0_CH7_P02_7, 12500, 0);//pwm，17000初始化另一个 PWM 输出通道，频率为 12,500 Hz，初始占空比为 0。
  //  pwm_init(ATOM0_CH7_P02_7, 50, 1000);
    gpio_init(P02_6, GPO, GPIO_HIGH, GPO_PUSH_PULL);


    //初始化一个四相编码器，用于获取电机的旋转信息。编码器连接到定时器 TIM2，且使用 P33_7 和 P33_6 引脚。
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);

  //初始化另一个编码器，连接到定时器 TIM5，使用 P10_3 和 P10_1 引脚。
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);

    //婵＄偟鎳撳畷顒佹叏閳哄懏鏅搁柨鐕傛嫹??闂佽法鍣﹂幏锟�??

   gpio_init(P21_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠囧弶顥濋梺闈涚墕濡挳骞忛敓锟�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎厽鏅搁柨鐕傛嫹??
   pwm_init(ATOM0_CH3_P21_5, 12500, 0);//pwm
 //  pwm_init(ATOM0_CH1_P21_3, 12500, 0);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠鸿櫣绛�?m
   encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);//初始化另一个编码器，连接到定时器 TIM4，使用 P02_8 和 P00_9 引脚。




   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠囧弶顥濋梺闈涚墕濡挳骞忛敓锟�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻缂佹ɑ娅�?
  //  encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
 //    pwm_init(ATOM0_CH0_P21_2, 50, 0);
   //  pwm_init(ATOM0_CH1_P21_3, 50, 0);
    //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠鸿櫣绛�?m闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘???闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狅拷?



}

extern float pwm_alone;
extern float GyroP;////0.52;//2;//1; //
extern float GyroI;
extern int stop_flag;
//void protect(float fy,float fg)
//{
//    if(fy>55||fg>55||fy<-55||fg<-55)
//    {
//        mode=3;
//    }
//}
//int speed1,speed2;
//void jiema(void)
//{


   // speed1 = encoder_get_count(TIM4_ENCODER);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梺璺ㄥ櫐閹凤拷???1
   // speed2 = encoder_get_count(TIM5_ENCODER);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梺璺ㄥ櫐閹凤拷???2

   // encoder_clear_count(TIM4_ENCODER);
  //  encoder_clear_count(TIM5_ENCODER);
//}






//int Incremental_PI (int Encoder,int Target)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰杈仧闁瑰嚖鎷�?閻庢鍠栫敮锟�
//{
 //  float Kp=20,Ki=30;
  //   static int Bias,Pwm,Last_bias;
   //  Bias=Encoder-Target;                //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш鐎规洘鍨肩粻娑樷槈濞嗘劖顏熼梻浣告惈閸婂爼宕愰弽顓熸櫢闁跨噦鎷�??
   //  Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰杈仧闁瑰嚖鎷�?閻庣偣鍊栭幑銏ゆ⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佺粯鍨兼慨銈夊疾閹间焦鐓涢柛灞久敓鑺ョ墱閿熻姤鑹剧紞濠囧蓟閵娾晜鍋勯柛娑橈功娴煎嫰鏌ㄩ悤鍌涘???
  //   Last_bias=Bias;                       //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣告贡閺屽銆掗崷顓犲崥闁圭虎鍠楅悡銉╂煟閺傛寧鍟為柣蹇ｅ櫍閺岀喎鐣￠弶鎸幮╂繛瀛樼矋閹倿寮婚妸鈺傚亜闁告稑锕︽导鍕煥閻曞倹瀚�???
  //   return Pwm;                         //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻缂佹ɑ娅�?
//}



//int Incremental_PI1 (int Encoder1,int Target1)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰杈仧闁瑰嚖鎷�?閻庢鍠栫敮锟�
//{
  // float Kp1=20,Ki1=30;
   //  static int Bias1,Pwm1,Last_bias1;
    // Bias1=Encoder1-Target1;                //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш鐎规洘鍨肩粻娑樷槈濞嗘劖顏熼梻浣告惈閸婂爼宕愰弽顓熸櫢闁跨噦鎷�??
    // Pwm1+=Kp1*(Bias1-Last_bias1)+Ki1*Bias1;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰杈仧闁瑰嚖鎷�?閻庣偣鍊栭幑銏ゆ⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佺粯鍨兼慨銈夊疾閹间焦鐓涢柛灞久敓鑺ョ墱閿熻姤鑹剧紞濠囧蓟閵娾晜鍋勯柛娑橈功娴煎嫰鏌ㄩ悤鍌涘???
    // Last_bias1=Bias1;                       //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣告贡閺屽銆掗崷顓犲崥闁圭虎鍠楅悡銉╂煟閺傛寧鍟為柣蹇ｅ櫍閺岀喎鐣￠弶鎸幮╂繛瀛樼矋閹倿寮婚妸鈺傚亜闁告稑锕︽导鍕煥閻曞倹瀚�???
   //  return Pwm1;                         //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻缂佹ɑ娅�?
//}





//void dongliang(void)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻閼搁潧浠掗梺璺ㄥ櫐閹凤拷???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼捄铏圭瓘?m

//{
   // pwm_set_duty(ATOM0_CH3_P21_5, Incremental_PI (speed1,5000)); //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梺璺ㄥ櫐閹凤拷???1
   // pwm_set_duty(ATOM0_CH6_P02_6, Incremental_PI1 (speed2,5000)); //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梺璺ㄥ櫐閹凤拷???2

//}





//澶勭悊鐢垫満姝诲尯闂
int moter_dead_area=0;



float SpeedP=150;//105 -17//-4;//-85;//3.5  127  155速度比例P
float SpeedI=59;//100 -5;//100  55  100速度积分I
//这是角度控制器的比例增益参数，用于调整控制系统对角度误差的反应程度。值越大，系统对角度误差的反应越强。
float AngleP=-51;//-100//-410;//-80;//-80;//-20;//-80;//-100 -80;//-80;//30;//60;角度比例P
float AngleI=0;//积分增益，用于累计过去的角度误差，防止长期的偏差积累。
float AngleD=-101;//微分增益，用于计算角度误差变化的速率，帮助减少震荡和过冲。
/*
 * Angle_Data:实际的角度数据，可能来自陀螺仪或其他传感器。
 * Angle_m:期望角度，目标角度.
 * balance_angle：平衡角度，可能是一个静态或动态的平衡点。
 */
float Angle_PID(float Angle_Data,float Angle_m,float balance_angle)//角度误差
{      static float Angle_Error_Least=0;
       static  float Angle_Error=0,Realize=0;


         Angle_Error = Angle_m - Angle_Data - balance_angle;//计算期望角度和实际角度之间的误差，再减去一个平衡角度
         /*
          * AngleP * Angle_Error：比例项，控制器根据当前误差来调整输出，误差越大，输出越大。
          * AngleD * (Angle_Error - Angle_Error_Least)：微分项，计算误差变化的速率，从而调整输出，减少过冲或震荡。
          * */
         Realize = AngleP * Angle_Error + AngleD * (Angle_Error - Angle_Error_Least);


         Angle_Error_Least = Angle_Error;

        if(Realize>8000)//为了防止输出过大或过小，控制量被限制在 -8000 到 8000 之间。这个范围可能是硬件可接受的控制信号的范围。
                   Realize=8000;
               if(Realize<-8000)
                   Realize=-8000;
         return Realize;//函数最终返回经过 PID 计算后的控制量，通常用于调节电机或其他执行器。
}

float GyroP=-83;//1;//0.52;//2;//1; //54 0.8陀螺仪比例P
float GyroI=-78;//7.5;//6;//7.5陀螺仪积分I
float GyroD=0;
/*
 * Gyro_Data：当前的陀螺仪读取值，代表系统的实际角速度。
 * Gyro_m：期望的陀螺仪数据，代表目标角速度。
 * */
/* 陀螺仪（Gyroscope）数据的 PID 控制器，用于处理和调整陀螺仪的输出值。
 * 代码中定义了一个函数 Gyro_PID，用来根据当前陀螺仪数据与目标值之间的误差，
 * 计算并输出一个控制量。以下是对代码的详细解释：*/
/*
 * Realize：保存当前的控制量，代表 PID 控制器的输出结果。
 * Gyro_Error：当前的角速度误差（目标值与实际值之差）。
 * last_gyro_error：上一个时刻的角速度误差，用于计算微分项。
 * */
float Gyro_PID(float Gyro_Data,float Gyro_m)
{  static  float Realize=0,Gyro_Error=0,last_gyro_error=0;
           //Gyro_Data=0.75*Gyro_Data+0.25*LAST_Gyro_Data;

          // if(Gyro_Data>-20&&Gyro_Data<20)
         //  Gyro_Data=0;
        // LAST_Gyro_Data=Gyro_Data;
        /*
         * 计算期望角速度与实际角速度之间的误差，Gyro_m 是目标角速度，Gyro_Data 是当前实际角速度。
         * */
         Gyro_Error = Gyro_m - Gyro_Data;
     /*
      * 比例项（P）：GyroP / 100 * (Gyro_Error - last_gyro_error)。此项根据当前误差与上一时刻误差的差值（误差变化量）来调整输出。
      * 这个差值可以帮助控制系统更快速地响应误差变化。
      * 积分项（I）：GyroI / 1000.0 * Gyro_Error。此项用于消除长期的积累误差，防止系统长期偏离目标。它根据误差的累计来调整控制量。
      * */
     Realize += GyroP/100*(Gyro_Error-last_gyro_error)+GyroI/1000.0 * Gyro_Error;

           last_gyro_error=Gyro_Error;
        /*控制量 Realize 被限制在 -3333 到 3333 之间。这个范围可能是硬件的限制，防止输出值过大或过小，
         *从而导致系统过于激烈的响应或无法响应。*/
       if(Realize>3333)
           Realize=3333;
           if(Realize<-3333)
           Realize=-3333;

         /*Realize 存储了基于 PID 计算的控制量，用于调整执行器（如电机或其他硬件）。*/
         return Realize;


}
/*
 * Speed_Error_Integral：积累的速度误差，用于积分项（I），目的是消除系统中长期存在的偏差。
 *  Speed_Error：当前的速度误差，即目标速度 Speed_m 和当前速度 Speed_New 之间的差值。
 *  Realize：最终计算的控制量，代表 PID 控制器的输出，控制器的目标是将 Realize 调整到合理范围内。
 *  last_error：上一次的速度误差，用于计算积分项时的加权。
 * */
float Speed_PID(float Speed_New,float Speed_m,int speeed_low)//速度环
{ static float Speed_Error_Integral=0;
    static  float Speed_Error=0,Realize=0,last_error=0;
      Speed_Error =  Speed_m-Speed_New;//速度误差是期望速度和当前速度之间的差值。这个误差决定了系统当前的偏差
      /*
       * Speed_Error * 0.7：当前误差的 70%。
       * 0.3 * last_error：上次误差的 30%，帮助平滑积分项，避免积分爆炸（过度累积误差）。
       * */
        Speed_Error_Integral+= Speed_Error*0.7+0.3*last_error;//积分项用于消除系统中的长期偏差（积累误差）。积分项是通过当前误差和上次误差的加权和来计算的：
        /*为了防止积分项累积过多，导致控制量过大或不稳定，Speed_Error_Integral 被限制在 -2000 到 2000 的范围内。*/
    if(Speed_Error_Integral>=2000) Speed_Error_Integral=2000;
    if(Speed_Error_Integral<=-2000) Speed_Error_Integral=-2000;
    /*
     * 比例项（P）：SpeedP * Speed_Error / 5000，根据当前的速度误差调整控制量，SpeedP 是比例增益。
     *  积分项（I）：Speed_Error_Integral / 100000 * SpeedI，根据累积的速度误差调整控制量，SpeedI 是积分增益。
     * */
        Realize = SpeedP * Speed_Error/5000 + Speed_Error_Integral/100000* SpeedI;
                        last_error=Speed_Error;
     // ips200_showint16(65,2,Realize);//85
  if(Realize>40) Realize=40;//输出限幅
     if(Realize<-40) Realize=-40;
//ips200_showint16(65,2,Realize);
//     ips200_show_float(130, 275,Realize,2,2);
      return Realize;
}


float Speed_Measure()//速度测量
{
        static float Speed_New=0;
        /*
         * encoder_get_count(TIM4_ENCODER)：这段代码读取连接到 TIM4_ENCODER 编码器的计数值。编码器通常用来测量旋转的角度或位置，
         * 它会根据电机或轮子的旋转输出计数值。每次电机转动一定角度，编码器计数值增加。
         * */
        Speed_New = encoder_get_count(TIM4_ENCODER);
        /*
         * encoder_clear_count(TIM4_ENCODER)：这段代码会将编码器的计数器清零，准备下一次测量。
         * 这样做是为了确保下次读取的计数值从零开始，避免重复计算历史数据，准确测量新的速度变化
         * */
        encoder_clear_count(TIM4_ENCODER);
        return Speed_New;//当前速度
}










//基于陀螺仪计算角度
/*Target_Angle：目标角度（可能是所需角度或参考角度）。
 *alpha：一些校准或偏移值，可能与传感器读数有关。
 *mpu_gyro_y：陀螺仪沿 Y 轴的角速度（通常以度/秒或弧度/秒为单位）。
 * */
float Angle_Calculate(float Target_Angle,float alpha,float mpu_gyro_y)     //闁荤喐鐟︾敮鎺旓拷瑙勫▕閺佹捇鏁撻敓锟�??
{
    /*
     * Angle_Biasalpha：与的差值Target_Angle，表示目标与当前角度的误差。
     * Angle：将计算的输出角度。
     * Integral_Angle_Bias：此变量是静态的，这意味着它在函数调用之间保留其值。它用于随时间累积角度偏差。
     * */
float Angle_Bias=0,Angle=0;
static float Integral_Angle_Bias=0;
//是Angle_Bias通过从Target_Angle中减去 来计算的。从 中(alpha - 0.5)减去可以用作偏移或校准因子。0.5alpha
Angle_Bias=(alpha-0.5)-Target_Angle;


/*
 * Angle_Bias随着时间的推移，会累积到，Integral_Angle_Bias然后将其夹在-20和之间，20以防止积分饱和（累积误差变得太大）。
 * */
Integral_Angle_Bias += Angle_Bias;

if(Integral_Angle_Bias>20) Integral_Angle_Bias=20;
if(Integral_Angle_Bias<-20) Integral_Angle_Bias=-20;


Angle=AngleP*Angle_Bias + AngleI*Integral_Angle_Bias +AngleD * (-mpu_gyro_y);
// AngleP * Angle_Bias: 比例项，表示当前角度和目标角度之间的直接误差。
// AngleI * Integral_Angle_Bias: 积分项。补偿随时间累积的误差。
// AngleD * (-mpu_gyro_y): 导数项。通过考虑角速度（角度变化率）来帮助平滑系统，从而防止振荡和过冲。


return Angle;
}


/*
 * Target_AngVelocity：这是您想要达到的期望或目标角速度（根据具体情况可以是每秒度数或每秒弧度）。
 * mpu_gyro_y：这是陀螺仪当前的角速度读数（大概以度/秒或弧度/秒为单位）。
 * */
float AngVelocity_Calculate(float Target_AngVelocity,float mpu_gyro_y)     //定义的角速度Target_AngVelocity。它利用来自陀螺仪的输入（由 表示mpu_gyro_y）来计算实现目标角速度所需的调整。
{
static float Last_AngVelocity_Bias=0;//AngVelocity_Bias的先前值，用于PID控制中的微分计算。
/*
 * AngVelocity_Bias。纠正随时间推移而累积的任何持续性错误
 * AngVelocity：基于PID控制算法计算的角速度调整。
 * */
float AngVelocity_Bias=0,AngVelocity=0;
static float Integral_AngVelocity_Bias=0;//Integral_AngVelocity_Bias：随时间推移的累积（积分）



AngVelocity_Bias = mpu_gyro_y-Target_AngVelocity;//计算当前角速度（mpu_gyro_y）和期望角速度（Target_AngVelocity）之间的误差或偏差。

Integral_AngVelocity_Bias += AngVelocity_Bias;//偏差（AngVelocity_Bias）被添加到累加器（Integral_AngVelocity_Bias），这有助于纠正累积或持续的错误。
//这将积分限制为最大值 30 和最小值 -30，防止积分项变得过大，从而导致过度校正。
if(Integral_AngVelocity_Bias>30) Integral_AngVelocity_Bias=30;
if(Integral_AngVelocity_Bias<-30) Integral_AngVelocity_Bias=-30;
/*PID
 * 比例项（GyroP * AngVelocity_Bias）根据当前误差进行调整。
 * 积分项（GyroI * Integral_AngVelocity_Bias）补偿过去累积的误差。
 * 导数项（GyroD * (AngVelocity_Bias - Last_AngVelocity_Bias)）表示误差变化的速度，提供阻尼以防止振荡。
 */
AngVelocity=GyroP*AngVelocity_Bias + GyroI*Integral_AngVelocity_Bias +GyroD * (AngVelocity_Bias-Last_AngVelocity_Bias);
Last_AngVelocity_Bias = AngVelocity_Bias;

/*AngVelocity基于PID控制算法计算的角速度调整。*/
return AngVelocity;
}


/*encoder编码器当前读数，旋转位置变化*/
float Velocity_Calculate(int encoder)      //根据编码器计算速度
{/*
Velocity：最终计算速度，这是根据编码器读数和控制条件调整的速度。
Encoder：该变量表示编码器位置的运行值，由环路中的控制逻辑进行调整。
Encoder_Integral：这是用于随时间累积编码器值的积分项，可用于补偿持续性错误。
Encoder_Least：这是一个临时变量，用于在进行调整之前保存当前编码器的值。
*/
static float Encoder=0,Encoder_Integral=0;
float Velocity=0,Encoder_Least=0;

/*当前编码器读数 (encoder) 首先存储在Encoder_Least中，并将Encoder的值按 0.7 的倍数缩小。
 * 这意味着系统在下一次计算中仅考虑当前编码器读数的 70%，这有助于减少噪音或防止过度调整。*/
Encoder_Least = encoder*1.0;
Encoder *= 0.7;
/*此行通过将Encoder的值的30% 添加到Encoder_Least（原始编码器读数）的当前值来更新Encoder。
 * 当前和先前编码器读数的加权组合有助于随着时间的推移平滑编码器输入。
 */
Encoder += Encoder_Least*0.3;
/*随着时间的推移，累积Encoder_Integral调整后的编码器读数。这是一个积分项，有助于消除稳态误差，类似于 PID 控制器中积分项的工作原理。*/
Encoder_Integral += Encoder;
if(Encoder_Integral > +2000) Encoder_Integral = +2000;//限幅
/*
 * 比例项：编码器值 ( Encoder) 乘以比例常数 ( SpeedP)，然后从速度中减去该值。这会根据编码器的当前位置（与误差成比例）调整速度。
 * 积分项：累积编码器值 ( Encoder_Integral) 乘以积分常数 ( SpeedI)，然后从速度中减去该值。这会根据过去的编码器读数调整速度，以纠正随时间推移而产生的持续错误。
 * 负号（-SpeedP, -SpeedI）表示随着编码器值的增加，速度将会降低，这可能表明系统需要随着编码器位置的增加而降低速度。
 * */
//夹紧Velocity？
Velocity = Encoder * -SpeedP + Encoder_Integral * -SpeedI;


 return Velocity;
}
//----------------------------------------------------------------------------------------------------
//int moter_dead_area=600;这表示 PWM 值为零附近的“死区”。电机不会对pwm_alone处于此范围内的微小变化做出反应，
//从而降低噪音或不稳定运动。此值可能补偿电机控制系统中的小传感器误差或缺陷。

/*
 * 通过PWM控制电机，根据输入的pwm_alone调制pwm信号（电机所需速度或功率水平）
 * */
void motor_alone(float pwm_alone)
{


    if(pwm_alone>0)
     {
        pwm_alone += moter_dead_area;//如果pwm_alone为正，则首先将死区（moter_dead_area）添加到 PWM 值，以确保不会将零附近的非常小的 PWM 值应用于电机。
        if(pwm_alone>3333)pwm_alone=3333;//然后将 PWM 值限制为最大值 3333，以确保电机不会接收可能导致损坏或不稳定的过高占空比。
        gpio_set_level (P21_4,1);//控制电机启用方向，P21_4高电平，电机正转。
        pwm_set_duty(ATOM0_CH3_P21_5,pwm_alone);//控制电机占空比为pwm_alone，占空比越高，电机速度越高

     }
    else if(pwm_alone<0)
         {
        pwm_alone-= moter_dead_area;
        if(pwm_alone<-3333)
               pwm_alone=-3333;
            gpio_set_level (P21_4,0);
            pwm_set_duty(ATOM0_CH3_P21_5, -pwm_alone);//P21_4电机反向运行

         }
    else if(pwm_alone==0)
             {

                pwm_alone=0;
                gpio_set_level (P21_4,0);
                pwm_set_duty(ATOM0_CH3_P21_5, pwm_alone);//电机停转

             }
}

_pid_typedef pid;//结构体pid
void PID_init()
{
    /*陀螺仪控制环路的比例增益和积分增益*/
    pid.fgyrokp=-6;//-8
    pid.fgyroki=-131;//-200
    /*/*飞轮
    pid.flywheelkp=-4;//-5
    pid.flywheelkd=-29;//-59*/

    /*飞轮
    pid.flymotorKp=86;//13
    pid.flymotorKi=70;*/

  pid.F_zc_Tzero=0.5;//过滤噪音


  pid.I_zc_speed=0;//累积与速度相关的误差的积分速度值


  pid.I_zc_output=0;//这可以表示与积分控制相关的输出，存储特定控制回路的累积误差。

}


//闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹??
float turn_kp=-18;//-1;//-0.7;//-30;//-10//7
float turn_kd=-3.14;//-0.1;


float turn_kp2=40;//150
float turn_kd2=3;


//float turn_kp=-10;//-1;//-0.7;//-30;//-10//7
//float turn_kd=0;//-0.1;
//
//
//float turn_kp2=40;//150
//float turn_kd2=3;


//float turn_kp3=-20;//21
float turn_kp3=0;//21

float turn_ki3=0;
float control_speed=0;
void keyinit(void)//gpio 初始化，详情看open declaration
{
gpio_init(P20_6, GPI, 1, GPI_PULL_UP);
gpio_init(P20_7, GPI, 1, GPI_PULL_UP);
gpio_init(P20_8, GPI, 1, GPI_PULL_UP);
gpio_init(P33_11, GPI, 1, GPI_PULL_UP);
}

extern float speed_alone;
extern float straight_speed;
extern float other_speed;
int mode=-2;//
int i=13;
/*void write_flash(int index, float value) {
    flash_union_buffer[index].float_type = value;
    flash_erase_page(0, 5); // 擦除第5页
    flash_write_page_from_buffer(0, 5); // 写入数据
}

void handle_button(int button, float *param, int index, float step) {
    if (gpio_get_level(button) == 0) {
        system_delay_ms(500); // 消抖
        if (gpio_get_level(button) == 0) {
            *param += step;
            write_flash(index, *param);
        }
    }
}

void scankey() {
    // 模式切换
    if (gpio_get_level(P33_11) == 0) {
        system_delay_ms(500);
        if (gpio_get_level(P33_11) == 0) {
            i++;
            if (i == 19) i = 0;
        }
    }

    // 调节参数（通过P20_6和P20_7）
    switch(i) {
        case 0:
            handle_button(P20_6, &pid.F_zc_Tzero, 17, 0.1);
            handle_button(P20_7, &pid.F_zc_Tzero, 17, -0.1);
            break;
        case 1:
            handle_button(P20_6, &pid.fgyrokp, 0, 1);
            handle_button(P20_7, &pid.fgyrokp, 0, -1);
            break;
        case 2:
            handle_button(P20_6, &pid.fgyroki, 1, 1);
            handle_button(P20_7, &pid.fgyroki, 1, -1);
            break;
        case 3:
            handle_button(P20_6, &pid.flywheelkp, 2, 1);
            handle_button(P20_7, &pid.flywheelkp, 2, -1);
            break;
        case 4:
            handle_button(P20_6, &pid.flywheelkd, 3, 1);
            handle_button(P20_7, &pid.flywheelkd, 3, -1);
            break;
        case 5:
            handle_button(P20_6, &pid.flymotorKp, 4, 1);
            handle_button(P20_7, &pid.flymotorKp, 4, -1);
            break;
        case 6:
            handle_button(P20_6, &pid.flymotorKi, 5, 1);
            handle_button(P20_7, &pid.flymotorKi, 5, -1);
            break;
        case 7:
            handle_button(P20_6, &GyroP, 6, 1);
            handle_button(P20_7, &GyroP, 6, -1);
            break;
        case 8:
            handle_button(P20_6, &GyroI, 7, 1);
            handle_button(P20_7, &GyroI, 7, -1);
            break;
        case 9:
            handle_button(P20_6, &AngleP, 8, 1);
            handle_button(P20_7, &AngleP, 8, -1);
            break;
        case 10:
            handle_button(P20_6, &AngleD, 9, 1);
            handle_button(P20_7, &AngleD, 9, -1);
            break;
        case 11:
            handle_button(P20_6, &SpeedP, 10, 1);
            handle_button(P20_7, &SpeedP, 10, -1);
            break;
        case 12:
            handle_button(P20_6, &SpeedI, 11, 1);
            handle_button(P20_7, &SpeedI, 11, -1);
            break;
        case 13:
            handle_button(P20_6, &turn_kp, 12, 1);
            handle_button(P20_7, &turn_kp, 12, -1);
            break;
        case 14:
            handle_button(P20_6, &turn_kd, 13, 1);
            handle_button(P20_7, &turn_kd, 13, -1);
            break;
        case 15:
            handle_button(P20_6, &turn_kp2, 14, 1);
            handle_button(P20_7, &turn_kp2, 14, -1);
            break;
        case 16:
            handle_button(P20_6, &turn_kd2, 15, 1);
            handle_button(P20_7, &turn_kd2, 15, -1);
            break;
        case 17:
            handle_button(P20_6, &straight_speed, 16, 5);
            handle_button(P20_7, &straight_speed, 16, -5);
            break;
        case 18:
            handle_button(P20_6, &other_speed, 17, 5);
            handle_button(P20_7, &other_speed, 17, -5);
            break;
    }
}
 * */
void scankey()//扫描按键
{

/*gpio_get_level：gpio 电平获取
 */
 if( gpio_get_level(P33_11) == 0 )//如果P33_11引脚电平为0,控制i
 {
     system_delay_ms(500);//延迟消抖
     if( gpio_get_level(P33_11) == 0 )
     {
         i++;
         if(i==19)i=0;
     }
 }
 if( gpio_get_level(P20_8) == 0 )//如果P20_8引脚电平为0,控制mode
 {
     system_delay_ms(500);//延迟消抖
     if( gpio_get_level(P20_8) == 0 )
     {
         mode++;
         if(mode==5)mode=-2;
     }
 }
     if(i==0)
    {
        if( gpio_get_level(P20_6) == 0 )
       {
          system_delay_ms(500);//延迟消抖
          if( gpio_get_level(P20_6) == 0 )
         {
             pid.F_zc_Tzero+=0.1;
            flash_union_buffer[17].float_type =pid.F_zc_Tzero;
            flash_erase_page(0, 5);                // 擦除第5页
            flash_write_page_from_buffer(0, 5);    //  向指定 FLASH 的扇区的指定页码写入缓冲区的数据
         }
       }
        if( gpio_get_level(P20_7) == 0 )
       {
          system_delay_ms(500);//延迟消抖
          if( gpio_get_level(P20_7) == 0 )
         {
            pid.F_zc_Tzero-=0.1;
            flash_union_buffer[17].float_type =pid.F_zc_Tzero;
            flash_erase_page(0, 5);                // 擦除第5页
            flash_write_page_from_buffer(0, 5);    //  向指定 FLASH 的扇区的指定页码写入缓冲区的数据
         }
        }
        break;
     }
    if(i==-1)
     {
         if( gpio_get_level(P20_6) == 0 )
       {
           system_delay_ms(500);//延迟消抖
           if( gpio_get_level(P20_6) == 0 )
         {
            speed_alone += 10;
            flash_union_buffer[0].float_type =pid.fgyrokp;
            flash_erase_page(0, 5);                // 擦除第5页
            flash_write_page_from_buffer(0, 5);    //  向指定 FLASH 的扇区的指定页码写入缓冲区的数据
         }
       }
         if( gpio_get_level(P20_7) == 0 )
           {
               system_delay_ms(500);//延迟消抖
               if( gpio_get_level(P20_7) == 0 )
               {
                 speed_alone -= 10;
                 flash_union_buffer[0].float_type =pid.fgyrokp;
                 flash_erase_page(0, 5);             // 擦除第5页
                 flash_write_page_from_buffer(0, 5); // 向指定 FLASH 的扇区的指定页码写入缓冲区的数据
               }
            }
         break;
      }



 if(i==0)
    {
        if( gpio_get_level(P20_6) == 0 )
      {
          system_delay_ms(500);
          if( gpio_get_level(P20_6) == 0 )

        {
              pid.F_zc_Tzero += 0.1;
            flash_union_buffer[0].float_type =pid.fgyrokp;
           flash_erase_page(0, 5);
            flash_write_page_from_buffer(0, 5);        //
        }

      }


        if( gpio_get_level(P20_7) == 0 )
          {
              system_delay_ms(500);
              if( gpio_get_level(P20_7) == 0 ) //
              {
                  pid.F_zc_Tzero -= 0.1;
                  flash_union_buffer[0].float_type =pid.fgyrokp;
                flash_erase_page(0, 5);                //
                flash_write_page_from_buffer(0, 5);        //
              }

          }



    }




    if(i==1)
  {
      if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

      {
          pid.fgyrokp++;
          flash_union_buffer[0].float_type =pid.fgyrokp;
         flash_erase_page(0, 5);                //
          flash_write_page_from_buffer(0, 5);        //
          }

    }


      if( gpio_get_level(P20_7) == 0 )
        {
            system_delay_ms(500);
            if( gpio_get_level(P20_7) == 0 ) //

            {
                pid.fgyrokp--;
                flash_union_buffer[0].float_type =pid.fgyrokp;
              flash_erase_page(0, 5);                //
              flash_write_page_from_buffer(0, 5);        //
              }

        }



 }

    if(i==2)
   {if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

        {
            pid.fgyroki++;
            flash_union_buffer[1].float_type =pid.fgyroki;
                     flash_erase_page(0, 5);                //
                     flash_write_page_from_buffer(0, 5);
        }

    }


   if( gpio_get_level(P20_7) == 0 )
     {
         system_delay_ms(500);
         if( gpio_get_level(P20_7) == 0 ) //

         {
             pid.fgyroki--;
             flash_union_buffer[1].float_type =pid.fgyroki;
                                 flash_erase_page(0, 5);                //
                                 flash_write_page_from_buffer(0, 5);
         }

     }



   }


    if(i==3)
   {if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

        {
            pid.flywheelkp++;
            flash_union_buffer[2].float_type =pid.flywheelkp;
                                flash_erase_page(0, 5);                //
                                flash_write_page_from_buffer(0, 5);        //
        }

    }


   if( gpio_get_level(P20_7) == 0 )
     {
         system_delay_ms(500);
         if( gpio_get_level(P20_7) == 0 ) //

         {
             pid.flywheelkp--;
             flash_union_buffer[2].float_type =pid.flywheelkp;
                                           flash_erase_page(0, 5);                //
                                           flash_write_page_from_buffer(0, 5);        //
         }

     }




   }


    if(i==4)
   {if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

        {
            pid.flywheelkd++;
            flash_union_buffer[3].float_type = pid.flywheelkd;
                   flash_erase_page(0, 5);                //
                   flash_write_page_from_buffer(0, 5);        //
        }

    }


   if( gpio_get_level(P20_7) == 0 )
     {
         system_delay_ms(500);
         if( gpio_get_level(P20_7) == 0 ) //

         {
             pid.flywheelkd--;
             flash_union_buffer[3].float_type = pid.flywheelkd;
                          flash_erase_page(0, 5);                //
                          flash_write_page_from_buffer(0, 5);        //
         }

     }



   }


    if(i==5)
   {
        if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

        {
            pid.flymotorKp++;
            flash_union_buffer[4].float_type = pid.flymotorKp;
                         flash_erase_page(0, 5);                //
                         flash_write_page_from_buffer(0, 5);        //
        }

    }



   if( gpio_get_level(P20_7) == 0 )
     {
         system_delay_ms(500);
         if( gpio_get_level(P20_7) == 0 ) //

         {
             pid.flymotorKp--;
             flash_union_buffer[4].float_type = pid.flymotorKp;
                                     flash_erase_page(0, 5);                //
                                     flash_write_page_from_buffer(0, 5);        //
         }

     }


   }


    if(i==6)
   {if( gpio_get_level(P20_6) == 0 )
    {
        system_delay_ms(500);
        if( gpio_get_level(P20_6) == 0 ) //

        {
            pid.flymotorKi++;
            flash_union_buffer[5].float_type = pid.flymotorKi;
                                    flash_erase_page(0, 5);                //
                                    flash_write_page_from_buffer(0, 5);        //
        }

    }


   if( gpio_get_level(P20_7) == 0 )
     {
         system_delay_ms(500);
         if( gpio_get_level(P20_7) == 0 ) //

         {
             pid.flymotorKi--;
             flash_union_buffer[5].float_type = pid.flymotorKi;
                            flash_erase_page(0, 5);                //
                  flash_write_page_from_buffer(0, 5);        //
         }

     }


   }




    if(i==7)
       {if( gpio_get_level(P20_6) == 0 )
        {
            system_delay_ms(500);
            if( gpio_get_level(P20_6) == 0 ) //

            {
                GyroP++;
                flash_union_buffer[6].float_type = GyroP;
                   flash_erase_page(0, 5);                //
                       flash_write_page_from_buffer(0, 5);        //
            }

        }


       if( gpio_get_level(P20_7) == 0 )
         {
             system_delay_ms(500);
             if( gpio_get_level(P20_7) == 0 ) //

             {
                 GyroP--;
                 flash_union_buffer[6].float_type = GyroP;
                                  flash_erase_page(0, 5);                //
                                      flash_write_page_from_buffer(0, 5);        //
             }

         }



       }

    if(i==8)
           {if( gpio_get_level(P20_6) == 0 )
            {
                system_delay_ms(500);
                if( gpio_get_level(P20_6) == 0 ) //

                {
                    GyroI++;
                    flash_union_buffer[7].float_type = GyroI;
                                 flash_erase_page(0, 5);                //
                                    flash_write_page_from_buffer(0, 5);        //
                }

            }


           if( gpio_get_level(P20_7) == 0 )
             {
                 system_delay_ms(500);
                 if( gpio_get_level(P20_7) == 0 ) //

                 {
                     GyroI--;
                     flash_union_buffer[7].float_type = GyroI;
                            flash_erase_page(0, 5);                //
                          flash_write_page_from_buffer(0, 5);        //
                 }

             }



           }


    if(i==9)
          {if( gpio_get_level(P20_6) == 0 )
           {
               system_delay_ms(500);
               if( gpio_get_level(P20_6) == 0 ) //

               {
                   AngleP++;
                   flash_union_buffer[8].float_type = AngleP;
                                            flash_erase_page(0, 5);                //
                                          flash_write_page_from_buffer(0, 5);
               }

           }


          if( gpio_get_level(P20_7) == 0 )
            {
                system_delay_ms(500);
                if( gpio_get_level(P20_7) == 0 )

                {
                    AngleP--;
                    flash_union_buffer[8].float_type = AngleP;
                                  flash_erase_page(0, 5);
                       flash_write_page_from_buffer(0, 5);
                }

            }

          }

       if(i==10)
              {if( gpio_get_level(P20_6) == 0 )
               {
                   system_delay_ms(500);
                   if( gpio_get_level(P20_6) == 0 ) //

                   {
                       AngleD++;
                       flash_union_buffer[9].float_type = AngleD;
                                           flash_erase_page(0, 5);                //
                                flash_write_page_from_buffer(0, 5);
                   }
               }


              if( gpio_get_level(P20_7) == 0 )
                {
                    system_delay_ms(500);
                    if( gpio_get_level(P20_7) == 0 ) //

                    {
                        AngleD--;
                        flash_union_buffer[9].float_type = AngleD;
                                     flash_erase_page(0, 5);                //
                            flash_write_page_from_buffer(0, 5);
                    }

                }


              }


       if(i==11)
                {if( gpio_get_level(P20_6) == 0 )
                 {
                     system_delay_ms(500);
                     if( gpio_get_level(P20_6) == 0 ) //

                     {
                         SpeedP++;
                         flash_union_buffer[10].float_type = SpeedP;
                                                  flash_erase_page(0, 5);                //
                                         flash_write_page_from_buffer(0, 5);        //
                     }

                 }


                if( gpio_get_level(P20_7) == 0 )
                  {
                      system_delay_ms(500);
                      if( gpio_get_level(P20_7) == 0 ) //

                      {
                          SpeedP--;
                          flash_union_buffer[10].float_type = SpeedP;
                                      flash_erase_page(0, 5);                //
                                    flash_write_page_from_buffer(0, 5);        //
                      }

                  }


                }



             if(i==12)
                    {if( gpio_get_level(P20_6) == 0 )
                     {
                         system_delay_ms(500);
                         if( gpio_get_level(P20_6) == 0 ) //

                         {
                             SpeedI++;
                             flash_union_buffer[11].float_type = SpeedI;
                              flash_erase_page(0, 5);                //
                             flash_write_page_from_buffer(0, 5);        //
                         }

                     }


                    if( gpio_get_level(P20_7) == 0 )
                      {
                          system_delay_ms(500);
                          if( gpio_get_level(P20_7) == 0 ) //

                          {
                              SpeedI--;
                              flash_union_buffer[11].float_type = SpeedI;
                                   flash_erase_page(0, 5);                //
                            flash_write_page_from_buffer(0, 5);        //
                            }

                      }

                    }



             if(i==13)
                               {if( gpio_get_level(P20_6) == 0 )
                                {
                                    system_delay_ms(500);
                                    if( gpio_get_level(P20_6) == 0 ) //

                                    {
                                        turn_kp++;
                                        flash_union_buffer[12].float_type = turn_kp;
                                         flash_erase_page(0, 5);                //
                                        flash_write_page_from_buffer(0, 5);
                                    }

                                }


                               if( gpio_get_level(P20_7) == 0 )
                                 {
                                     system_delay_ms(500);
                                     if( gpio_get_level(P20_7) == 0 ) //

                                     {
                                         turn_kp--;
                                         flash_union_buffer[12].float_type = turn_kp;
                                              flash_erase_page(0, 5);                //
                                       flash_write_page_from_buffer(0, 5);        //
                                       }

                                 }


                               }



             if(i==14)
                                     {if( gpio_get_level(P20_6) == 0 )
                                      {
                                          system_delay_ms(500);
                                          if( gpio_get_level(P20_6) == 0 ) //

                                          {
                                              turn_kd++;
                                              flash_union_buffer[13].float_type = turn_kd;
                                               flash_erase_page(0, 5);                //
                                              flash_write_page_from_buffer(0, 5);
                                          }

                                      }


                                     if( gpio_get_level(P20_7) == 0 )
                                       {
                                           system_delay_ms(500);
                                           if( gpio_get_level(P20_7) == 0 ) //

                                           {
                                               turn_kd--;
                                               flash_union_buffer[13].float_type = turn_kd;
                                                    flash_erase_page(0, 5);                //
                                             flash_write_page_from_buffer(0, 5);        //
                                             }

                                       }


                                     }


             if(i==15)
                  {if( gpio_get_level(P20_6) == 0 )
                   {
                       system_delay_ms(500);
                       if( gpio_get_level(P20_6) == 0 ) //

                       {
                           turn_kp2++;
                           flash_union_buffer[14].float_type = turn_kp2;
                            flash_erase_page(0, 5);                //
                           flash_write_page_from_buffer(0, 5);        //
                           }

                   }


                  if( gpio_get_level(P20_7) == 0 )
                    {
                        system_delay_ms(500);
                        if( gpio_get_level(P20_7) == 0 ) //

                        {
                            turn_kp2--;
                            flash_union_buffer[14].float_type = turn_kp2;
                                 flash_erase_page(0, 5);                //
                          flash_write_page_from_buffer(0, 5);
                        }

                    }




}


             if(i==16)
                            {if( gpio_get_level(P20_6) == 0 )
                              {
                                  system_delay_ms(500);
                                  if( gpio_get_level(P20_6) == 0 ) //

                                  {
                                      turn_kd2++;
                                      flash_union_buffer[15].float_type = turn_kd2;
                                      flash_erase_page(0, 5);                //
                                      flash_write_page_from_buffer(0, 5);        //
                                      }

                              }


                            if( gpio_get_level(P20_7) == 0 )
                              {
                                  system_delay_ms(500);
                                  if( gpio_get_level(P20_7) == 0 ) //

                                  {
                                      turn_kd2--;
                                      flash_union_buffer[15].float_type = turn_kd2;
                                            flash_erase_page(0, 5);                //
                                    flash_write_page_from_buffer(0, 5);        //
                                  }

                              }


                                                              }



             if(i==17)
                                {
                    if( gpio_get_level(P20_6) == 0 )
                  {
                      system_delay_ms(500);
                      if( gpio_get_level(P20_6) == 0 ) //

                      {
                          straight_speed+=5;

                      }

                  }


                if( gpio_get_level(P20_7) == 0 )
                  {
                      system_delay_ms(500);
                      if( gpio_get_level(P20_7) == 0 ) //

                      {
                          straight_speed-=5;

                      }

                            }



            }
             if(i==18)
                                         {
                             if( gpio_get_level(P20_6) == 0 )
                           {
                               system_delay_ms(500);
                               if( gpio_get_level(P20_6) == 0 ) //

                               {
                                   other_speed+=5;

                               }

                           }


                         if( gpio_get_level(P20_7) == 0 )
                           {
                               system_delay_ms(500);
                               if( gpio_get_level(P20_7) == 0 ) //
                               {
                                   other_speed-=5;

                               }

                                     }



                     }
 }



void flash_init(void)// 读取 Flash 数据并初始化 PID 控制器的参数
{



       flash_read_page_to_buffer(0, 5);           // 读取第 0 页到缓冲区
       /*将 flash_union_buffer 中的各个浮点型值提取并赋值给 pid 结构体的成员变量。
        * 这样，PID 控制器的各个系数（例如 fgyrokp, fgyroki, flywheelkp 等）就从 Flash 存储器中恢复到程序中，供后续使用。*/
       pid.fgyrokp=flash_union_buffer[0].float_type;
       pid.fgyroki=flash_union_buffer[1].float_type;
       pid.flywheelkp=flash_union_buffer[2].float_type;
       pid.flywheelkd=flash_union_buffer[3].float_type;
       pid.flymotorKp=flash_union_buffer[4].float_type;
       pid.flymotorKi=flash_union_buffer[5].float_type;
       /*flash_union_buffer 是一个 union 类型的缓冲区，它的成员 float_type 表示存储在 Flash 中的数据类型是 float。*/



      // flash_read_page_to_buffer(0, 8);           //

       GyroP=flash_union_buffer[6].float_type;
       GyroI=flash_union_buffer[7].float_type;
       AngleP=flash_union_buffer[8].float_type;
       AngleD=flash_union_buffer[9].float_type;
       SpeedP=flash_union_buffer[10].float_type;
       SpeedI=flash_union_buffer[11].float_type;
      // test=flash_union_buffer[0].float_type;
       //test1=flash_union_buffer[11].float_type;


       turn_kp=flash_union_buffer[12].float_type;
       turn_kd=flash_union_buffer[13].float_type;
       turn_kp2=flash_union_buffer[14].float_type;
       turn_kd2=flash_union_buffer[15].float_type;
       /*这些参数用于控制转向（例如 turn_kp, turn_kd 等是转向 PID 控制的系数），可能用于机器人的自动控制中。*/
       speed_alone=flash_union_buffer[16].float_type;
       pid.F_zc_Tzero=flash_union_buffer[17].float_type;
       /*speed_alone 和 pid.F_zc_Tzero 可能是与速度或其他控制因素相关的参数，用于微调控制算法。*/


}


extern float fg;
extern float fy;
extern float error_of_CameraOrBalance;
extern float angle_s;
extern float turn_jiaodu;
extern float angle_pid;
extern float output;
extern float speed_pid;
extern float turn_sudu;
extern float pwm_alone;
extern int current_target;
extern int define_target;
extern float target_distance;
extern float bmq;
extern float yaw_angle1;
extern float zero_data;
extern float micrae;
extern int rtk_current_target;
extern int rtk_define_target;
extern double angle_3;
extern double angle_2;
extern double rtk_target_distance;
//extern double gnss.antenna_direction;
void change_page(void)//
{//显示信息到屏幕上
    ips200_show_float(0,160,pwm_alone,4,1);
    ips200_show_int(180,0,stop_flag,1);
    ips200_show_int(180,15,mode,1);
    ips200_show_int(170,30,define_target,3);
    ips200_show_int(170,45,current_target,3);
    ips200_show_int(170,60,target_distance,4);
    ips200_show_int(170,75,bmq,4);
    ips200_show_float(100,275, yaw_angle1, 3, 1);//
    ips200_show_float(170,90,zero_data,3,1);//
    ips200_show_float(90,170,speed_alone,3,3);
    ips200_show_float(170,115,micrae,1,4);
    ips200_show_int(190,30,rtk_define_target,2);
    ips200_show_int(190,45,rtk_current_target,2);
    ips200_show_float(170,130,angle_2,3,1);
    ips200_show_float(170,145,gnss.antenna_direction,3,2);
    ips200_show_float(170,160,rtk_target_distance,2,3);
//    ips200_show_float(0,160,rtk_target_distance,2,3);
    if(i==-1)
    {
        ips200_show_float(90,185,speed_alone,3,3);
        ips200_show_int(0, 300, i, 2);
        ips200_displayimage8660((const uint16 *)scc8660_image, SCC8660_W, SCC8660_H);
        ips200_show_float(130, 275,pwm_alone,2,2);
        ips200_show_int(180,185,angle_s,4);
        ips200_show_int(180,225,angle_pid,4);
        ips200_show_float(180,265,turn_jiaodu,3,2);
        ips200_show_float(180,300,error_of_CameraOrBalance,3,1);
        ips200_show_float(180, 205,output,2,4);
        ips200_show_float(180,245,speed_pid,2,3);
        ips200_show_float(180,284,turn_sudu,2,3);

    }
 if(i==0)
    {

     ips200_show_float(110,185,pid.F_zc_Tzero,2,2);
     ips200_show_float(0,275, fg, 2, 2);
     ips200_show_float(50,275, fy, 2, 2);
     ips200_show_int(0, 300, i, 2);
     ips200_displayimage8660((const uint16 *)scc8660_image, SCC8660_W, SCC8660_H);
     ips200_show_int(180,185,angle_s,4);
     ips200_show_int(180,225,angle_pid,4);
     ips200_show_float(180,265,turn_jiaodu,3,2);
     ips200_show_float(180,300,error_of_CameraOrBalance,3,1);
     ips200_show_float(180, 205,output,2,3);
     ips200_show_float(180,245,speed_pid,2,3);
     ips200_show_float(180,284,turn_sudu,2,3);

//     ips200_show_float(0,0,error_of_CameraOrBalance,2,2);
  }
       //ips200_clear ();
    //tft180_full(RGB565_BLACK);
   if((i>=1)&&(i<=6))
      {
        ips200_show_float(0,185, pid.fgyrokp, 3, 2);
        ips200_show_float(50,185, pid.fgyroki, 3, 2);
        ips200_show_float(0,215, pid.flywheelkp, 3, 2);
        ips200_show_float(60,215, pid.flywheelkd, 3, 2);
        ips200_show_float(0,245, pid.flymotorKp, 3, 2);
       ips200_show_float(50,245, pid.flymotorKi, 3, 2);
       ips200_show_float(0,275, fg, 2, 2);
       ips200_show_float(50,275, fy, 2, 2);
       ips200_show_int(0, 300, i, 2);
       ips200_displayimage8660((const uint16 *)scc8660_image, SCC8660_W, SCC8660_H);
       ips200_show_int(180,185,angle_s,4);
       ips200_show_int(180,225,angle_pid,4);
       ips200_show_float(180,265,turn_jiaodu,3,2);
       ips200_show_float(180,300,error_of_CameraOrBalance,3,1);
       ips200_show_float(180, 205,output,2,3);
       ips200_show_float(180,245,speed_pid,2,3);
       ips200_show_float(180,284,turn_sudu,2,3);


    }

    //tft180_full(RGB565_BLACK);
    if((i>=7)&&(i<=12))
       {

            ips200_show_float(0,185, GyroP, 3, 2);
            ips200_show_float(50,185, GyroI, 3, 2);
            ips200_show_float(0,215, AngleP, 3, 2);
            ips200_show_float(50,215,AngleD, 3, 2);
            ips200_show_float(0,245, SpeedP, 3, 2);
            ips200_show_float(50,245, SpeedI, 3, 2);
           ips200_show_float(0,275, fg, 2, 2);
           ips200_show_float(50,275, fy, 2, 2);
           ips200_show_int(0, 300, i, 2);
           ips200_displayimage8660((const uint16 *)scc8660_image, SCC8660_W, SCC8660_H);
           ips200_show_int(180,185,angle_s,4);
           ips200_show_int(180,225,angle_pid,4);
           ips200_show_float(180,265,turn_jiaodu,3,2);
           ips200_show_float(180,300,error_of_CameraOrBalance,3,1);
           ips200_show_float(180, 205,output,2,3);
           ips200_show_float(180,245,speed_pid,2,3);
           ips200_show_float(180,284,turn_sudu,2,3);

       }

    if((i>=13)&&(i<=18))
        {

                 ips200_show_float(0,185, turn_kp, 3, 2);
                 ips200_show_float(50,185, turn_kd, 3, 2);
                 ips200_show_float(0,215, turn_kp2, 3, 2);
                 ips200_show_float(50,215,turn_kd2, 3, 2);
                 ips200_show_float(0,245, straight_speed, 3, 2);
                 ips200_show_float(50,245, other_speed, 3, 2);
                 ips200_show_float(0,275, fg, 2, 2);
                 ips200_show_float(50,275, fy, 2, 2);
                 ips200_show_int(0, 300, i, 2);
                 ips200_displayimage8660((const uint16 *)scc8660_image, SCC8660_W, SCC8660_H);
                 ips200_show_int(180,185,angle_s,4);
                 ips200_show_int(180,225,angle_pid,4);
                 ips200_show_float(180,265,turn_jiaodu,3,2);
                 ips200_show_float(180,300,error_of_CameraOrBalance,3,1);
                 ips200_show_float(180, 205,output,2,3);
                 ips200_show_float(180,245,speed_pid,2,3);
                 ips200_show_float(180,284,turn_sudu,2,3);

          }


}


//extern float fy;
/********************************
{
    float  error = 0;
    error = fy- pid.F_zc_Tzero;
    pid.I_zc_output = pid.F_zc_standp2 * error;
    pid.I_zc_output -= pid.F_zc_standd *  icm20602_gyro_y;

    pid.I_zc_output1= pid.I_zc_output;
}*/
/*********************************/
/*void CTRL_compute_SpeedY()
{
    float error = pid.I_zc_setspeed - pid.I_zc_speed;

    pid.I_zc_output1 -= pid.F_zc_standp1 * error;                        //闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎厽鏅搁柨鐕傛嫹??
}*/


//float P_Balance_KP=2990, P_Balance_KI=0,P_Balance_KD=-80;                //C闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闁圭厧鐡ㄧ划鎾荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕拷鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梺璺ㄥ櫐閹凤拷???
//Y闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚崡鎶藉箰椤掑嫭鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箳缁瑩鎯傞幀鎰版⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佺粯鍨兼慨銈夊疾閹间焦鐓涢柛灞久敓鑺ョ墵閺佹捇鏁撻敓锟�??,闂傚倷娴囧▔鏇㈠窗閿燂拷?闂佺澹堥幓顏嗙磽濮樿埖鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯閿燂拷?
/**************************************************************************/
/*float P_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    static float PWM=0,Bias=0;
    static float error=0;
    Bias=Angle-Angle_Zero;                                               //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚妯煎緤閸ф鐓曢悗锝庝簽缁犲鏌℃担瑙勫磳鐎殿噮鍓熸俊鍫曞幢椤撶喎顏�?
    error+=Bias;                                                         //闂備胶顭堥鍛崲濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?闂傚倷娴囧▔鏇㈠窗閿燂拷?濠电偞娼欓崥瀣偡閿曞倹鏅搁柨鐕傛嫹??
   // if(error > 30)
     //   error = 30;
    //if(error < 30)
       // error = -30;
   // error = constrain_float(error, -30, 30);                            //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
    PWM=P_Balance_KP*Bias + P_Balance_KI*error + Gyro*P_Balance_KD/10;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚妯煎緤閸ф鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸㈡煡宕查弻銉︽櫢闁跨噦鎷�???
if( fabsf(Angle)>30 )
{
 gpio_set_level(P21_4, 0);
 gpio_set_level(P02_5, 0);
}

    return PWM;
}*/


/**************************************************************************
闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?PI闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎厽鏅搁柨鐕傛嫹??,闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠囧弶顥濋梺闈涚墕濡挳骞忛敓锟�?
**************************************************************************/
/*float R_Velocity_KP=0,  R_Velocity_KI=0;                               //AB闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰鎰佹綈缂佸矂浜堕弫鎰板炊瑜嶈〖闂傚倷娴囧▔鏇㈠闯閿曞倹鏅搁柨鐕傛嫹??
float Velocity_Control_A(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder;                                                   //闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗鎼淬垻鐝舵繛鍡樺灦鐎氭氨鎲告惔銊︽櫢闁跨噦鎷�??
    Encoder *= 0.7;                                                              //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
    Encoder += Encoder_Least*0.3;                                                //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
    Encoder_Integral += Encoder;                                               //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹泛鈽夐搹顐㈩伓??闂備礁婀遍弻澶娿�掗崷顓犵闁告稑鐡ㄩ悡銉╂煟閺傛寧鍟為柣蹇ｅ櫍閺佹捇鏁撻敓锟�???
    if(Encoder_Integral>2600)
        Encoder_Integral=2600;
    if(Encoder_Integral<-2600)
          Encoder_Integral=-2600;
    //Encoder_Integral = constrain_float(Encoder_Integral, -2600, 2600);        //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
    Velocity = Encoder * R_Velocity_KP + Encoder_Integral * R_Velocity_KI/100; //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚妯煎緤閸ф鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸㈡煡宕查弻銉︽櫢闁跨噦鎷�???
    if(Velocity>5000)
        Velocity=5000;
     if(Velocity<-5000)
         Velocity=-5000;
   // Velocity = constrain_float(Velocity, -5000, 5000);
    return Velocity;
}*/



//***********************闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰槌栧姧id





//简单的陀螺仪（Gyro）PID 控制算法，其中 Gyro_PIDL 函数用于根据给定的陀螺仪数据和目标数据计算误差
/*
 * Gyro_Data：当前的陀螺仪测量值（实际角速度）。
 * Gyro_m：目标角速度或期望值。
 * */
float Gyro_PIDL(float Gyro_Data,float Gyro_m)
{
    static  float Gyro_Error=0;//误差 Increase1=0,Increase2=0,
    static  float last_gyro_error=0;//上一周期的陀螺仪误差
    //Gyro_Data=0.65*Gyro_Data+0.35*LAST_Gyro_Data;
    //uint8 dat[10];
   // LAST_Gyro_Data=Gyro_Data;
    static float Realize=0;//累计的 PID 控制量，用于根据误差调整输出。
    //积分限幅么的
    Gyro_Error =  Gyro_Data-Gyro_m;//Gyro_Error = Gyro_m - Gyro_Data;

    Realize += pid.fgyrokp*(Gyro_Error-last_gyro_error)+ pid.fgyroki/100 * Gyro_Error;//fgyrokp


    if(Realize>10000) Realize=10000;
    else if(Realize<-10000) Realize=-10000;


    last_gyro_error=Gyro_Error;

    return Realize;
}





float flywheel_speedL(float Angle,float Angle_m)//闂佸搫绉烽～澶婄暤娴ｈ妲归柣鎰閺嬪倿鏌ｉ妸銉ヮ仼缂傚秴顑夊畷婊冾吋韫囨柨顏�?闁硅壈鎻拋锝囨濮楁棘gle闂佹寧绋戦ˇ顖炲箯娴煎瓨鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柟鐑樺灥椤斿﹪鏌ㄥ☉妯荤ngle_m闂佹寧绋戦ˇ鏉款焽閻㈠灚濯奸柨娑樺閺嗩剙鈽夐幘顖氫壕婵炴垶鎸剧�ｄ箘M闂佹寧绋戦悧鎰板礈閿曞倸绀冮悹鍥囧嫬顏�?闁硅壈鎻粻鎺楁儍閻旂厧绀嗛悹浣告贡缁�鍡椙庨崶锝忔嫹閸愭彃鈻忛梺姹囧妼鐎氼剟鍩�椤掑倶锟藉妲愬┑鍥╊浄闁靛绠戞禒姗�鏌涢幒鏇ㄥ晣闁瑰嚖鎷�?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹?
{
    float PWM,flywheel_error;
    float flywheel_error_last=0;
    flywheel_error = Angle-pid.F_zc_Tzero-Angle_m;///flywheel.error = Angle-expect_angle-Angle_m; //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺佹捇鏁撻敓锟�?闂佽法鍠愰弸濠氬箯妞嬪海绀婇悗锝庡枛缁�鍫拷骞垮劚椤︿即寮查幖浣圭厸闁稿本锚閿熻姤鐗滈敓鑺ヨ壘缂嶅﹪寮婚妸鈺傚亞闁稿本绋戦锟�

  //  flywheel.Integration += flywheel.error ;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎厽鏅搁柨鐕傛嫹??
    //if(flywheel.Integration<-I_max)      flywheel.Integration=-I_max;         //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
    //else if(flywheel.Integration>I_max)  flywheel.Integration= I_max;         //闂傚倷娴囧▔鏇㈠窗閹炬剚鐒藉Δ锝呭暙缁�鍕煏閸繃锛旈柟鍑ゆ嫹?

    //guodud=;
    PWM = flywheel_error * pid.flywheelkp + ( pid.flywheelkd) * (flywheel_error-flywheel_error_last) ; //


    flywheel_error_last=flywheel_error;

    //if(PWM>8000) PWM=8000;
    //else if(PWM<=-8000) PWM=-8000;

    return PWM;
}




float fspeedL(float INencoder)//闂佺儵鍋撻崝瀣姳椤掑倹濯奸柨娑樺閺嗩剙顪冪�ｎ剙浠╅柣銈呮閹粙宕归锝囩毣闂佹眹鍔岀�氫即鍩�椤掑倸鏋庨悗瑙勫▕閺佸秶浠﹂崐鐔风彲闂佺硶鏅炲銊ц姳椤掍焦缍囬柟鎯у暱瀵娊鏌ｉ妸銉ヮ伀缂侀硸浜幆宥嗘媴閸涘﹦褰滈梺绋匡功閵嗗妲愬顪攅ncoder闂佹寧绋戦ˇ宕囨崲閹达箑鐐婇柣鎰椤忛亶鏌ㄩ悤鍌涘?闂佽法鍠愰弸濠氬箯闁垮鍎熼柨鏇嫹闁逞屽墰閵嗗妲愬绀秎ocity闂佹寧绋戦¨锟介柟鍑ゆ嫹?
{
    static float Encoder,flymotor_Integration=0;//,Encoder_Least

    float Velocity;
/*
    Encoder_Least = INencoder;  //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏犵暋闂堣埖鐩幃瑙勭瑹婵犲啫顏�?闂佽法鍣﹂幏锟�??                                                //闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗鎼淬垻鐝舵繛鍡樺灦鐎氭氨鎲告惔銊︽櫢闁跨噦鎷�??
    Encoder *= 0.7; //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?                                                           //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
    Encoder += Encoder_Least*0.3;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ偖鐎涙ê顏�?ncoder*0.3+闂備礁鎲￠…鍥窗閺嶎厼鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳绱掔仦浠嬫寬coder*0.7                                      //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
  */                                               //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹泛鈽夐搹顐㈩伓??闂備礁婀遍弻澶娿�掗崷顓犵闁告稑鐡ㄩ悡銉╂煟閺傛寧鍟為柣蹇ｅ櫍閺佹捇鏁撻敓锟�???
    Encoder = INencoder;                                                //闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗鎼淬垻鐝舵繛鍡樺灦鐎氭氨鎲告惔銊︽櫢闁跨噦鎷�??
     //  Encoder *= 0.2;                                                            //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
     //  Encoder += Encoder_Least*0.8;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ偖鐎涙ê顏�?ncoder*0.7+闂備礁鎲￠…鍥窗閺嶎厼鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳绱掔仦浠嬫寬coder*0.3

//    if(abs(Encoder)>150)
//        {flymotor.Integration=0;}
//    else{
             flymotor_Integration += Encoder;
             if(flymotor_Integration >= +2000) flymotor_Integration = +2000;                    //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
             if(flymotor_Integration < -2000) flymotor_Integration = -2000;                    //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
//        }
   // oled_printf_float(80, 0,Encoder,3,3);
    Velocity = Encoder *  pid.flymotorKp/1000 + flymotor_Integration *  pid.flymotorKi/100000;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚妯煎緤閸ф鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸㈡煡宕查弻銉︽櫢闁跨噦鎷�???
    if(Velocity>30) Velocity=30;
    else if(Velocity<=-30) Velocity=-30;
    return Velocity;

}






//void shuchu1()
//{// printf("%d,%d,%f\n",fg,fy,0);
//}







void motorAB_set(float pwm,float pwm1)          //balance ，设置电机AB转速
{
    if(pwm<0)
    {
      //pwm-=330;
      pwm=-pwm;
      if(pwm>10000)
      pwm=10000;

          pwm=10000-pwm;
       gpio_set_level(P02_4, 0);
         // gpio_set_level(P02_6, 0);

                   // pwm_set_duty(ATOM0_CH7_P02_7, pwm1);
       pwm_set_duty(ATOM0_CH5_P02_5, pwm);
    }
    else
    {
        if(pwm>10000)
            pwm=10000;
        pwm=10000-pwm;

      //  gpio_set_level(P02_6, 1);

                //  pwm_set_duty(ATOM0_CH7_P02_7, pwm1);

       gpio_set_level(P02_4, 1);
       pwm_set_duty(ATOM0_CH5_P02_5, pwm);
    }



    if(pwm1<0)
        {
          //pwm1-=330;
          pwm1=-pwm1;
          if(pwm1>10000)
              pwm1=10000;
          pwm1=10000-pwm1;
           gpio_set_level(P02_6, 0);

           pwm_set_duty(ATOM0_CH7_P02_7, pwm1);
        }
        else
        {

          if(pwm1>10000)
              pwm1=10000;
          pwm1=10000-pwm1;

           gpio_set_level(P02_6, 1);
           pwm_set_duty(ATOM0_CH7_P02_7, pwm1);
        }

}












/*gyro_z：当前的陀螺仪 Z 轴的角速度（单位：角度/秒或者弧度/秒）。它表示当前设备的旋转速度。
 * jiaodu：目标角度。通常，这个值是预设的目标角度，表示设备应保持的角度。
 * */
float turn_gryo(float gyro_z,float jiaodu)//计算出控制电机的pwm值
{
  static float PWM_out=0;
  static float error=0;
    static float last_error=0;
  //static float last=0;
//      data_conversion(gyro_Z,gyro_z,0,0, virtual_scope_data);//濠电偠鎻徊钘夘焽閿熺姴鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒佽础缂侇喖鐗撻弫鎾绘晸閿燂拷?闂佽法鍠愰弸濠氬箯閻戣姤鐓涢柛灞久敓鑺ョ墱閿熻姤鑹剧紞濠囧蓟閵娾晜鍋勯柛娑橈功娴煎嫰姊鸿ぐ鎺濇闁稿繑锕㈠顐﹀箻鐠囧弶顥濋梺闈涚墕濡挳骞忛敓锟�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘?闂佽法鍠愰弸濠氬箯閻戣姤鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇寮绘径濞掑綊宕卞☉娆戝弳闂佺粯鏌ㄩ幉锟犳倶椤曪拷閺岀喐顦版惔鈥冲箣闂佽桨鐒﹂幑鍥ь嚕椤掑嫬围闁糕槅鍘界�氾拷?闂傚倷娴囧▔鏇㈠窗閹邦剛鏆ら幖娣妽閺呮煡鐓崶銊︻棖闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш鐎规洏鍎虫禒锕傚箻閸涱喖顏�?闂佽法鍣﹂幏锟�???闂佽桨鐒﹂幑鍥ь嚕椤掑嫬围闁糕槅鍘界�氾拷?闂傚倷娴囧▔鏇㈠窗瀹ュ鍤戦幖娣妽椤ュ牓鏌嶉崫鍕喊闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎厽鏅搁柨鐕傛嫹??
//     uart_putbuff(UART_1,virtual_scope_data,10);  //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰鎰佹綈婵炵》鎷�?闂佽桨鐒﹂幑鍥ь嚕椤掑嫬围闁糕槅鍘界�氾拷?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垰顪冪�ｎ亪顎楅柟鎼炲�濆铏规崉閵娿儲鐏佺紓渚囧枟閹倿骞嗛崒鐐插窛濠电姴绻戠�氾拷?濠电偞鍨堕弻銊╂偋閹捐鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂佽崵濮村ú锔惧垝閹捐鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闁诲海鏁告晶妤冩暜閿熺姴鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀锟藉顐﹀箻鐠囪尙顔夐悗鐟板濠㈡ê鈻撻埡鍛厵濡炲楠搁崢鎾煛娴ｈ宕岄柡灞斤躬閺佹捇鏁撻敓锟�???闂備胶鎳撻崲鏌ュ床閺屻儲鏅搁柨鐕傛嫹??
    error=gyro_z-jiaodu;
  PWM_out = turn_kp*error+turn_kd*(error-last_error);
  last_error=error;
   if(PWM_out>=7000)
        PWM_out=7000;
    if(PWM_out<=-7000)
        PWM_out=-7000;

  return PWM_out;
}



/*这个参数表示来自摄像头或者平衡控制系统的误差数据。它通常是一个角度误差或者与期望状态的偏差。
 * sudu：表示当前的速度或某个相关的物理量，通常是角速度或者设备的实际移动速度。
 * */
float turn_angle(float error_of_CameraOrBalance,float sudu)//
{

 static float PWM_out=0;
 static float error=0;
 static float last_error=0;

   // PWM_out = (turn_kp2)*error_of_CameraOrBalance +turn_kd2*(error_of_CameraOrBalance-last_gyro_z);
//  PWM_out = (turnkp2)*gyro_z +turn_kd2*(gyro_z-last_gyro_z);
   // last_gyro_z=error_of_CameraOrBalance;
 error=(error_of_CameraOrBalance-sudu);//误差的计算方式表明，系统试图让 error_of_CameraOrBalance 逐渐接近 sudu，即目标和当前速度之间的差异。
 PWM_out =turn_kp2*error +turn_kd2*(error-last_error);
   last_error=error;
       // if(PWM_out>5000)
       // PWM_out=5000;
  //  if(PWM_out<-5000)
      //  PWM_out=-5000;
  if(PWM_out>=2000)
        PWM_out=2000;
  if(PWM_out<=-2000)
        PWM_out=-2000;
  return PWM_out;
}


/*INencoder：输入的编码器值，通常代表编码器的计数值或者角度位置，表示设备的当前位置变化。
 * */
float turn_speed(float INencoder)//
{
    static float Encoder,control_Integration,Encoder_Least;

    float Velocity;

    Encoder = INencoder*1.0;  //用于存储编码器的当前值。
    Encoder *= 0.8; //这一步是对编码器值进行平滑处理，通过将当前编码器值乘以 0.8 来减少系统中的噪声或者小幅波动。这样做能使系统响应更加平稳，减少由于瞬时波动导致的控制误差
    Encoder += Encoder_Least*0.2;   //将上一周期的编码器值（Encoder_Least）按 0.2 的权重加到当前编码器值中，这样做是为了保留一定的历史信息，进一步平滑编码器输入。
   // Encoder = INencoder;                                                //闂傚倷娴囧▔鏇㈠窗鎼淬劍鏅搁柨鐕傛嫹?闂佽法鍠愰弸濠氬箯妞嬪孩鍠嗛柛鏇″煐鐎氾拷?闂傚倷娴囧▔鏇㈠窗鎼淬垻鐝舵繛鍡樺灦鐎氭氨鎲告惔銊︽櫢闁跨噦鎷�??
     //  Encoder *= 0.2;                                                            //濠电偞鍨堕幖顐﹀箯閻戣姤鈷戦悹鎭掑妼閺嬫稒淇婇娆掑厡闁哥姴锕ュ鍕拷锝堝煐鐎氾拷?闂佽法鍣﹂幏锟�?闂佽法鍠愰弸濠氬箯閻戣棄鏋侀柟鎹愵嚙缁�鍫ユ煥閻曞倹瀚�???闂備礁缍婇ˉ鎾诲礂濮楋拷瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ鎾箯閿燂拷?
     //  Encoder += Encoder_Least*0.8;   //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ偖鐎涙ê顏�?ncoder*0.7+闂備礁鎲￠…鍥窗閺嶎厼鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囷紨闁瑰嚖鎷�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳绱掔仦浠嬫寬coder*0.3
    control_Integration+=Encoder;//补偿长期偏差，积分项
//    if(abs(Encoder)>150)
//        {flymotor.Integration=0;}
//    else{
           //  flymotor_Integration += Encoder;
           //  if(flymotor_Integration >= +2000) flymotor_Integration = +2000;                    //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
           //  if(flymotor_Integration < -2000) flymotor_Integration = -2000;                    //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο鐚存嫹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゆ倻濮楋拷閻涘酣姊洪崨濠傜仧闁稿﹥鐗犻弫鎾绘晸閿燂拷??
//        }
   // oled_printf_float(80, 0,Encoder,3,3);
    /*这个公式通过编码器值与积分项计算出最终的速度输出。*/
    Velocity = Encoder *  turn_kp3/1000 + turn_ki3/10000 *control_Integration;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳锟界懓瀚妯煎緤閸ф鈷戦悹鎭掑妼閺嬫垿鏌＄�ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偓鎷峰顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸㈡煡宕查弻銉︽櫢闁跨噦鎷�???
    if(Velocity>30) Velocity=30;
    else if(Velocity<=-30) Velocity=-30;
    Encoder_Least=Encoder;
    return Velocity;

}


float jueduizhi(float err)//濠殿喚鎳撻崐鑽ゅ垝妞嬪函鎷烽悽鍨仧闁瑰嚖鎷�?
{

    float quzhi;

    if(err<0)
        quzhi=-err;
    else
        quzhi=err;
    return err;

}



extern int stop_flag;
void stop_car(void)
{
    if(stop_flag == 1)//设备停止
    {
        gpio_set_level(P11_2, 0);// P11_2 和 P11_3 的电平设置为 0
        gpio_set_level(P11_3, 0);
        GyroP=0;//将陀螺仪的比例增益（GyroP）设置为 0，可能是为了停止陀螺仪的控制功能或停止控制某个运动。
        GyroI=0;//GyroI = 0;：将陀螺仪的积分增益（GyroI）设置为 0，可能是为了防止控制器继续计算积分误差，确保设备停止时不再受控制。


        pwm_alone=0;//将单独的 PWM 控制值设置为 0，通常这是用来控制电机的 PWM 信号，这一操作意味着电机停止工作。
    }
    if(stop_flag == 0)
    {
        gpio_set_level(P11_2, 1);
        gpio_set_level(P11_3, 1);
        GyroP=-80;
        GyroI=-78;


//        pwm_alone=0;
    }
}




void scan_mode(void)
{
    if(mode==0||1)//璋冭瘯妯″紡
    {
        stop_flag=1;


    }

    if(mode==2||3)
    {
        stop_flag=0;


    }


}


void protect(float fy,float fg)
{
    if(fy>30||fg>30||fy<-30||fg<-30)
    {
        mode=-2;
    }
}




