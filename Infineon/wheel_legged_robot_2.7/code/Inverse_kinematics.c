#include <Inverse_kinematics.h>

//轮腿运动学逆解结构体声明
//示例：wheel_coordinate_struct wheel;

//轮腿运动学逆解结构体初始化
//示例：wheel_coordinate_struct_init(&wheel, 10, 10);
void wheel_coordinate_struct_init(wheel_coordinate_struct * wheel, float x, float y)
{

    wheel -> x = x;//相对后腿舵机轴的坐标x
    wheel -> y = y;//相对后腿舵机轴的坐标y
//    printf("%f,%f\n",x,y);
    wheel -> L1 = 60.0f;//后腿第一节长度
    wheel -> L2 = 90.0f;//后腿第二节长度
    wheel -> L3 = 90.0f;//前退第二节长度
    wheel -> L4 = 60.0f;//前腿第一节长度
    wheel -> L5 = 37.0f;//电机轴间距

    Inverse_kinematics(wheel);//轮腿运动学逆解计算
}

//轮腿运动学逆解计算
//示例：Inverse_kinematics(&wheel);
void Inverse_kinematics(wheel_coordinate_struct * wheel)
{
//    printf("%f,%f\n",wheel -> x,wheel -> y);
//    printf("%f\n",wheel -> y);
    float a = 2 * wheel -> x * wheel -> L1;
    float b = 2 * wheel -> y * wheel -> L1;
    float c = wheel -> x * wheel -> x + wheel -> y * wheel -> y + wheel -> L1 * wheel -> L1 - wheel -> L2 * wheel -> L2;
    float d = 2 * wheel -> L4 * (wheel -> x - wheel -> L5);
    float e = 2 * wheel -> L4 * wheel -> y;
    float f = ((wheel -> x - wheel -> L5) * (wheel -> x - wheel -> L5) + wheel -> L4 * wheel -> L4 + wheel -> y * wheel -> y - wheel -> L3 * wheel -> L3);

    float alpha1 = 2 * atan((b + sqrt((a * a) + (b * b) - (c * c))) / (a + c));
    float alpha2 = 2 * atan((b - sqrt((a * a) + (b * b) - (c * c))) / (a + c));
    float beta1 = 2 * atan((e + sqrt((d * d) + e * e - (f * f))) / (d + f));
    float beta2 = 2 * atan((e - sqrt((d * d) + e * e - (f * f))) / (d + f));

    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

//    printf("alpha1:%f,alpha2:%f,beta1:%f,beta2:%f\n",(alpha1 / PI) * 180,(alpha2 / PI) * 180,(beta1 / PI) * 180,(beta2 / PI) * 180);

    if(alpha1 >= PI/4) wheel -> rear_radian = alpha1;//去增根
    else wheel -> rear_radian = alpha2;
    if(beta1 >= 0 && beta1 <= PI/4) wheel -> front_radian = beta1;
    else wheel -> front_radian = beta2;

    wheel -> rear_angle = ((wheel -> rear_radian / PI) * 180);//弧度转角度
    wheel -> front_angle = ((wheel -> front_radian / PI) * 180);

//    printf("rear:%f,front:%f\n",wheel -> rear_angle,wheel -> front_angle);
}
