/*
 * universal_filter.c
 *
 *  Created on: 2024年11月17日
 *      Author: 17104
 */

#include <universal_filter.h>

/*------------------------------限幅滤波器------------------------------*/
//优点：能有效克服因偶然因素引起的脉冲干扰
//缺点：无法抑制那种周期性的干扰，平滑度差
//限幅滤波器结构体声明
//示例：limiting_filter_struct filter;

//限幅滤波器初始化
//示例：limiting_filter_init(&filter,100);
void limiting_filter_init(limiting_filter_struct * filter,int32 limit)
{
    filter -> limit = limit;
    filter -> flag = 0;
}

//限幅滤波器更新
//示例：limiting_filter_update(&filter);
void limiting_filter_update(limiting_filter_struct * filter)
{
    if(filter -> flag == 0)
    {
        filter -> flag = 1;
        filter -> output = filter -> data;
        filter -> last_data = filter -> data;
        return;
    }
    if (filter -> data - filter -> last_data > filter ->limit || filter -> data - filter -> last_data < (-1) * filter ->limit)
    {
        filter -> output = filter -> last_data;
    }
    else
    {
        filter -> output = filter -> data;
        filter -> last_data = filter -> data;
    }
}
/*------------------------------限幅滤波器------------------------------*/

/*------------------------------中位值滤波器------------------------------*/
//优点：能有效克服因偶然因素引起的波动干扰，对温度、液位的变化缓慢的被测参数有良好的滤波效果
//缺点：对流量、速度等快速变化的参数不宜
//中位值滤波器结构体声明
//示例：median_value_filter_struct filter;

//中位值滤波器初始化
//示例：median_value_filter_init(&filter,5);
void median_value_filter_init(median_value_filter_struct * filter,int times)
{
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//中位值滤波器更新
//示例：median_value_filter_update(&filter);
void median_value_filter_update(median_value_filter_struct * filter)
{
    filter -> value_buf[filter -> count] = filter -> data;
    filter -> count++;
    if (filter -> count < filter -> times)
    {
        return;
    }
    else
    {
        int i,j;
        filter -> flag = 1;
        int32 temp;
        filter -> count = 0;
        for (j = 0; j < filter -> times - 1; j++)
        {
            for (i = 0; i < filter -> times - j; i++)
            {
                if (filter -> value_buf[i] > filter -> value_buf[i+1] )
                {
                    temp = filter -> value_buf[i];
                    filter -> value_buf[i] = filter -> value_buf[i+1];
                    filter -> value_buf[i+1] = temp;
                }
            }
        }
        filter -> output = filter -> value_buf[(filter -> times - 1) / 2];
    }
}
/*------------------------------中位值滤波器------------------------------*/

/*------------------------------算数平均滤波器------------------------------*/
//优点：适用于对一般具有随机干扰的信号进行滤波，这样信号的特点是有一个平均值，信号在某一数值范围附近上下波动
//缺点：对于测量速度较慢或要求数据计算速度较快的实时控制不适用，比较浪费RAM
//算数平均滤波器结构体声明
//示例：arithmetic_mean_filter_struct filter;

//算数平均滤波器初始化
//一般流量：times=12，压力：times=4
//示例：arithmetic_mean_filter_init(&filter,5);
void arithmetic_mean_filter_init(arithmetic_mean_filter_struct * filter,int times)
{
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//算数平均滤波器更新
//示例：arithmetic_mean_filter_update(&filter);
void arithmetic_mean_filter_update(arithmetic_mean_filter_struct * filter)
{
    filter -> value_buf[filter -> count] = filter -> data;
    filter -> count++;
    if (filter -> count < filter -> times)
    {
        return;
    }
    else
    {
        int i;
        filter -> flag = 1;
        int32 total = 0;
        filter -> count = 0;
        for (i = 0; i < filter -> times; i++)
        {
            total += filter -> value_buf[i];
        }
        filter -> output = total / filter -> times;
    }
}
/*------------------------------算数平均滤波器------------------------------*/

/*------------------------------递推平均滤波器（滑动平均滤波器）------------------------------*/
//优点：对周期性干扰有良好的抑制作用，平滑度高，适用于高频振荡的系统
//缺点：灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差，不易消除由于脉冲干扰所引起的采样值偏差，不适用于脉冲干扰比较严重的场合，比较浪费RAM
//递推平均滤波器（滑动平均滤波器）结构体声明
//示例：recursive_average_filter_struct filter;

//递推平均滤波器（滑动平均滤波器）初始化
//一般流量：N=12，压力：N=4，液面，N=4~12，温度，N=1~4
//示例：recursive_average_filter_init(&filter,5);
void recursive_average_filter_init(recursive_average_filter_struct * filter,int times)
{
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//递推平均滤波器（滑动平均滤波器）更新
//示例：recursive_average_filter_update(&filter);
void recursive_average_filter_update(recursive_average_filter_struct * filter)
{
    if (filter -> flag == 0)
    {
        filter -> value_buf[filter -> count] = filter -> data;
        filter -> count++;
        if (filter -> count == filter -> times)
        {
            filter -> flag = 1;
        }
        else
        {
            return;
        }
    }
    else
    {
        int i;
        for (i = 0; i < filter -> times - 1; i++)
        {
            filter -> value_buf[i] = filter -> value_buf[i + 1];
        }
        filter -> value_buf[filter -> times - 1] = filter -> data;

    }
    if (filter -> flag == 1)
    {
        int i;
        int32 total = 0;
        for (i = 0; i < filter -> times; i++)
        {
            total += filter -> value_buf[i];
        }
        filter -> output = total / filter -> times;
    }
}
/*------------------------------递推平均滤波器（滑动平均滤波器）------------------------------*/

/*------------------------------中位值平均滤波器------------------------------*/
//优点：融合了两种滤波法的优点，对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差
//缺点：测量速度较慢，和算术平均滤波法一样，比较浪费RAM
//中位值平均滤波器结构体声明
//示例：median_average_filter_struct filter;

//中位值平均滤波器初始化
//示例：median_average_filter_init(&filter,5);
void median_average_filter_init(median_average_filter_struct * filter,int times)
{
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//中位值平均滤波器更新
//示例：median_value_filter_update(&filter);
void median_average_filter_update(median_average_filter_struct * filter)
{
    filter -> value_buf[filter -> count] = filter -> data;
    filter -> count++;
    if (filter -> count < filter -> times)
    {
        return;
    }
    else
    {
        int i,j;
        filter -> flag = 1;
        int32 temp,total = 0;
        filter -> count = 0;
        for (j = 0; j < filter -> times - 1; j++)
        {
            for (i = 0; i < filter -> times - j; i++)
            {
                if (filter -> value_buf[i] > filter -> value_buf[i+1] )
                {
                    temp = filter -> value_buf[i];
                    filter -> value_buf[i] = filter -> value_buf[i+1];
                    filter -> value_buf[i+1] = temp;
                }
            }
        }
        for (i = 1; i < filter -> times - 1; i++)
        {
            total += filter -> value_buf[i];
        }
        filter -> output = total / (filter -> times - 2);
    }
}
/*------------------------------中位值平均滤波器------------------------------*/

/*------------------------------限幅平均滤波器------------------------------*/
//优点：融合了两种滤波法的优点，对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差
//缺点：比较浪费RAM
//限幅平均滤波器结构体声明
//示例：limiting_average_filter_struct filter;

//限幅平均滤波器初始化
//一般流量：N=12，压力：N=4，液面，N=4~12，温度，N=1~4
//示例：limiting_average_filter_init(&filter,5,100);
void limiting_average_filter_init(limiting_average_filter_struct * filter,int times,int32 limit)
{
    filter -> limit = limit;
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
    filter -> limit_flag = 0;
}

//限幅平均滤波器更新
//示例：limiting_average_filter_update(&filter);
void limiting_average_filter_update(limiting_average_filter_struct * filter)
{

    if(filter -> limit_flag == 0)
    {
        filter -> limit_flag = 1;
        filter -> last_data = filter -> data;
    }
    if (filter -> data - filter -> last_data > filter ->limit || filter -> data - filter -> last_data < (-1) * filter ->limit)
    {
        filter -> data = filter -> last_data;
    }
    else
    {
        filter -> last_data = filter -> data;
    }

    if (filter -> flag == 0)
    {
        filter -> value_buf[filter -> count] = filter -> data;
        filter -> count++;
        if (filter -> count == filter -> times)
        {
            filter -> flag = 1;
        }
        else
        {
            return;
        }
    }
    else
    {
        int i;
        for (i = 0; i < filter -> times - 1; i++)
        {
            filter -> value_buf[i] = filter -> value_buf[i + 1];
        }
        filter -> value_buf[filter -> times - 1] = filter -> data;

    }
    if (filter -> flag == 1)
    {
        int i;
        int32 total = 0;
        for (i = 0; i < filter -> times; i++)
        {
            total += filter -> value_buf[i];
        }
        filter -> output = total / filter -> times;
    }
}
/*------------------------------限幅平均滤波器------------------------------*/

/*------------------------------一阶滞后滤波器------------------------------*/
//优点：对周期性干扰具有良好的抑制作用，适用于波动频率较高的场合
//缺点：相位滞后，灵敏度低，滞后程度取决于a值大小，不能消除滤波频率高于采样频率的1/2的干扰信号
//一阶滞后滤波器结构体声明
//示例：first_order_lag_filter_struct filter;

//一阶滞后滤波器初始化
//示例：first_order_lag_filter_init(&filter,0.5);
void first_order_lag_filter_init(first_order_lag_filter_struct * filter,float ratio)
{
    filter -> ratio = ratio;
    filter -> flag = 0;
}

//一阶滞后滤波器更新
//示例：first_order_lag_filter_update(&filter);
void first_order_lag_filter_update(first_order_lag_filter_struct * filter)
{
    if(filter -> flag == 0)
    {
        filter -> flag = 1;
        filter -> output = filter -> data;
        filter -> last_data = filter -> output;
    }
    else
    {
        filter -> output = filter -> data * (1 - filter -> ratio) + filter -> last_data * filter -> ratio;
        filter -> last_data = filter -> output;
    }
}
/*------------------------------一阶滞后滤波器------------------------------*/

/*------------------------------加权递推平均滤波器------------------------------*/
//优点：适用于有较大纯滞后时间常数的对象，和采样周期较短的系统
//缺点：对于纯滞后时间常数较小，采样周期较长，变化缓慢的信号，不能迅速反应系统当前所受干扰的严重程度，滤波效果差
//加权递推平均滤波器结构体声明
//示例：weighted_recursive_average_filter_struct filter;

//加权递推平均滤波器初始化
//示例：float duty_arr[] = {0.1,0.2,0.3,0.4};weighted_recursive_average_filter_init(&filter,duty_arr,4);
void weighted_recursive_average_filter_init(weighted_recursive_average_filter_struct * filter,float * duty,int times)
{
    int i;
    for (i = 0; i < times; i++)
    {
        filter -> duty[i] = duty[i];
    }
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//加权递推平均滤波器更新
//示例：weighted_recursive_average_filter_update(&filter);
void weighted_recursive_average_filter_update(weighted_recursive_average_filter_struct * filter)
{
    if (filter -> flag == 0)
    {
        filter -> value_buf[filter -> count] = filter -> data;
        filter -> count++;
        if (filter -> count == filter -> times)
        {
            filter -> flag = 1;
        }
        else
        {
            return;
        }
    }
    else
    {
        int i;
        for (i = 0; i < filter -> times - 1; i++)
        {
            filter -> value_buf[i] = filter -> value_buf[i + 1];
        }
        filter -> value_buf[filter -> times - 1] = filter -> data;
    }
    if (filter -> flag == 1)
    {
        int i;
        filter -> output = 0;
        for (i = 0; i < filter -> times; i++)
        {
            filter -> output += filter -> value_buf[i] * filter -> duty[i];
        }
    }
}
/*------------------------------加权递推平均滤波器------------------------------*/

/*------------------------------消抖滤波器------------------------------*/
//优点：对于变化缓慢的被测参数有较好的滤波效果，可避免在临界值附近控制器的反复开/关跳动或显示器上数值抖动
//缺点：对于快速变化的参数不宜，如果在计数器溢出的那一次采样到的值恰好是干扰值,则会将干扰值当作有效值导入系统
//消抖滤波器结构体声明
//示例：debounce_filter_struct filter;

//消抖滤波器初始化
//示例：debounce_filter_init(&filter,5);
void debounce_filter_init(debounce_filter_struct * filter,int times)
{
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//消抖滤波器更新
//示例：debounce_filter_update(&filter);
void debounce_filter_update(debounce_filter_struct * filter)
{
    if (filter -> flag == 0)
    {
        filter -> output = filter -> data;
        filter -> flag = 1;
    }
    else
    {
        if(filter -> output == filter -> data)
        {
            filter -> count = 0;
        }
        else
        {
            filter -> count++;
        }
        if(filter -> count >= filter -> times)
        {
            filter -> output = filter -> data;
            filter -> count = 0;
        }
    }
}
/*------------------------------消抖滤波器------------------------------*/

/*------------------------------限幅消抖滤波器------------------------------*/
//优点：继承了“限幅”和“消抖”的优点，改进了“消抖滤波法”中的某些缺陷,避免将干扰值导入系统
//缺点： 对于快速变化的参数不宜
//限幅消抖滤波器结构体声明
//示例：limiting_and_debounce_filter_struct filter;

//限幅消抖滤波器初始化
//示例：limiting_and_debounce_filter_init(&filter,5,100);
void limiting_and_debounce_filter_init(limiting_and_debounce_filter_struct * filter,int times,int32 limit)
{
    filter -> limit = limit;
    filter -> times = times;
    filter -> count = 0;
    filter -> flag = 0;
}

//限幅消抖滤波器更新
//示例：limiting_and_debounce_filter_update(&filter);
void limiting_and_debounce_filter_update(limiting_and_debounce_filter_struct * filter)
{
    if(filter -> flag == 0)
    {
        filter -> flag = 1;
        filter -> output = filter -> data;
        filter -> last_data = filter -> data;
        return;
    }
    int32 temp;
    if (filter -> data - filter -> last_data > filter ->limit || filter -> data - filter -> last_data < (-1) * filter ->limit)
    {
        temp = filter -> last_data;
    }
    else
    {
        temp = filter -> data;
        filter -> last_data = filter -> data;
    }
    if(filter -> output == temp)
    {
        filter -> count = 0;
    }
    else
    {
        filter -> count++;
    }
    if(filter -> count >= filter -> times)
    {
        filter -> output = temp;
        filter -> count = 0;
    }
}
/*------------------------------限幅消抖滤波器------------------------------*/
