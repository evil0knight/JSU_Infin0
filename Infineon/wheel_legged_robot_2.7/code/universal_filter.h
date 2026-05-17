/*
 * universal_filter.h
 *
 *  Created on: 2024年11月17日
 *      Author: 17104
 */

#ifndef CODE_UNIVERSAL_FILTER_H_
#define CODE_UNIVERSAL_FILTER_H_

#include "zf_common_headfile.h"

/*------------------------------限幅滤波器------------------------------*/
typedef struct
{
    int32 limit;
    int32 data;
    int32 last_data;
    int32 output;
    int flag;
}limiting_filter_struct;

void limiting_filter_init(limiting_filter_struct * filter,int32 limit);
void limiting_filter_update(limiting_filter_struct * filter);

/*------------------------------中位值滤波器------------------------------*/
typedef struct
{
    int32 data;
    int32 value_buf[31];
    int times;
    int count;
    int32 output;
    int flag;
}median_value_filter_struct;

void median_value_filter_init(median_value_filter_struct * filter,int times);
void median_value_filter_update(median_value_filter_struct * filter);

/*------------------------------算数平均滤波器------------------------------*/
typedef struct
{
    int32 data;
    int32 value_buf[32];
    int times;
    int count;
    int32 output;
    int flag;
}arithmetic_mean_filter_struct;

void arithmetic_mean_filter_init(arithmetic_mean_filter_struct * filter,int times);
void arithmetic_mean_filter_update(arithmetic_mean_filter_struct * filter);

/*------------------------------递推平均滤波器（滑动平均滤波器）------------------------------*/
typedef struct
{
    int32 data;
    int32 value_buf[32];
    int times;
    int count;
    int32 output;
    int flag;
}recursive_average_filter_struct;

void recursive_average_filter_init(recursive_average_filter_struct * filter,int times);
void recursive_average_filter_update(recursive_average_filter_struct * filter);

/*------------------------------中位值平均滤波器------------------------------*/
typedef struct
{
    int32 data;
    int32 value_buf[32];
    int times;
    int count;
    int32 output;
    int flag;
}median_average_filter_struct;

void median_average_filter_init(median_average_filter_struct * filter,int times);
void median_average_filter_update(median_average_filter_struct * filter);

/*------------------------------限幅平均滤波器------------------------------*/
typedef struct
{
    int32 limit;
    int32 last_data;
    int32 data;
    int32 value_buf[32];
    int times;
    int count;
    int32 output;
    int limit_flag;
    int flag;
}limiting_average_filter_struct;

void limiting_average_filter_init(limiting_average_filter_struct * filter,int times,int32 limit);
void limiting_average_filter_update(limiting_average_filter_struct * filter);

/*------------------------------一阶滞后滤波器------------------------------*/
typedef struct
{
    float ratio;
    int32 data;
    int32 last_data;
    int32 output;
    int flag;
}first_order_lag_filter_struct;

void first_order_lag_filter_init(first_order_lag_filter_struct * filter,float ratio);
void first_order_lag_filter_update(first_order_lag_filter_struct * filter);

/*------------------------------加权递推平均滤波器------------------------------*/
typedef struct
{
    float duty[32];
    int32 data;
    int32 value_buf[32];
    int times;
    int count;
    int32 output;
    int flag;
}weighted_recursive_average_filter_struct;

void weighted_recursive_average_filter_init(weighted_recursive_average_filter_struct * filter,float * duty,int times);
void weighted_recursive_average_filter_update(weighted_recursive_average_filter_struct * filter);

/*------------------------------消抖滤波器------------------------------*/
typedef struct
{
    int32 data;
    int times;
    int count;
    int32 output;
    int flag;
}debounce_filter_struct;

void debounce_filter_init(debounce_filter_struct * filter,int times);
void debounce_filter_update(debounce_filter_struct * filter);

/*------------------------------限幅消抖滤波器------------------------------*/
typedef struct
{
    int32 limit;
    int32 data;
    int32 last_data;
    int32 output;
    int times;
    int count;
    int flag;
}limiting_and_debounce_filter_struct;

void limiting_and_debounce_filter_init(limiting_and_debounce_filter_struct * filter,int times,int32 limit);
void limiting_and_debounce_filter_update(limiting_and_debounce_filter_struct * filter);


#endif /* CODE_UNIVERSAL_FILTER_H_ */
