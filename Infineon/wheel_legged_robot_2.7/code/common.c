/*
 * common.c
 *
 *  Created on: 2024Äê10ÔÂ7ÈÕ
 *      Author: A
 */

#include "headfile.h"

int32_t limit(int32_t x, int32_t low, int32_t up)
{
    return x > up ? up : x < low ? low
                                 : x;
}
int clip(int x, int low, int up)
{
    return x > up ? up : x < low ? low
                                 : x;
}

float fclip(float x, float low, float up)
{
    return x > up ? up : x < low ? low
                                 : x;
}

float Q_sqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = *(long *)&y;           // evil floating point bit level hacking
    i  = 0x5f3759df - (i >> 1); // what the fuck?
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y)); // 1st iteration
    y  = y * (threehalfs - (x2 * y * y)); // 2nd iteration, this can be removed

    return (1.0 / y);
}


