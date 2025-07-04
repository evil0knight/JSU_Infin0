/*
 * ksy_status.c
 *
 *  Created on: 2024年9月30日
 *      Author: 17104
 */

#include "zf_common_headfile.h"

key_struct key_up   = {0};
key_struct key_down = {0};
key_struct key_next = {0};
key_struct key_back = {0};

void my_key_init(void)
{
    /*获取按键状态*/
    gpio_init(button_back, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(button_next, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(button_up  , GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(button_down, GPI, GPIO_HIGH, GPI_PULL_UP);

    /*key_up.button_prot = button_up;
    key_down.button_prot = button_down;
    key_next.button_prot = button_next;
    key_back.button_prot = button_back;
    */
}

void scan_key_status(key_struct * key)
{
    key_status_enum key_now = gpio_get_level(key->button_prot);
    key -> key_output = (key->key_last == key_now && key_now == key_pressed)?key_pressed:key_released;
    key -> key_last = key_now;
}

int get_key_status(void)
{

    scan_key_status(&key_up);
    scan_key_status(&key_down);
    scan_key_status(&key_next);
    scan_key_status(&key_back);

    if(key_up.key_output == key_pressed)
    {
        return 1;
    }
    if(key_down.key_output == key_pressed)
    {
        return 2;
    }
    if(key_next.key_output == key_pressed)
    {
        return 3;
    }
    if(key_back.key_output == key_pressed)
    {
        return 4;
    }
    return 0;
}
void scankey(void)
{
    if( gpio_get_level(button_up) == 0 )//如果P33_11引脚电平为0,控制i
     {
         system_delay_ms(50);//延迟消抖
         if( gpio_get_level(button_up) == 0 )
         {
             mode=stopworking;
             small_driver_set_duty(0,0);//站立串级pid设置电机转速
         }
     }
     if( gpio_get_level(button_down) == 0 )//如果P20_8引脚电平为0,控制mode
     {
         system_delay_ms(50);//延迟消抖
         if( gpio_get_level(button_down) == 0 )
         {
             mode=walk;
         }
     }
}

