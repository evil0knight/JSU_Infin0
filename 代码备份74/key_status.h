/*
 * key_status.h
 *
 *  Created on: 2024年9月30日
 *      Author: 17104
 */

#ifndef CODE_KEY_STATUS_H_
#define CODE_KEY_STATUS_H_

#include "zf_common_headfile.h"
/*定义按键功能*/
#define button_back (P20_6) /*20_6返回上一级目录*/
#define button_next (P11_3) /*20_7前往下一级目录*/
#define button_up   (P11_2) /*11_2上一个*/
#define button_down (P20_7) /*11_3下一个*/

typedef enum{
    key_pressed,
    key_released
}key_status_enum;

typedef struct
{
    gpio_pin_enum button_prot;
    key_status_enum key_last;
    key_status_enum key_output;
}key_struct;



void my_key_init(void);
void scan_key_status(key_struct * key);
int get_key_status(void);
void scankey(void);


#endif /* CODE_KEY_STATUS_H_ */
