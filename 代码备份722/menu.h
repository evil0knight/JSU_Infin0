#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"
extern int place_index;
extern int value_index;
extern int last_place_index;
extern int allow_value_show ;
extern int allow_image_show ;
extern int key_pit_flag;
#define Font_size_H 16
#define Font_size_W 8
void adjust_menu(void);
void control_menu(void);
void image_menu(void);
void function_menu(void);
void image_menu_2(void);
void image_menu_3(void);
void image_menu_4(void);
void image_menu_5(void);
void show_string_value(uint16 base_y, uint32 value, uint8 num_digits, const char* str) ;
void show_string_value_float(uint16 base_y, float value,uint8 num, uint8 pointnum, const char* str) ;
#endif /* CODE_MENU_H_ */
