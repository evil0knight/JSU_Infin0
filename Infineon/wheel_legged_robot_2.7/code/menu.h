/*
 * menu.h
 *
 *  Created on: 2024年11月25日
 *      Author: 17104
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"
#include <database.h>
#include <universal_filter.h>
#include <Base.h>
#include "enconder.h"
#include "headfile.h"

/*定义鼠标样式*/
#define mouse_look_left  "<-"
#define mouse_look_right "->"

#define dir_num 36   //目录条目总数

#define x_pixel 10
#define y_pixel 20

/*编码目录结构体定义*/
//一级目录：100、200、x00
//二级目录：110、120、1x0
//三级目录：111、112、11x
typedef struct {int id;int choose_status;
                int int_data; int int_step;
                float float_data; float float_step;
                int is_float; char name[128]; int is_show;}directory;

//typedef struct{uint16 first_level_dir;uint16 second_level_dir;uint16 choose_status;}dir;
extern directory all_dir[dir_num];
extern float K_v_sq;
extern float K_v;
extern float K_curvature;
extern float K_lateral_deviation;
extern float K_lateral_deviation_straight;
extern int image;
extern float eva_speed;

void save_raw_data(void);
void reset_as_raw_data(void);
void set_as_flash_data(void);

void menu_init(void);
void print_menu(void);
void menu_run(debounce_filter_struct * filter);

float get_float_by_name(char name[]);
float get_float_by_id(int id);
int get_int_by_name(char name[]);
void change_float_by_name(char name[],float data);
#endif /* CODE_MENU_H_ */
