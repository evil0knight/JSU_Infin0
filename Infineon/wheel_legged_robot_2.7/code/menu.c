/*
 * menu.c
 *
 *  Created on: 2024年11月25日
 *      Author: 17104
 */

#include <menu.h>

/*一级目录不超过12个*/
directory all_dir[dir_num] = {//编号，选择状态=0，整型数据，整型修改步长，浮点型数据，浮点型修改步长，是否为浮点型，目录名（！！！更新目录条目总数！！！）
        {10,0,0,0,0,0,0,"reset_data",0},
        {11,0,0,0,0,0,0,"NO",0},
        {12,0,0,0,0,0,0,"YES",0},

        {20,0,0,0,0,0,0,"RUN",0},
        {21,0,0,0,0,0,0,"STOP",0},
        {22,0,1,1,0,0,0,"TRACK",1},
        {23,0,0,0, 1.6f,   0.1f,1,"speed",1},
        {24,0,0,0, 14.0f,   0.1f,1,"me_zreo",1},
        {25,0,0,0, 0.5f,   0.0f,1,"ave_spd",1},
        {26,0,0,0, 0.5f,   0.0f,1,"max_spd",1},
        {27,0,0,0, 0.0f,   0.0f,1,"diaplace",1},


        {30,0,0,0,0,0,0,"update_LQR",1},
        {31,0,0,0,0,0,0,"NO",0},
        {32,0,0,0,0,0,0,"YES",0},

        {40,0,0,0,0,0,0,"Q",0},
        {41,0,0,0, 0,   1.0f,1,"x",1},
        {42,0,0,0, 2,  1.0f,1,"v",1},
        {43,0,0,0, 7, 1.0f,1,"roll",1},
        {44,0,0,0, 2,  0.1f,1,"gx",1},
        {45,0,0,0, 0.38f,0.001f,1,"c",1},
        {46,0,0,0, 0.05f,0.001f,1,"gz",1},
        {47,0,0,0, 1,   0.1f,1,"leg",1},

        {50,0,0,0,0,0,0,"R",0},
        {51,0,0,0, 1, 0.01f,1,"TL",1},
        {52,0,0,0, 1, 0.01f,1,"TR",1},
        {53,0,0,0, 15, 1.0f,1,"a",1},
        {54,0,0,0, 1,   0.1f,1,"legL",1},
        {55,0,0,0, 1,   0.1f,1,"legR",1},
        //-0.5,-60
        {60,0,0,0, 0,0,0,"aim",0},
        {61,0,0,0, 0.19f, 0.01f,1,"K_v_sq",1},
        {62,0,0,0, 0.12f, 0.01f,1,"K_v",1},
        {63,0,0,0, 0.027f, 0.01f,1,"K_curv",1},
        {64,0,0,0, 0.0f, 0.01f,1,"K_devi",1},
        {65,0,0,0, 0.4f, 0.01f,1,"K_devi_stra",1},

        {70,0,0,0, 0,0,0,"camera",0},
        {71,0,0,0, 0,0,0,"",0}
};

flash_data_union database_buffer[12][data_num * 2];//数据库二级缓冲区
flash_data_union raw_data[12][data_num * 2];//烧录数据储存

int mouse;//鼠标位置编号
int print_height = 0;//当前显示行数
int have_chosen = 0;//二级目录鼠标选择状态
int prev_have_chosen = 0;
int have_chosen_rising = 0;
int image = 0;
//保留烧录数据
//示例：save_raw_data();
void save_raw_data(void)
{
    int i,page,order_num;
    for(i = 0; i < dir_num; i++)
    {
        page = all_dir[i].id / 10;
        order_num = all_dir[i].id % 10;
        if(all_dir[i].is_float)//将数据存储到raw_data[][]中
        {
            raw_data[page][order_num * 2].float_type = all_dir[i].float_data;
            raw_data[page][order_num * 2 + 1].int32_type = all_dir[i].is_float;
        }
        else
        {
            raw_data[page][order_num * 2].int32_type = all_dir[i].int_data;
            raw_data[page][order_num * 2 + 1].int32_type = all_dir[i].is_float;
        }
    }
}

//重置为原始数据
//示例：reset_as_raw_data();
void reset_as_raw_data(void)
{
    clear_all_data();
    int i,page,order_num;
    for(i = 0; i < dir_num; i++)//将raw_data[][]中数据覆写到all_dir[]和flash中
    {
        page = all_dir[i].id / 10;
        order_num = all_dir[i].id % 10;
        if(raw_data[page][order_num * 2 + 1].int32_type)
        {
            all_dir[i].float_data = raw_data[page][order_num * 2].float_type;
            all_dir[i].is_float = raw_data[page][order_num * 2 + 1].int32_type;
            write_database(page,order_num * 2,&raw_data[page][order_num * 2].float_type,raw_data[page][order_num * 2 + 1].int32_type);
            write_database(page,order_num * 2 + 1,&raw_data[page][order_num * 2 + 1].int32_type,0);
        }
        else
        {
            all_dir[i].int_data = raw_data[page][order_num * 2].int32_type;
            all_dir[i].is_float = raw_data[page][order_num * 2 + 1].int32_type;
            write_database(page,order_num * 2,&raw_data[page][order_num * 2].int32_type,raw_data[page][order_num * 2 + 1].int32_type);
            write_database(page,order_num * 2 + 1,&raw_data[page][order_num * 2 + 1].int32_type,0);
        }
    }
}

//设为flash中存储的数据
//示例：set_as_flash_data();
void set_as_flash_data(void)
{
    int i,page,order_num;
    for(i = 0; i < dir_num; i++)//将flash中数据覆写到all_dir[]中
    {
        page = all_dir[i].id / 10;
        order_num = all_dir[i].id % 10;
        if(read_database_int(page,order_num * 2 + 1))
        {
            all_dir[i].float_data = read_database_float(page,order_num * 2);
            all_dir[i].is_float = read_database_int(page,order_num * 2 + 1);
        }
        else
        {
            all_dir[i].int_data = read_database_int(page,order_num * 2);
            all_dir[i].is_float = read_database_int(page,order_num * 2 + 1);
        }
    }
}

//显示目录（内部调用）
//示例：print_menu();
void print_menu(void)
{
    int first_dir = mouse / 10;//鼠标一级目录位置编号
    int second_dir = mouse % 10;//鼠标二级目录位置编号
    int i;//循环
    uint16 dir_height;//打印高度
    print_height = 0;

    if(prev_have_chosen == 0 && have_chosen == 1)have_chosen_rising = 1;
    else have_chosen_rising = 0;

    if(second_dir == 0)//一级目录显示
    {
        ips200_clear();
        dir_height = 2;
        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，显示一级目录目录名
        {
            if(all_dir[i].id % 10 == 0)
            {
                ips200_show_string(0, dir_height, all_dir[i].name);
                if(all_dir[i].is_float && all_dir[i].is_show)
                {
                    ips200_show_float(150, dir_height, all_dir[i].float_data, 3, 5);
                }
                else if (all_dir[i].is_show)
                {
                    ips200_show_int(150, dir_height, all_dir[i].int_data, 6);
                }
                dir_height += 20;
                print_height ++;
            }
        }
        dir_height = 2 + (first_dir - 1) * 20;//鼠标显示
        ips200_show_string(120, dir_height, mouse_look_left);
    }
    else if(second_dir != 0 && have_chosen == 0 && image != 1)//二级目录显示整页
    {
        if(image == 0)
        {
            ips200_clear();
        }
        dir_height = 20;
        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，显示二级目录目录名、数据
        {
            if(all_dir[i].id / 10 == first_dir && all_dir[i].id % 10 == 0)//显示大标题
            {
//                ips200_set_font(IPS200_8X16_FONT);
                ips200_show_string(0, 0, all_dir[i].name);
//                ips200_set_font(TFT180_6X8_FONT);
            }
            else if(all_dir[i].id / 10 == first_dir && all_dir[i].id % 10 != 0)//显示二级目录目录名、数据
            {
                ips200_show_string(0, dir_height, all_dir[i].name);
                if(all_dir[i].is_float && all_dir[i].is_show)
                {
                    ips200_show_float(150, dir_height, all_dir[i].float_data, 3, 5);
                }
                else if (all_dir[i].is_show)
                {
                    ips200_show_int(150, dir_height, all_dir[i].int_data, 8);
                }
                dir_height += 20;
                print_height ++;
            }
        }
        dir_height = 20 * second_dir;//鼠标显示
        ips200_show_string(120, dir_height, mouse_look_left);
    }
    else if(second_dir != 0 && have_chosen == 1 && image != 1)//二级目录更新数据优化
    {
        dir_height = 20 * second_dir;
        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新修改的二级目录数据
        {
            if(all_dir[i].id / 10 == first_dir && all_dir[i].id % 10 == second_dir)
            {
                if(all_dir[i].is_float&& all_dir[i].is_show)
                {
                    ips200_show_float(150, dir_height, all_dir[i].float_data, 3, 5);
                }
                else if (all_dir[i].is_show)
                {
                    ips200_show_int(150, dir_height, all_dir[i].int_data, 8);
                }
                ips200_show_string(120, dir_height, mouse_look_right);//鼠标显示
                break;
            }
        }
    }

    if(first_dir == 2 && second_dir != 0)   //启动页
    {
        dir_height = 20;
        ips200_show_string(150, 20, "RUN");

        if(have_chosen_rising == 1 && second_dir == 1)
        {
            input.max_speed = get_float_by_name("speed");
            input.Car_zero = get_float_by_name("me_zreo");
            input.track = get_int_by_name("TRACK");
            input.flag = 1;
            input.actual_target_speed = 0;
            K_v_sq = get_float_by_name("K_v_sq");
            K_v = get_float_by_name("K_v");
            K_curvature = get_float_by_name("K_curv");
            K_lateral_deviation = get_float_by_name("K_devi");
            K_lateral_deviation_straight = get_float_by_name("K_devi_stra");
            ips200_show_string(120, dir_height, mouse_look_right);
        }
        else if(have_chosen == 0 && second_dir == 1)
        {
            change_float_by_name("ave_spd",eva_speed);
            change_float_by_name("max_spd",max_speed);
            change_float_by_name("diaplace",vel_kf.est_displacement);

            input.flag = 0;
            ips200_show_string(120, dir_height, mouse_look_left);

        }
    }
    else if(first_dir == 7 && second_dir != 0)   //图像页
    {
//        printf("777\n");
        dir_height = 20;
        input.flag = 0;
        image = 1;

        K_v_sq = get_float_by_name("K_v_sq");
        K_v = get_float_by_name("K_v");
        K_curvature = get_float_by_name("K_curv");
        K_lateral_deviation = get_float_by_name("K_devi");

        ips200_show_string(x_pixel*1, y_pixel*2, "Angle:");
        ips200_show_float (x_pixel*7, y_pixel*2, pure_angle,2,3);
        ips200_show_string(x_pixel*1, y_pixel*3, "State:");
        ips200_show_uint(x_pixel*7, y_pixel*3,(uint16)state_type,3);
        ips200_show_string(x_pixel*1, y_pixel*4, "track:");
        ips200_show_uint(x_pixel*7, y_pixel*4,(uint16)track_type,3);
        ips200_show_string(x_pixel*1, y_pixel*5, "cross:");
        ips200_show_uint(x_pixel*7, y_pixel*5,(uint16)cross_type,3);
        ips200_show_string(x_pixel*1, y_pixel*6, "Time:");
        ips200_show_string(x_pixel*1, y_pixel*7, "pvdt:");
        ips200_show_float (x_pixel*7, y_pixel*7, preview_distance,2,2);
        ips200_show_string(x_pixel*1, y_pixel*8, "delta_y:");
        ips200_show_float (x_pixel*8, y_pixel*8, delta_y,2,2);
//        Atimer_stop(1);
        ips200_show_uint(x_pixel*7, y_pixel*6, Atimer_get(1),3);
        //ips200_show_gray_image(0,y_pixel*9, mt9v03x_image_copy[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, FIX_BINTHRESHOLD);
        ips200_show_string(x_pixel*12, y_pixel*3, "LptL:");
        ips200_show_uint(x_pixel*17, y_pixel*3,left_conf,3);
        ips200_show_string(x_pixel*12, y_pixel*4, "LptR:");
        ips200_show_uint(x_pixel*17, y_pixel*4,right_conf,3);
        ips200_show_string(x_pixel*12, y_pixel*5, "barrier:");
        ips200_show_uint(x_pixel*17, y_pixel*5,(uint16)barrier_type,3);
        ips200_show_string(x_pixel*12, y_pixel*6, "cur_a:");
        ips200_show_float(x_pixel*19, y_pixel*6,standardized_curvature_ave,2,2);
        ips200_show_string(x_pixel*12, y_pixel*7, "devia:");
        ips200_show_float(x_pixel*19, y_pixel*7,lateral_deviation,1,3);
        ips200_show_string(x_pixel*12, y_pixel*8, "dista:");
        ips200_show_float(x_pixel*19, y_pixel*8,transection_distance,1,3);

//        ips200_show_gray_image(0,20, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 1*MT9V03X_W, 1*MT9V03X_H, FIX_BINTHRESHOLD);
        for(uint16 i=0;i<pts_far_right_count;i++){
            ips200_draw_line((uint16)pts_far_right[i][1], (uint16)pts_far_right[i][0]+y_pixel*9,(uint16)pts_far_right[i+1][1],(uint16)pts_far_right[i+1][0]+y_pixel*9, RGB565_BLUE);
        }
        for(uint16 i=0;i<pts_far_left_count;i++){
            ips200_draw_line((uint16)pts_far_left[i][1], (uint16)pts_far_left[i][0]+y_pixel*9,(uint16)pts_far_left[i+1][1],(uint16)pts_far_left[i+1][0]+y_pixel*9, RGB565_GREEN);
        }
        for(uint16 i=0;i<mid_track_count;i++){
            ips200_draw_line((uint16)rptsn[i][1], (uint16)rptsn[i][0]+y_pixel*9,(uint16)rptsn[i+1][1],(uint16)rptsn[i+1][0]+y_pixel*9, RGB565_RED);
        }


    }
    else if(second_dir == 0)
    {
//        input.flag = 0;
        image = 0;
    }
    ips200_show_float(180, 0, input.batter_voltage , 2, 2);
    ips200_show_string(230, 0, "V");
    prev_have_chosen = have_chosen;
}

//屏幕、目录初始化
//示例：menu_init();
void menu_init(void)
{
    /*ips200 初始化*/
    ips200_init(IPS200_TYPE_SPI);
//    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();
    ips200_set_font(IPS200_8X16_FONT);
    ips200_show_string(0, 0, "loading...");
//    ips200_set_font(IPS200_6X8_FONT);
    /*保留烧录数据*/
    save_raw_data();
    /*设为flash中存储的数据*/
//    set_as_flash_data();
    /*鼠标初始化*/
    mouse = 10;
    /*显示目录*/
    print_menu();

}

//目录更新
//示例：menu_run(&key_filter);
void menu_run(debounce_filter_struct * filter)
{
    int first_dir = mouse / 10;//鼠标一级目录位置编号
    int second_dir = mouse % 10;//鼠标二级目录位置编号
    int i;//循环
    int page,order_num;
    if(second_dir == 0)//一级目录
    {
        if(filter -> output == 1)//up，鼠标移动
        {
            if(mouse < print_height*10)
            {
                mouse += 10;
            }
            else
            {
                mouse = 10;
            }
        }
        else if(filter -> output == 2)//down，鼠标移动
        {
            if(mouse > 10)
            {
                mouse -= 10;
            }
            else
            {
                mouse = print_height*10;
            }
        }
        else if(filter -> output == 3)//next，鼠标移动
        {
            for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
            {
                if(all_dir[i].id / 10 == first_dir)
                {
                    all_dir[i].choose_status = 1;
                    break;
                }
            }
            mouse += 1;
        }
    }
    else if(second_dir != 0 && have_chosen == 0)//二级目录
    {
        if(filter -> output == 1)//up，鼠标移动
        {
            if(mouse < first_dir * 10 + print_height)
            {
                mouse += 1;
            }
            else
            {
                mouse = first_dir * 10 + 1;
            }
        }
        else if(filter -> output == 2)//down，鼠标移动
        {
            if(mouse > first_dir * 10 + 1)
            {
                mouse -= 1;
            }
            else
            {
                mouse = first_dir * 10 + print_height;
            }
        }
        else if(filter -> output == 3)//next，鼠标移动
        {
            for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    all_dir[i].choose_status = 1;
                    have_chosen = 1;
                    break;
                }
            }
        }
        else if(filter -> output == 4)//back，鼠标移动，存储数据
        {
            ips200_clear();
//            ips200_set_font(IPS200_8X16_FONT);
            ips200_show_string(0, 0, "data saving...");
//            ips200_set_font(IPS200_8X16_FONT);
            mouse = first_dir * 10;
            for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态，存储数据
            {

                if(all_dir[i].id / 10 == first_dir)
                {
                    all_dir[i].choose_status = 0;

                    page = first_dir;
                    order_num = all_dir[i].id % 10;
                    if(all_dir[i].is_float)
                    {
                        write_database(page,order_num * 2,&all_dir[i].float_data,all_dir[i].is_float);
                        write_database(page,order_num * 2 + 1,&all_dir[i].is_float,0);
                    }
                    else
                    {
                        write_database(page,order_num * 2,&all_dir[i].int_data,all_dir[i].is_float);
                        write_database(page,order_num * 2 + 1,&all_dir[i].is_float,0);
                    }

                }
                else if(all_dir[i].id / 10 == first_dir + 1)
                {
                    break;
                }
            }
        }
        else if(filter -> output == 5)//正转,被选中的结构体的数值加
        {
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    if(all_dir[i].is_float) all_dir[i].float_data += all_dir[i].float_step;
                    else all_dir[i].int_data += all_dir[i].int_step;
                    break;
                }
            }
        }
        else if(filter -> output == 6)//反转,被选中的结构体的数值减
        {
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    if(all_dir[i].is_float) all_dir[i].float_data -= all_dir[i].float_step;
                    else all_dir[i].int_data -= all_dir[i].int_step;
                    break;
                }
            }
        }
    }
    else if(second_dir != 0 && have_chosen == 1)//二级目录数据修改
    {
        if(filter -> output == 1)//up,被选中的结构体的数值加
        {
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    if(all_dir[i].is_float) all_dir[i].float_data -= all_dir[i].float_step;
                    else all_dir[i].int_data -= all_dir[i].int_step;
                    break;
                }
            }
        }
        else if(filter -> output == 2)//down,被选中的结构体的数值减
        {
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    if(all_dir[i].is_float) all_dir[i].float_data += all_dir[i].float_step;
                    else all_dir[i].int_data += all_dir[i].int_step;
                    break;
                }
            }
        }
        else if(filter -> output == 4)//back，鼠标移动
        {
            for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
            {
                if(all_dir[i].id % 10 == second_dir && all_dir[i].id / 10 == first_dir)
                {
                    all_dir[i].choose_status = 0;
                    have_chosen = 0;
                    break;
                }
            }
        }
    }
    print_menu();
}

//获取浮点型数据
//示例：float output; output = get_float_by_name("chioce21");
float get_float_by_name(char name[])
{
    float output;
    int i;
    for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
    {
        if(strcmp(all_dir[i].name, name) == 0)
        {
            output = all_dir[i].float_data;
            break;
        }
    }
    return output;
}

float get_float_by_id(int id)
{
    float output;
    int i;
    for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
    {
        if(all_dir[i].id == id)
        {
            output = all_dir[i].float_data;
            break;
        }
    }
    return output;
}

//获取整型数据
//示例：int output; output = get_int_by_name("chioce23");
int get_int_by_name(char name[])
{
    int output, i;
    for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
    {
        if(strcmp(all_dir[i].name, name) == 0)
        {
            output = all_dir[i].int_data;
            break;
        }
    }
    return output;
}

void change_float_by_name(char name[],float data)
{
    int i;
    for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新被选中的结构体的选择状态
    {
        if(strcmp(all_dir[i].name, name) == 0)
        {
            all_dir[i].float_data = data;
            break;
        }
    }
}
