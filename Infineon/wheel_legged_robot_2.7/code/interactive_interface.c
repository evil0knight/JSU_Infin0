/*
 * interactive_interface.c
 *
 *  Created on: 2024年11月24日
 *      Author: 17104
 */

#include <interactive_interface.h>

debounce_filter_struct key_filter;//消抖滤波器结构体声明
//extern encoder_struct encoder;
int counter = 0;

void interactive_interface_init(void)
{
    my_key_init();//按键初始化
    debounce_filter_init(&key_filter, 2);//消抖滤波器初始化
    my_encoder_init();
    menu_init();//目录初始化
}

void interactive_interface_run(void)
{
    key_filter.data = get_key_status();//获取按键状态
    if(key_filter.data == 0)
    {
        key_filter.data = get_encoder_status();
    }
    debounce_filter_update(&key_filter);//消抖滤波器更新

//    printf("%d\n",key_filter.output);
    /*下降沿长按连续触发（仅限up、down）*/
    if(key_filter.output == key_filter.data && key_filter.data != 0)
    {
        counter ++;
        if(counter > 500 / pit_time0_ms && (key_filter.data == 1 || key_filter.data == 2))//触发时长500ms
        {
            menu_run(&key_filter);
        }
    }
    else
    {
        counter = 0;
    }
//    printf("counter:%d,%d\n",counter,key_filter.data);
    /*下降沿单击触发*/
    if(key_filter.output != 0 && key_filter.data == 0)
    {
        menu_run(&key_filter);

        /*数据重置*/
        if(key_filter.output == 3)
        {
            int i;
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id == 10)
                {
                    if(all_dir[i + 2].choose_status == 1)
                    {
                        ips200_clear();
                        ips200_set_font(IPS200_8X16_FONT);
                        ips200_show_string(0, 0, "data reset");
                        ips200_show_string(0, 20, "loading...");
//                        ips200_set_font(IPS200_8X16_FONT);

                        reset_as_raw_data();

                        mouse = 10;
                        int i;
                        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新修改的二级目录数据
                        {
                            all_dir[i].choose_status = 0;
                            have_chosen = 0;
                        }
                        print_menu();

                    }
                    else if (all_dir[i + 1].choose_status == 1)
                    {
                        mouse = 10;
                        int i;
                        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新修改的二级目录数据
                        {
                            all_dir[i].choose_status = 0;
                            have_chosen = 0;
                        }
                        print_menu();
                    }
                    break;
                }
            }
        }
        /*LQR*/
        if(key_filter.output == 3)
        {
            int i;
            for(i = 0; i < dir_num; i++)
            {
                if(all_dir[i].id == 30)
                {
                    if(all_dir[i + 2].choose_status == 1)
                    {
                        ips200_clear();
                        ips200_set_font(IPS200_8X16_FONT);
                        ips200_show_string(0, 0, "update_LQR");
                        ips200_show_string(0, 20, "loading...");
//                        ips200_set_font(TFT180_6X8_FONT);

                        float Q_data[7] ={ 0 };
                        int j;
                        for(j = 0; j < 7; j++)
                        {
                            Q_data[j] = get_float_by_id(41 + j);
                            printf("%f\n",Q_data[j]);

                        }
                        float R_data[5] = { 0 };
                        for(j = 0; j < 5; j++)
                        {
                            R_data[j] = get_float_by_id(51 + j);
                            printf("%f\n",R_data[j]);

                        }
//                        LQR_AB_init(&A, &B);
//                        LQR_QR_init(&LS_STRAIGHT_Q, &Q_data[0], &LS_STRAIGHT_R, &R_data[0]);
//                        int count = LQR_K_update(&LS_STRAIGHT_Q, &LS_STRAIGHT_R, &LS_STRAIGHT_K);

//                        for(j = 0; j < dir_num; j++)//遍历目录结构体数组，更新被选中的结构体的选择状态
//                        {
//                            if(strcmp(all_dir[j].name, "update_LQR") == 0)
//                            {
////                                all_dir[j].int_data = count;
//                                break;
//                            }
//                        }
                        LQR_AB_init(&A, &B);
                        LQR_QR_init(&COMMON_Q, &Q_data[0], &COMMON_R, &R_data[0]);
                        int count = LQR_K_update(&COMMON_Q, &COMMON_R, &COMMON_K);
                        for(j = 0; j < dir_num; j++)//遍历目录结构体数组，更新被选中的结构体的选择状态
                        {
                            if(strcmp(all_dir[j].name, "update_LQR") == 0)
                            {
                                all_dir[j].int_data = count;
                                break;
                            }
                        }
                        get_controller_gains(&COMMON_K, KL, KR, Ka);

                        Q_data[2] *= 0.5f;
                        Q_data[3] *= 2.0f;
                        Q_data[4] *= 0.5f;
                        LQR_QR_init(&COMMON_Q, &Q_data[0], &COMMON_R, &R_data[0]);
                        LQR_K_update(&COMMON_Q, &COMMON_R, &STABLE_K);

                        mouse = 10;
                        int i;
                        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新修改的二级目录数据
                        {
                            all_dir[i].choose_status = 0;
                            have_chosen = 0;
                        }
                        print_menu();

                    }
                    else if (all_dir[i + 1].choose_status == 1)
                    {
                        mouse = 10;
                        int i;
                        for(i = 0; i < dir_num; i++)//遍历目录结构体数组，更新修改的二级目录数据
                        {
                            all_dir[i].choose_status = 0;
                            have_chosen = 0;
                        }
                        print_menu();
                    }
                    break;
                }
            }
        }
    }
    /*图像显示触发*/
    if(image == 2)
    {
        print_menu();
    }
}
